#ifndef PID_AUTOTUNER_H
#define PID_AUTOTUNER_H

#include <Arduino.h>

// ============================================================================
// PID_AutoTuner — Relay / Ziegler-Nichols Auto-Tuner cho UAV ESP32-C6
// Phương pháp: Relay Feedback có Hysteresis (Åström-Hägglund)
// - Áp relay bang-bang hồi tiếp âm (restoring) trên 1 trục tại một thời điểm
// - Hysteresis chống chập chờn zero-crossing do nhiễu sensor
// - Đo chu kỳ dao động Tu (s), biên độ tốc độ góc A (rad/s)
// - Tính Ku = 4*d / (pi * sqrt(A^2 - eps^2))
// - Xuất bộ số ZN Standard (Kp, Ki, Kd), ZN Smooth (50% damping), và Angle P
// - Watchdog an toàn: góc nghiêng > 25° hoặc timeout 3.5s tự động ABORT
// ============================================================================

enum TuneAxis {
    TUNE_NONE = 0,
    TUNE_ROLL_RATE,
    TUNE_PITCH_RATE,
    TUNE_YAW_RATE,
    TUNE_ANGLE_P       // Đánh giá vòng ngoài Angle P
};

enum TuneState {
    TUNE_STATE_IDLE = 0,
    TUNE_STATE_RELAY,       // Đang chạy relay, thu thập dữ liệu
    TUNE_STATE_ANALYZING,   // Đủ chu kỳ, đang phân tích
    TUNE_STATE_DONE,        // Kết quả sẵn sàng
    TUNE_STATE_ABORTED      // Bị dừng do góc lớn hoặc timeout
};

enum TuneAbortReason {
    ABORT_NONE = 0,
    ABORT_TILT_LIMIT,       // Nghiêng quá góc an toàn (> 25°)
    ABORT_TIMEOUT,          // Quá thời gian không thấy đổi chiều
    ABORT_USER_STOP,        // Người dùng chủ động dừng
    ABORT_LOW_THROTTLE      // Ga quá thấp không đủ sinh mô-men
};

struct TuneResult {
    float ku;           // Ultimate gain (hệ số tới hạn)
    float tu;           // Ultimate period (chu kỳ tới hạn, giây)
    
    // ZN Standard (Ziegler-Nichols chuẩn)
    float kp;
    float ki;
    float kd;
    
    // ZN Smooth (Damped / Khuyên nghị cho quadcopter giảm rung)
    float kp_smooth;
    float ki_smooth;
    float kd_smooth;
    
    // Outer loop Angle P gợi ý (dựa trên băng thông vòng trong)
    float angle_p;

    bool  valid;
    char  axis_name[12];
    int   cycles_completed;
};

class PID_AutoTuner {
public:
    PID_AutoTuner();

    // Cấu hình các tham số tuner
    void configure(float relay_amplitude = 0.15f,
                   int   min_cycles       = 8,
                   float abort_angle_deg  = 25.0f,
                   float hysteresis_rad   = 0.04f);

    // Bắt đầu tune trục chỉ định.
    // initial_bias: mô-men tích phân PID đang giữ trục đó (bù trọng lực / lệch trọng tâm đã học được).
    // Relay bắt đầu từ mô-men này thay vì 0 => không bị trọng lực kéo lật ngay khi chuyển sang tune.
    void start(TuneAxis axis, float initial_bias = 0.0f);

    // Dừng chủ động
    void stop(TuneAbortReason reason = ABORT_USER_STOP);

    // Chu trình cập nhật 200Hz trong loop bay
    // rate_meas: tốc độ góc (rad/s)
    // angle_abs: độ lệch góc tuyệt đối (độ)
    // dt: chu kỳ (giây)
    // Trả về: mô-men relay [-1..1]
    float update(float rate_meas, float angle_abs, float dt);

    /**
     * @brief Relay quanh MỐC GIỮ GÓC (dùng cái này trong vòng bay):
     * relay đảo chiều theo (rate_meas - rate_hold), với rate_hold = -HOLD_P * axis_angle (kẹp ±HOLD_MAX).
     * Relay thuần theo tốc độ góc không có thông tin góc => mô-men trọng lực / lệch trọng tâm / trễ làm
     * trục đang tune trôi dần rồi lật. Mốc giữ góc (4 rad/s/rad) kéo trục về quanh 0° mà gần như
     * không ảnh hưởng chu kỳ dao động nhanh của relay dùng để đo Ku, Tu.
     * @param rate_meas_rad  tốc độ góc trục đang tune (rad/s)
     * @param axis_angle_deg góc trục đang tune so với mốc (độ; Yaw: sai lệch so với hướng lúc bắt đầu)
     * @param tilt_abs_deg   góc dùng cho ngắt an toàn (độ)
     */
    float updateWithAngleHold(float rate_meas_rad, float axis_angle_deg, float tilt_abs_deg, float dt);
    static constexpr float HOLD_P       = 3.0f;   // rad/s trên rad
    static constexpr float HOLD_MAX_RAD = 0.6f;   // ~34 deg/s
    // Bù lệch (biased relay): mô-men hằng do trọng lực / lệch trọng tâm lớn hơn biên độ relay thì relay
    // ±d quanh 0 không giữ nổi trục. _bias học mô-men đó bằng tích phân chậm của góc trục.
    static constexpr float BIAS_KI      = 0.2f;   // mô-men chuẩn hóa / (rad * s): chậm, không đuổi theo dao động
    static constexpr float BIAS_MAX     = 0.25f;
    float getBias() const { return _bias; }

    // Getters trạng thái
    bool      isActive()   const { return _state == TUNE_STATE_RELAY || _state == TUNE_STATE_ANALYZING; }
    bool      isDone()     const { return _state == TUNE_STATE_DONE; }
    bool      isAborted()  const { return _state == TUNE_STATE_ABORTED; }
    TuneState getState()   const { return _state; }
    TuneAxis  getAxis()    const { return _axis; }
    TuneAbortReason getAbortReason() const { return _abort_reason; }
    int       getCycleCount() const { return _cycle_count; }
    int       getMinCycles()  const { return _min_cycles; }
    const TuneResult& getResult() const { return _result; }

    // Đóng gói JSON trạng thái cho Telemetry
    int  buildStatusJson(char* buf, int buf_len) const;

    // Reset về IDLE
    void reset();

private:
    TuneState       _state;
    TuneAxis        _axis;
    TuneAbortReason _abort_reason;

    float _relay_amplitude;     // Biên độ relay đang dùng (tự thích nghi, xem _adaptAmplitude)
    float _relay_amp_max;       // Biên độ tối đa người dùng cho phép (lệnh "tune roll <amp>")
    float _bias;                // Mô-men bù lệch (biased relay)
    int   _amp_settle_halves;   // Số nửa chu kỳ liên tiếp biên độ dao động nằm trong dải mục tiêu
    int   _min_cycles;          // Số chu kỳ cần đo (mặc định 8)
    float _abort_angle_deg;     // Góc ngắt an toàn (mặc định 25.0°)
    float _hysteresis_rad;      // Băng trễ chống nhiễu (rad/s)

    // Biến đo lường dao động
    float _relay_sign;          // -1.0 hoặc +1.0
    float _last_transition_time;// Thời điểm đảo relay gần nhất (giây)
    float _time_elapsed;        // Tổng thời gian từ lúc bắt đầu (giây)
    float _half_period_sum;     // Tổng các nửa chu kỳ hợp lệ
    int   _half_period_count;   // Số nửa chu kỳ đã ghi nhận
    int   _cycle_count;         // Số chu kỳ hoàn chỉnh
    float _peak_sum;            // Tổng biên độ đỉnh
    int   _peak_count;          // Số lượng đỉnh đo được
    float _current_half_peak;   // Đỉnh lớn nhất trong nửa chu kỳ hiện tại

    TuneResult _result;

    void _analyze();
    void _resetMeasurement();
    bool _adaptAmplitude(float half_peak);   // true nếu vừa đổi biên độ

public:
    // Dải biên độ dao động tốc độ góc mục tiêu khi tune (rad/s). Relay cố định ±0.10 trên khung nhẹ
    // (độ cứng k >= 700 rad/s^2) cho dao động > 200 deg/s và lật quá 40 deg trước khi đo xong.
    static constexpr float AMP_TARGET_LO = 0.35f;   // ~20 deg/s
    static constexpr float AMP_TARGET_HI = 1.57f;   // ~90 deg/s
    static constexpr float AMP_MIN       = 0.03f;   // dưới mức này relay không đủ quyền giữ trục
    static constexpr float RATE_ABORT    = 4.36f;   // ~250 deg/s: dao động phân kỳ => ngắt sớm
    static constexpr float AMP_START     = 0.03f;
private:
    void _setAxisName();
};

extern PID_AutoTuner autoTuner;

#endif // PID_AUTOTUNER_H

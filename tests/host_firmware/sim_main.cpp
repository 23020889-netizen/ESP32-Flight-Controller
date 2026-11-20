// Kiểm thử vòng kín bộ điều khiển PX4 cascade (Angle P -> Rate PID -> Mixer) của firmware
// bằng mô hình động lực học quay 3 trục đơn giản của quadcopter/giá gimbal 3 trục.
#include "Arduino.h"
#include "../../firmware/dual_imu_esp32c6/px4_pid.h"
#include "../../firmware/dual_imu_esp32c6/pid_autotuner.h"
#include "../../firmware/dual_imu_esp32c6/telemetry_wifi.h"
#include "../../firmware/dual_imu_esp32c6/telemetry_json.h"
#include <vector>
#include <string>

uint32_t g_fake_millis = 0;
FakeSerial Serial;

static int g_fail = 0;
#define CHECK(cond, ...) do { if (!(cond)) { g_fail++; printf("  [FAIL] " __VA_ARGS__); printf("\n"); } \
                              else { printf("  [ OK ] " __VA_ARGS__); printf("\n"); } } while (0)

// ---------------------------------------------------------------------------
// Mô hình: mô-men từ chênh lệch lực đẩy 4 motor (Quad-X, đúng dấu với Mixer firmware),
// động cơ bậc nhất tau_m = 40 ms, ESC cập nhật theo tần số PWM (zero-order hold),
// cảm biến trễ 1 chu kỳ + nhiễu Gyro.
// ---------------------------------------------------------------------------
struct QuadSim {
    double phi = 0, theta = 0, psi = 0;  // rad
    double p = 0, q = 0, r = 0;          // rad/s body
    double motor[4] = {0, 0, 0, 0};      // lực đẩy chuẩn hóa thực tế
    double cmd_hold[4] = {0, 0, 0, 0};   // lệnh ESC đang giữ (ZOH)
    double esc_timer = 0;
    double esc_period = 1.0 / 50.0;      // 50 Hz PWM mặc định của firmware
    double motor_tau = 0.04;
    // Hệ số "mô-men chuẩn hóa -> gia tốc góc" (rad/s^2 cho 1.0 chênh lệch)
    double k_roll = 60.0, k_pitch = 60.0, k_yaw = 12.0;
    double damp = 0.5;                   // ma sát giá gimbal
    double dist[3] = {0, 0, 0};          // nhiễu mô-men ngoài (rad/s^2)

    void step(const float u[4], double dt) {
        esc_timer += dt;
        if (esc_timer >= esc_period) {
            esc_timer -= esc_period;
            for (int i = 0; i < 4; i++) cmd_hold[i] = u[i];
        }
        for (int i = 0; i < 4; i++) motor[i] += (cmd_hold[i] - motor[i]) * (dt / motor_tau);
        // Đảo ngược Mixer: M1=FL(+R+P+Y) M2=RR(-R-P+Y) M3=RL(+R-P-Y) M4=FR(-R+P-Y)
        double tr = (motor[0] - motor[1] + motor[2] - motor[3]) / 4.0;
        double tp = (motor[0] - motor[1] - motor[2] + motor[3]) / 4.0;
        double ty = (motor[0] + motor[1] - motor[2] - motor[3]) / 4.0;
        double pd = k_roll * tr - damp * p + dist[0];
        double qd = k_pitch * tp - damp * q + dist[1];
        double rd = k_yaw * ty - damp * r + dist[2];
        p += pd * dt; q += qd * dt; r += rd * dt;
        // Động học Euler ZYX
        double sphi = sin(phi), cphi = cos(phi), cth = cos(theta), tth = tan(theta);
        double phid = p + (q * sphi + r * cphi) * tth;
        double thd = q * cphi - r * sphi;
        double psid = (q * sphi + r * cphi) / cth;
        phi += phid * dt; theta += thd * dt; psi += psid * dt;
        if (psi > M_PI) psi -= 2 * M_PI;
        if (psi < -M_PI) psi += 2 * M_PI;
    }
};

static double deg(double r) { return r * 180.0 / M_PI; }

struct RunResult { double final_err[3]; double max_abs_after[3]; double max_rate; bool nan; };

// Chạy mô phỏng: setpoint (độ) từ t_step, trả về sai số cuối và dao động sau settle
static RunResult run(PX4_FlightController& fc, QuadSim& sim, float thr_pct,
                     float sp_r, float sp_p, float sp_y, double T, double settle_after,
                     unsigned seed = 1, double gyro_noise_dps = 0.5) {
    RunResult res{};
    const double dt = 0.005;
    fc.setTargetThrottle(thr_pct);
    srand(seed);
    float meas_prev[6] = {0};
    for (double t = 0; t < T; t += dt) {
        g_fake_millis = (uint32_t)(t * 1000);
        fc.setTargetAttitude(sp_r, sp_p, sp_y);
        auto nz = [&]() { return gyro_noise_dps * ((rand() / (double)RAND_MAX) * 2 - 1); };
        // 1 chu kỳ trễ cảm biến
        fc.update(meas_prev[0], meas_prev[1], meas_prev[2], meas_prev[3], meas_prev[4], meas_prev[5],
                  (float)dt, true, true);
        meas_prev[0] = deg(sim.phi); meas_prev[1] = deg(sim.theta); meas_prev[2] = deg(sim.psi);
        meas_prev[3] = deg(sim.p) + nz(); meas_prev[4] = deg(sim.q) + nz(); meas_prev[5] = deg(sim.r) + nz();
        const float* u = fc.getNormalizedMotorOutputs();
        for (int i = 0; i < 4; i++) if (std::isnan(u[i])) res.nan = true;
        sim.step(u, dt);
        if (t > settle_after) {
            double e[3] = { sp_r - deg(sim.phi), sp_p - deg(sim.theta), sp_y - deg(sim.psi) };
            for (int k = 0; k < 3; k++) res.max_abs_after[k] = std::max(res.max_abs_after[k], fabs(e[k]));
        }
        res.max_rate = std::max(res.max_rate, std::max(fabs(deg(sim.p)), fabs(deg(sim.q))));
    }
    res.final_err[0] = sp_r - deg(sim.phi);
    res.final_err[1] = sp_p - deg(sim.theta);
    res.final_err[2] = sp_y - deg(sim.psi);
    return res;
}

static void test_cascade_step_response() {
    printf("\n[1] Dap ung buoc cascade Angle-P -> Rate-PID (ESC 50 Hz, motor tau 40 ms)\n");
    PX4_FlightController fc; fc.init(200.0f);
    QuadSim sim;
    RunResult r = run(fc, sim, 50.0f, 10.0f, -8.0f, 20.0f, 6.0, 3.0);
    CHECK(!r.nan, "khong co NaN o dau ra motor");
    CHECK(fabs(r.final_err[0]) < 1.0, "Roll bam setpoint 10 deg (sai so cuoi %.2f deg)", r.final_err[0]);
    CHECK(fabs(r.final_err[1]) < 1.0, "Pitch bam setpoint -8 deg (sai so cuoi %.2f deg)", r.final_err[1]);
    CHECK(fabs(r.final_err[2]) < 2.0, "Yaw bam setpoint 20 deg (sai so cuoi %.2f deg)", r.final_err[2]);
    CHECK(r.max_abs_after[0] < 1.5 && r.max_abs_after[1] < 1.5,
          "khong dao dong duy tri sau 3 s (|e| max R=%.2f P=%.2f deg)", r.max_abs_after[0], r.max_abs_after[1]);
}

static void test_disturbance_rejection_integral() {
    printf("\n[2] Khu nhieu momen hang (lech trong tam) bang tich phan vong trong\n");
    PX4_FlightController fc; fc.init(200.0f);
    QuadSim sim; sim.dist[0] = 6.0; sim.dist[1] = -4.0;  // rad/s^2 hằng
    RunResult r = run(fc, sim, 50.0f, 0, 0, 0, 10.0, 7.0);
    CHECK(fabs(r.final_err[0]) < 1.0, "Roll ve 0 du co nhieu (sai so %.2f deg, I=%.3f)", r.final_err[0], fc.getRatePidRoll().getI());
    CHECK(fabs(r.final_err[1]) < 1.0, "Pitch ve 0 du co nhieu (sai so %.2f deg, I=%.3f)", r.final_err[1], fc.getRatePidPitch().getI());
}

static void test_anti_windup() {
    printf("\n[3] Anti-windup: bao hoa keo dai roi tha ra\n");
    PX4_RatePID pid; pid.init(0.15f, 0.20f, 0.003f, 0.30f, 20.0f, 200.0f);
    // Sai số lớn liên tục 3 s với giới hạn đầu ra 0.1 (ga thấp)
    for (int i = 0; i < 600; i++) pid.update(3.0f, 0.0f, 0.005f, true, 0.10f);
    CHECK(fabsf(pid.getI()) <= 0.10f + 1e-3f,
          "tich phan khong tran vuot gioi han dau ra khi bao hoa (I=%.3f, out_limit=0.10)", pid.getI());
    // Đảo chiều sai số: đầu ra phải đổi dấu nhanh (< 0.5 s)
    int steps = 0; float out = 1;
    for (; steps < 400; steps++) { out = pid.update(-0.5f, 0.0f, 0.005f, true, 0.10f); if (out < 0) break; }
    CHECK(steps * 0.005 < 0.5, "dau ra dao chieu sau %.3f s khi sai so doi dau", steps * 0.005);
}

static void test_gyro_filter_not_zeroed_when_disarmed() {
    printf("\n[4] Gyro telemetry khi DISARMED (bo loc khong bi reset ve 0 moi chu ky)\n");
    PX4_FlightController fc; fc.init(200.0f);
    for (int i = 0; i < 100; i++) fc.update(0, 0, 0, 50.0f, -30.0f, 10.0f, 0.005f, false, true);
    CHECK(fabsf(fc.getFilteredGx() - 50.0f) < 1.0f && fabsf(fc.getFilteredGy() + 30.0f) < 1.0f,
          "Gyro loc khi disarmed = (%.1f, %.1f, %.1f) ~ (50,-30,10) dps",
          fc.getFilteredGx(), fc.getFilteredGy(), fc.getFilteredGz());
    // ARM: không có xung quá độ (filter đã nạp sẵn mẫu thật)
    fc.reset();
    fc.setTargetThrottle(50);
    fc.update(0, 0, 0, 0.0f, 0.0f, 0.0f, 0.005f, true, true);
    CHECK(fabsf(fc.getFilteredGx()) < 50.0f, "khong nhay buoc khi ARM (gx loc = %.1f)", fc.getFilteredGx());
}

static void test_body_rate_transform() {
    printf("\n[5] Chuyen doi Euler-rate -> Body-rate o goc nghieng lon\n");
    PX4_AttitudeControl att;
    float tgt[3] = { 30.0f, 0.0f, 10.0f }, act[3] = { 30.0f, 0.0f, 0.0f }, out[3];
    att.update(tgt, act, out, true);
    // Chỉ có sai số yaw. Ở roll 30°, psi_dot phải phân bổ vào q và r (q = sin30*psi_dot)
    CHECK(fabsf(out[0]) < 1e-4f, "p = %.4f (khong ro sang roll)", out[0]);
    CHECK(out[1] > 0.0f && fabsf(out[1] / out[2] - tanf(30.0f * DEG_TO_RAD)) < 1e-3f,
          "q/r = %.4f = tan(30deg) = %.4f", out[1] / out[2], tanf(30.0f * DEG_TO_RAD));
    // NaN input
    float nanact[3] = { NAN, 0, 0 };
    att.update(tgt, nanact, out, true);
    CHECK(out[0] == 0 && out[1] == 0 && out[2] == 0, "goc NaN => lenh toc do = 0");
}

static void test_mixer_nan_and_signs() {
    printf("\n[6] Mixer: dau momen va NaN\n");
    PX4_QuadMixer mx; float u[4];
    mx.mixNormalized(0.5f, 0.1f, 0, 0, u);
    CHECK(u[0] > u[1] && u[2] > u[3], "Roll + : M1,M3 (trai) > M2,M4 (phai)");
    mx.mixNormalized(0.5f, NAN, 0, 0, u);
    CHECK(u[0] == 0 && u[1] == 0 && u[2] == 0 && u[3] == 0, "NaN => tat ca motor 0");
}

static void test_setgains_keeps_imax() {
    printf("\n[7] Doi PID truc Yaw qua CLI khong ghi de i_max\n");
    PX4_FlightController fc; fc.init(200.0f);
    fc.setRatePidYaw(0.3f, 0.1f, 0.0f);
    CHECK(fabsf(fc.getRatePidYaw().getGains().i_max - 0.25f) < 1e-6f,
          "i_max Yaw = %.2f (giu 0.25)", fc.getRatePidYaw().getGains().i_max);
}

static bool json_valid_python(const std::string& js) {
    FILE* f = fopen("/tmp/_telemetry_json_check.json", "w");
    if (!f) return false;
    fwrite(js.data(), 1, js.size(), f); fclose(f);
    return system("python3 -I -c \"import json,sys; json.load(open('/tmp/_telemetry_json_check.json'))\" 2>/dev/null") == 0;
}

static void test_telemetry_json() {
    printf("\n[8] JSON telemetry: NaN/Inf, chuoi dac biet, block tune dai\n");
    TelemetryPacket p; memset(&p, 0, sizeof(p));
    p.roll = NAN; p.pitch = INFINITY; p.yaw = -123456.7f; p.gx = -1e6f;
    p.state_str = "ARMED";
    for (int i = 0; i < 4; i++) { p.m_us[i] = 2000; p.m_pct[i] = 100.0f; p.u_norm[i] = 1.0f; }
    p.loop_hz = 200.0f; p.xs_age_ms = 0xFFFFFFFFu; p.rc_sequence = 0xFFFFFFFFu; p.t_ms = 0xFFFFFFFFu;
    PID_AutoTuner tuner; char tune[360];
    int tn = tuner.buildStatusJson(tune, sizeof(tune));
    CHECK(tn > 0 && tn < (int)sizeof(tune), "block tune vua buffer (%d byte)", tn);
    static char out[1400];
    size_t n = telemetry_build_json(p, true, "10.11.162.210", tune, out, sizeof(out));
    CHECK(n > 0, "JSON dong goi thanh cong (%zu byte, buffer 1400)", n);
    CHECK(strstr(out, "nan") == nullptr && strstr(out, "inf") == nullptr, "khong co 'nan'/'inf' trong JSON");
    CHECK(json_valid_python(out), "JSON hop le (python json.load)");
    // Buffer nhỏ: phải trả 0, không gửi JSON cụt
    char small[300];
    CHECK(telemetry_build_json(p, true, "1.2.3.4", tune, small, sizeof(small)) == 0, "buffer nho => tra 0 (khong gui JSON cut)");
    char lg[256];
    size_t ln = telemetry_build_log_json("Loi \"quote\" \\ backslash\n", lg, sizeof(lg));
    CHECK(ln > 0 && json_valid_python(lg), "log JSON escape dung: %s", lg);
}

static void test_autotuner_relay() {
    printf("\n[9] Auto-tuner relay tren mo hinh (truc Roll, cac truc khac do PID giu)\n");
    PX4_FlightController fc; fc.init(200.0f);
    QuadSim sim;
    PID_AutoTuner tuner; tuner.configure(0.15f, 8, 25.0f, 0.04f);
    tuner.start(TUNE_ROLL_RATE);
    fc.setTargetThrottle(50);
    double max_pitch = 0;
    for (int k = 0; k < 4000 && tuner.isActive(); k++) {
        const double dt = 0.005;
        fc.setTargetAttitude(0, 0, 0);
        fc.update(deg(sim.phi), deg(sim.theta), deg(sim.psi), deg(sim.p), deg(sim.q), deg(sim.r), dt, true, true);
        float relay = tuner.update(fc.getFilteredGx() * DEG_TO_RAD,
                                   fmax(fabs(deg(sim.phi)), fabs(deg(sim.theta))), dt);
        float tr, tp, ty; fc.getTorques(tr, tp, ty);
        float u[4];
        fc.getMixer().mixNormalized(0.5f, relay, tp, ty, u);
        sim.dist[1] = 2.0; // nhiễu nhỏ trục pitch: PID phải giữ
        sim.step(u, dt);
        max_pitch = std::max(max_pitch, fabs(deg(sim.theta)));
    }
    CHECK(tuner.isDone() && tuner.getResult().valid, "tune hoan thanh, Ku=%.3f Tu=%.3f s -> Kp=%.3f Ki=%.3f Kd=%.4f",
          tuner.getResult().ku, tuner.getResult().tu, tuner.getResult().kp, tuner.getResult().ki, tuner.getResult().kd);
    CHECK(max_pitch < 5.0, "truc Pitch duoc PID giu on dinh trong luc tune Roll (max %.2f deg)", max_pitch);
}

int main() {
    test_cascade_step_response();
    test_disturbance_rejection_integral();
    test_anti_windup();
    test_gyro_filter_not_zeroed_when_disarmed();
    test_body_rate_transform();
    test_mixer_nan_and_signs();
    test_setgains_keeps_imax();
    test_telemetry_json();
    test_autotuner_relay();
    printf("\n==== %s (%d loi) ====\n", g_fail ? "THAT BAI" : "TAT CA DAT", g_fail);
    return g_fail ? 1 : 0;
}

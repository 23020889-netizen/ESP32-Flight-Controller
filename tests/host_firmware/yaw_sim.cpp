// Kiểm thử điều khiển YAW trên mã firmware thật (px4_pid.cpp: vòng ngoài/trong + yawSetpointStep)
// Mô hình: giá thử chỉ tự do trục Yaw, ma sát tĩnh ổ trục, ESC 50 Hz (giữ lệnh 20 ms), motor tau 40 ms.
// Gói RC từ Web: gửi khi cần đổi >= 0.5 (cách nhau >= 30 ms), heartbeat 200 ms khi giữ yên cần
// (đúng dashboard_application_controller.js). Logic RC/loop mô phỏng đúng dual_imu_esp32c6.ino.
#include "Arduino.h"
#include "../../firmware/dual_imu_esp32c6/px4_pid.h"

uint32_t g_fake_millis = 0;
FakeSerial Serial;

// Hằng số giống dual_imu_esp32c6.ino
static const float YAW_STICK_FULL_SCALE   = 45.0f;
static const float YAW_STICK_MAX_RATE_DPS = 45.0f;
static const float YAW_SP_MAX_LEAD_DEG    = 10.0f;
static const double YAW_STICK_TIMEOUT_S   = 0.45;

static int g_fail = 0;
#define CHECK(cond, ...) do { if (!(cond)) { g_fail++; printf("  [FAIL] " __VA_ARGS__); } \
                              else { printf("  [ OK ] " __VA_ARGS__); } printf("\n"); } while (0)

static double wrap180(double a) { while (a > 180) a -= 360; while (a < -180) a += 360; return a; }

struct Scenario {
    double k_yaw = 25.0;      // rad/s^2 cho 1.0 mô-men Yaw chuẩn hóa
    double fric = 2.0;        // ma sát tĩnh/động (rad/s^2)
    double sign = +1.0;       // +1 = đúng vật lý đã xác minh trên khung; -1 = giả định khung bị đảo
    bool   fw_invert = false; // cờ "invert yaw" trong firmware
    double thr = 40.0;
    double (*stick)(double t) = nullptr;   // cần Yaw (-45..45) theo thời gian
    bool   drop_packets_after = false; double drop_t = 1e9;
};

struct Trace { std::vector<double> t, yaw, gz, sp; double max_lead = 0; };

static Trace simulate(const Scenario& sc, double T) {
    PX4_FlightController fc; fc.init(200.0f); fc.reset();
    fc.setYawInversion(sc.fw_invert);
    fc.setTargetThrottle((float)sc.thr);
    Trace tr;
    double psi = 0, r = 0, motor[4] = {0}, hold[4] = {0}, esc_t = 0;
    float yaw_sp = 0, yaw_rate_cmd = 0;
    YawStickShaper shaper;
    double last_pkt = -1, last_sent = -999, last_rc = -1;
    const double dt = 0.005;
    for (double t = 0; t < T; t += dt) {
        g_fake_millis = (uint32_t)(t * 1000);
        // ---- Web -> ESP32: gói RC ----
        double st = sc.stick ? sc.stick(t) : 0.0;
        bool changed = fabs(st - last_sent) >= 0.5;
        bool link = !(sc.drop_packets_after && t >= sc.drop_t);
        if (link && ((changed && t - last_pkt >= 0.03) || t - last_pkt >= 0.2)) {
            last_pkt = t; last_sent = st; last_rc = t;
            // executeCommand("RC,...") trong firmware
            yaw_rate_cmd = (fabs(st) > 1.0) ? (float)(constrain(st / YAW_STICK_FULL_SCALE, -1.0, 1.0) * YAW_STICK_MAX_RATE_DPS) : 0.0f;
        }
        // ---- loop() 200 Hz ----
        float cmd = (t - last_rc > YAW_STICK_TIMEOUT_S) ? 0.0f : yaw_rate_cmd;
        yaw_sp = shaper.update(yaw_sp, cmd, (float)psi, (float)dt, YAW_SP_MAX_LEAD_DEG);
        fc.setYawRateFeedforward(cmd * DEG_TO_RAD);
        fc.setTargetAttitude(0, 0, yaw_sp);
        fc.update(0, 0, (float)psi, 0, 0, (float)(r * 57.29578), (float)dt, true, true);
        tr.max_lead = std::max(tr.max_lead, fabs(wrap180(yaw_sp - psi)));
        const float* u = fc.getNormalizedMotorOutputs();
        // ---- ESC 50 Hz + motor + khung ----
        esc_t += dt; if (esc_t >= 0.02) { esc_t -= 0.02; for (int i = 0; i < 4; i++) hold[i] = u[i]; }
        for (int i = 0; i < 4; i++) motor[i] += (hold[i] - motor[i]) * (dt / 0.04);
        // Vật lý đã xác minh: cặp cánh CW (M3,M4) nhanh hơn => thân xoay CCW => Yaw tăng (xem frame_sim.cpp)
        double tyaw = (motor[2] + motor[3] - motor[0] - motor[1]) / 4.0;
        double acc = sc.sign * sc.k_yaw * tyaw - 0.3 * r;
        if (fabs(r) < 1e-3 && fabs(acc) < sc.fric) { r = 0; acc = 0; }
        else acc -= sc.fric * (r > 0 ? 1 : (r < 0 ? -1 : (acc > 0 ? 1 : -1)));
        r += acc * dt;
        psi = wrap180(psi + r * 57.29578 * dt);
        tr.t.push_back(t); tr.yaw.push_back(psi); tr.gz.push_back(r * 57.29578); tr.sp.push_back(yaw_sp);
    }
    return tr;
}

// Góc đã xoay (unwrap) giữa 2 thời điểm
static double travel(const Trace& tr, double t0, double t1) {
    double acc = 0; size_t i0 = (size_t)(t0 / 0.005), i1 = std::min(tr.yaw.size() - 1, (size_t)(t1 / 0.005));
    for (size_t i = i0 + 1; i <= i1; i++) acc += wrap180(tr.yaw[i] - tr.yaw[i - 1]);
    return acc;
}
static double first_move_time(const Trace& tr, double after, double deg) {
    for (size_t i = 0; i < tr.t.size(); i++) if (tr.t[i] > after && fabs(travel(tr, after, tr.t[i])) > deg) return tr.t[i] - after;
    return 1e9;
}

static double full_stick_5s(double t) { return (t > 0.5 && t < 5.5) ? 45.0 : 0.0; }
static double half_stick_5s(double t) { return (t > 0.5 && t < 5.5) ? -22.5 : 0.0; }

int main() {
    printf("\n[1] Giu full can Yaw +45 trong 5 s (ga 40%%, ma sat 2.0 rad/s^2, ESC 50 Hz)\n");
    {
        Scenario sc; sc.stick = full_stick_5s;
        Trace tr = simulate(sc, 9.0);
        double t_move = first_move_time(tr, 0.5, 2.0);
        double rate = travel(tr, 2.5, 5.5) / 3.0;
        CHECK(t_move < 0.6, "bat dau xoay sau %.2f s (code cu tren main: ~3.5 s)", t_move);
        CHECK(fabs(rate - 45.0) < 6.0, "toc do xoay on dinh %.1f deg/s ~ 45 deg/s (code cu: ~9 deg/s)", rate);
        CHECK(tr.max_lead <= YAW_SP_MAX_LEAD_DEG + 0.5, "moc yaw_sp khong chay truoc qua %.0f deg (max %.1f)", YAW_SP_MAX_LEAD_DEG, tr.max_lead);
        double after = travel(tr, 5.5, 9.0);
        double hold = travel(tr, 7.0, 9.0);
        CHECK(fabs(after) < 8.0 && fabs(hold) < 0.5, "nha can o 45 deg/s: troi theo quan tinh %.1f deg roi giu huong (troi 2 s cuoi %.2f deg)", after, hold);
    }
    printf("\n[2] Nua can nguoc chieu (-22.5) => ~ -22.5 deg/s\n");
    {
        Scenario sc; sc.stick = half_stick_5s;
        Trace tr = simulate(sc, 7.0);
        double rate = travel(tr, 2.5, 5.5) / 3.0;
        CHECK(fabs(rate + 22.5) < 4.0, "toc do %.1f deg/s", rate);
    }
    printf("\n[3] On dinh voi khung manh/yeu (k_yaw 12..60), khong ma sat: dap ung tat dan sau khi nha can\n");
    for (double k : {12.0, 25.0, 60.0}) {
        Scenario sc; sc.stick = full_stick_5s; sc.k_yaw = k; sc.fric = 0.0;
        Trace tr = simulate(sc, 13.0);
        double max_mid = 0, max_late = 0;
        for (size_t i = (size_t)(7.0 / 0.005); i < (size_t)(9.0 / 0.005); i++) max_mid = std::max(max_mid, fabs(tr.gz[i]));
        for (size_t i = (size_t)(11.0 / 0.005); i < tr.gz.size(); i++) max_late = std::max(max_late, fabs(tr.gz[i]));
        CHECK(max_late < 0.5 && max_late < max_mid + 1e-6,
              "k_yaw=%.0f: |gz| 7-9 s = %.2f -> 11-13 s = %.2f deg/s (tat dan, khong dao dong duy tri)", k, max_mid, max_late);
    }
    printf("\n[4] Mat goi RC khi dang giu can => ngung xoay (khong tu xoay mai)\n");
    {
        Scenario sc; sc.stick = [](double t) { return t > 0.5 ? 45.0 : 0.0; };
        sc.drop_packets_after = true; sc.drop_t = 3.0;
        Trace tr = simulate(sc, 7.0);
        double late = travel(tr, 5.0, 7.0);
        CHECK(fabs(late) < 0.5, "sau khi mat goi: xoay them %.2f deg trong 2 s cuoi", late);
    }
    printf("\n[5] Dau co cau chap hanh nguoc + co 'invert yaw' => dieu khien dung\n");
    {
        Scenario sc; sc.stick = full_stick_5s; sc.sign = -1.0; sc.fw_invert = true;
        Trace tr = simulate(sc, 7.0);
        double rate = travel(tr, 2.5, 5.5) / 3.0;
        CHECK(fabs(rate - 45.0) < 6.0, "toc do %.1f deg/s dung chieu can", rate);
    }
    printf("\n[6] Dau nguoc ma KHONG dao co => Yaw quay mat kiem soat (ly do can 'yaw check' + bao ve runaway)\n");
    {
        Scenario sc; sc.stick = full_stick_5s; sc.sign = -1.0; sc.fw_invert = false;
        Trace tr = simulate(sc, 4.0);
        double maxgz = 0; for (double g : tr.gz) maxgz = std::max(maxgz, fabs(g));
        CHECK(maxgz > 120.0, "|gz| dat %.0f deg/s > nguong runaway 120 deg/s (firmware se ngat mo-men Yaw)", maxgz);
    }
    printf("\n[7] yawSetpointStep: qua moc +-180 va gioi han dan truoc\n");
    {
        float sp = yawSetpointStep(179.0f, 45.0f, 178.0f, 0.1f, 20.0f);
        CHECK(fabs(sp - (-176.5f)) < 1e-3, "179 + 4.5 => %.2f (quan +-180)", sp);
        sp = yawSetpointStep(-170.0f, 45.0f, 170.0f, 0.005f, 20.0f);   // dang dan truoc 20 deg qua moc
        CHECK(fabs(wrap180(sp - 170.0f) - 20.0f) < 1e-3, "dan truoc bi kep 20 deg qua moc +-180 (sp=%.2f)", sp);
        sp = yawSetpointStep(10.0f, NAN, 0.0f, 0.005f, 20.0f);
        CHECK(sp == 10.0f, "lenh NaN => giu nguyen moc");
    }
    printf("\n==== YAW: %s (%d loi) ====\n", g_fail ? "THAT BAI" : "TAT CA DAT", g_fail);
    return g_fail ? 1 : 0;
}

// Kiểm thử hệ số PID Roll/Pitch & auto-tune trên mã firmware thật (px4_pid.cpp, pid_autotuner.cpp)
// với mô hình 1 trục sát thực tế của giá thử / drone:
//   - Xsens MTi-3 100 Hz (giữ mẫu) + trễ cảm biến, nhiễu Gyro
//   - Vòng PID 200 Hz, ESC 50 Hz (ESC chốt lệnh mỗi 20 ms), motor + cánh bậc 1 (A2212)
//   - Độ cứng khung k (rad/s^2 cho 1.0 mô-men chuẩn hóa): ~200 (giá thử nặng) .. ~1300 (drone nhẹ tự do)
//   - Con lắc trọng lực gs (rad/s^2 / rad, dương = trọng tâm NẰM TRÊN trục xoay của giá => mất ổn định)
//   - Lệch trọng tâm hằng (bias)
// Dấu vật lý theo câu trả lời xác minh trên khung thật: motor trái (M1, M3) tăng => Roll dương.
#include "Arduino.h"
#include "../../firmware/dual_imu_esp32c6/px4_pid.h"
#include "../../firmware/dual_imu_esp32c6/pid_autotuner.h"
#include <random>
#include <vector>
#include <functional>

uint32_t g_fake_millis = 0;
FakeSerial Serial;
std::deque<uint8_t> g_fake_uart_rx;
void telemetry_wifi_broadcast_msg(const char*) {}

static int g_fail = 0;
#define CHECK(cond, ...) do { if (!(cond)) { g_fail++; printf("  [FAIL] " __VA_ARGS__); } \
                              else { printf("  [ OK ] " __VA_ARGS__); } printf("\n"); } while (0)

struct Plant {
    double k = 600, tau_m = 0.06, gs = 0, bias = 0, damp = 0.3, sens_lat = 0.008, noise_dps = 0.3;
    double kick_t = -1, kick_rate_dps = 0;   // nhiễu va đập: cộng tốc độ góc tức thời tại kick_t
};
struct Result { bool stable; double max_abs, osc_rms, mean_err, t_settle; };

// ctrl(angle_deg, rate_dps, t, u[4]) -> false để dừng sớm
using Ctrl = std::function<bool(double, double, double, float*)>;

static Result run_axis(const Plant& P, double thr, double theta0, double T, const Ctrl& ctrl) {
    std::mt19937 rng(1);
    std::normal_distribution<double> nz(0, P.noise_dps);
    const double dt = 0.001;
    double th = theta0 * DEG_TO_RAD, w = 0;
    double mot[4] = { thr, thr, thr, thr }, hold[4] = { thr, thr, thr, thr };
    float u[4] = { (float)thr, (float)thr, (float)thr, (float)thr };
    std::vector<double> hth, hw;
    double mth = th, mw = 0, next_s = 0, next_p = 0, next_e = 0, last_big = 0, s1 = 0, s2 = 0;
    int n = 0;
    bool kicked = false;
    const size_t lat = (size_t)(P.sens_lat / dt + 0.5);
    Result r{ true, 0, 0, 0, T };
    for (double t = 0; t < T; t += dt) {
        if (!kicked && P.kick_t >= 0 && t >= P.kick_t) { w += P.kick_rate_dps * DEG_TO_RAD; kicked = true; }
        hth.push_back(th); hw.push_back(w);
        if (t >= next_s) {                                    // Xsens 100 Hz
            next_s += 0.01;
            const size_t i = hth.size() > lat ? hth.size() - 1 - lat : 0;
            mth = hth[i];
            mw = hw[i] + nz(rng) * DEG_TO_RAD;
        }
        if (t >= next_p) {                                    // PID 200 Hz
            next_p += 0.005;
            g_fake_millis = (uint32_t)(t * 1000);
            if (!ctrl(mth * RAD_TO_DEG, mw * RAD_TO_DEG, t, u)) break;
        }
        if (t >= next_e) {                                    // ESC 50 Hz
            next_e += 0.02;
            for (int i = 0; i < 4; i++) hold[i] = u[i];
        }
        for (int i = 0; i < 4; i++) mot[i] += (hold[i] - mot[i]) * dt / P.tau_m;
        const double d = ((mot[0] + mot[2]) - (mot[1] + mot[3])) / 4.0;   // Roll: trái - phải
        w += (P.k * d + P.gs * sin(th) + P.bias - P.damp * w) * dt;
        th += w * dt;
        const double deg = th * RAD_TO_DEG;
        r.max_abs = std::max(r.max_abs, fabs(deg));
        if (fabs(deg) > 2.0) last_big = t;
        if (fabs(deg) > 60.0) { r.stable = false; break; }   // firmware AUTO DISARM ở 60°
        if (t > T - 3.0) { s1 += deg; s2 += deg * deg; n++; }
    }
    if (n) { r.mean_err = s1 / n; r.osc_rms = sqrt(std::max(0.0, s2 / n - r.mean_err * r.mean_err)); }
    r.t_settle = last_big;
    if (r.osc_rms > 1.0 || fabs(r.mean_err) > 2.0) r.stable = false;
    return r;
}

// Vòng kín Roll với bộ điều khiển đầy đủ (Angle P -> Rate PID -> Mixer) của firmware
static Result closed_loop(const Plant& P, PX4_FlightController& fc, double thr, double theta0, double T = 10.0) {
    fc.setPidEnabled(true);
    fc.setTargetThrottle((float)(thr * 100));
    return run_axis(P, thr, theta0, T, [&](double ang, double rate, double, float* u) {
        fc.setTargetAttitude(0, 0, 0);
        fc.update((float)ang, 0, 0, (float)rate, 0, 0, 0.005f, true, true);
        const float* m = fc.getNormalizedMotorOutputs();
        for (int i = 0; i < 4; i++) u[i] = m[i];
        return true;
    });
}

static Plant mk(double k, double tm, double gs, double bias_frac, double lat = 0.008) {
    Plant p; p.k = k; p.tau_m = tm; p.gs = gs; p.bias = bias_frac * k; p.sens_lat = lat; return p;
}

int main() {
    printf("\n[1] He so MAC DINH (PidDefaults) tren luoi khung / tre / motor, nghieng ban dau 10 deg, ga 45%%\n");
    {
        struct Sc { const char* name; Plant p; };
        const Sc sc[] = {
            { "k=200  can bang          ", mk(200, 0.06, 0, 0) },
            { "k=200  lech TT + con lac ", mk(200, 0.08, 15, 0.06) },
            { "k=400  con lac manh 30   ", mk(400, 0.06, 30, 0.03) },
            { "k=700  can bang          ", mk(700, 0.06, 0, 0) },
            { "k=700  lech TT + con lac ", mk(700, 0.08, 15, 0.06) },
            { "k=1000 can bang          ", mk(1000, 0.06, 0, 0) },
            { "k=1300 (ngoai luoi chon) ", mk(1300, 0.06, 0, 0.03) },
            { "k=700  motor cham 0.12 s ", mk(700, 0.12, 0, 0.03) },
            { "k=700  tre Xsens 20 ms   ", mk(700, 0.06, 0, 0.03, 0.020) },
        };
        int old_fail = 0;
        for (const Sc& s : sc) {
            PX4_FlightController fc; fc.init(200.0f); fc.reset();
            Result r = closed_loop(s.p, fc, 0.45, 10.0);
            CHECK(r.stable, "%s max %5.1f deg, dao dong %.2f, lech %+.2f, on dinh sau %.1f s",
                  s.name, r.max_abs, r.osc_rms, r.mean_err, r.t_settle);
            // So sánh: bộ cũ trên main (Kp .06 Ki .03 Kd 0 imax .10 D-lọc 10 Hz, Angle 3.0)
            PX4_FlightController old; old.init(200.0f);
            old.getRatePidRoll().init(0.06f, 0.03f, 0.0f, 0.10f, 10.0f, 200.0f);
            old.setAngleGains(3.0f, 3.0f, 1.5f); old.reset();
            if (!closed_loop(s.p, old, 0.45, 10.0).stable) old_fail++;
        }
        printf("  (tham khao) bo cu Kp .06/Ki .03/Kd 0/Angle 3.0 that bai %d/9 truong hop tren\n", old_fail);
        CHECK(old_fail >= 5, "bo cu mat on dinh tren khung nhe (%d/9) => xac nhan nguyen nhan lat", old_fail);
    }

    printf("\n[2] Tich phan bat tu 15%% ga: giu thang bang o ga 20%% du lech trong tam\n");
    {
        PX4_FlightController fc; fc.init(200.0f); fc.reset();
        Result r = closed_loop(mk(500, 0.06, 10, 0.05), fc, 0.20, 5.0, 12.0);
        CHECK(r.stable && fabs(r.mean_err) < 0.5, "ga 20%%: lech trung binh %+.2f deg (I=%.3f)", r.mean_err, fc.getRatePidRoll().getI());
    }

    printf("\n[3] Anti-windup theo quyen dieu khien cua Mixer\n");
    {
        PX4_RatePID pid; pid.init(PidDefaults::RP_KP, PidDefaults::RP_KI, PidDefaults::RP_KD, PidDefaults::RP_IMAX, PidDefaults::RP_DCUT, 200.0f);
        for (int i = 0; i < 600; i++) pid.update(5.0f, 0.0f, 0.005f, true, 0.10f);   // bão hòa kéo dài 3 s
        CHECK(fabsf(pid.getI()) <= 0.10f + 1e-4f, "bao hoa 3 s voi gioi han 0.10: I = %.3f (khong tran toi i_max %.2f)",
              pid.getI(), PidDefaults::RP_IMAX);
        const float out = pid.update(5.0f, 0.0f, 0.005f, true, 0.10f);
        CHECK(fabsf(out) <= 0.10f + 1e-6f, "dau ra bi kep dung gioi han Mixer (%.3f)", out);
    }

    printf("\n[4] Auto-tune: PID giu 3 s -> relay giu goc + bu lech -> ap he so -> vong kin (danh nghia & lech mo hinh)\n");
    for (double k : { 200.0, 400.0, 700.0, 1000.0, 1300.0 })
        for (double gs : { 0.0, 15.0 }) {
            const Plant P = mk(k, 0.06, gs, 0.04);
            const double thr = 0.45;
            PX4_FlightController fc; fc.init(200.0f); fc.reset();
            fc.setPidEnabled(true); fc.setTargetThrottle((float)(thr * 100));
            PID_AutoTuner tuner;
            double max_tilt = 0;
            bool started = false;
            run_axis(P, thr, 0.0, 40.0, [&](double ang, double rate, double t, float* u) {
                fc.setTargetAttitude(0, 0, 0);
                fc.update((float)ang, 0, 0, (float)rate, 0, 0, 0.005f, true, true);
                if (!started && t >= 3.0) {
                    // Giống startTuneAxis() trong dual_imu_esp32c6.ino
                    tuner.configure(0.10f, 8, 35.0f, 0.04f);
                    tuner.start(TUNE_ROLL_RATE, fc.getRatePidRoll().getI());
                    started = true;
                }
                if (!started) {
                    const float* m = fc.getNormalizedMotorOutputs();
                    for (int i = 0; i < 4; i++) u[i] = m[i];
                    return true;
                }
                max_tilt = std::max(max_tilt, fabs(ang));
                const float relay = tuner.updateWithAngleHold((float)(rate * DEG_TO_RAD), (float)ang, (float)fabs(ang), 0.005f);
                if (!tuner.isActive()) return false;
                fc.getMixer().mixNormalized((float)thr, relay, 0, 0, u);
                return true;
            });
            const TuneResult& tr = tuner.getResult();
            CHECK(tuner.isDone() && tr.valid && max_tilt < 15.0,
                  "k=%4.0f gs=%2.0f: tune xong, nghieng max %.1f deg | Ku=%.4f Tu=%.3f s -> Kp %.3f Ki %.3f Kd %.4f Angle %.1f",
                  k, gs, max_tilt, tr.ku, tr.tu, tr.kp, tr.ki, tr.kd, tr.angle_p);
            if (!tr.valid) continue;
            Plant Q = P; Q.k *= 1.3; Q.sens_lat += 0.010;     // khung cứng hơn 30% + trễ thêm 10 ms
            for (int smooth = 0; smooth < 2; smooth++)
                for (int pass = 0; pass < 2; pass++) {
                    PX4_FlightController c; c.init(200.0f);
                    c.setRatePidRoll(smooth ? tr.kp_smooth : tr.kp, smooth ? tr.ki_smooth : tr.ki, smooth ? tr.kd_smooth : tr.kd);
                    // Như firmware: "tune apply" trục roll/pitch chỉ đổi hệ số Rate, Angle P giữ mặc định
                    c.setAngleGains(PidDefaults::ANG_RP, PidDefaults::ANG_RP, PidDefaults::ANG_Y);
                    c.reset();
                    Result r = closed_loop(pass ? Q : P, c, thr, 10.0);
                    if (!r.stable)
                        CHECK(false, "     ap '%s' %s: max %.1f dao dong %.2f", smooth ? "tune apply 50" : "tune apply",
                              pass ? "[k*1.3, +10 ms]" : "[danh nghia]", r.max_abs, r.osc_rms);
                }
        }

    printf("\n[5] Va dap manh luc dang tune (+250 deg/s): tune NGAT, PID nhan lai quyen voi tich phan ban giao, KHONG lat\n");
    {
        Plant P = mk(700, 0.06, 15, 0.04); P.kick_t = 3.5; P.kick_rate_dps = 250.0;   // 0.5 s sau khi bắt đầu tune
        const double thr = 0.45;
        PX4_FlightController fc; fc.init(200.0f); fc.reset();
        fc.setPidEnabled(true); fc.setTargetThrottle((float)(thr * 100));
        PID_AutoTuner tuner;
        bool started = false, handed = false, active_at_kick = false;
        Result r = run_axis(P, thr, 0.0, 14.0, [&](double ang, double rate, double t, float* u) {
            fc.setTargetAttitude(0, 0, 0);
            fc.update((float)ang, 0, 0, (float)rate, 0, 0, 0.005f, true, true);
            if (!started && t >= 3.0) {
                tuner.configure(0.10f, 8, 35.0f, 0.04f);
                tuner.start(TUNE_ROLL_RATE, fc.getRatePidRoll().getI());
                started = true;
            }
            if (started && tuner.isActive()) {
                if (t >= P.kick_t - 0.01 && t < P.kick_t) active_at_kick = true;
                const float bias_before = tuner.getBias();
                const float relay = tuner.updateWithAngleHold((float)(rate * DEG_TO_RAD), (float)ang, (float)fabs(ang), 0.005f);
                if (tuner.isActive()) { fc.getMixer().mixNormalized((float)thr, relay, 0, 0, u); return true; }
                fc.getRatePidRoll().setIntegral(bias_before);   // bàn giao như loop() của firmware
                handed = true;
            }
            const float* m = fc.getNormalizedMotorOutputs();
            for (int i = 0; i < 4; i++) u[i] = m[i];
            return true;
        });
        CHECK(active_at_kick && handed && tuner.isAborted(), "dang tune luc va dap -> tune bi ngat (ly do %d)", (int)tuner.getAbortReason());
        CHECK(r.stable && r.max_abs < 60.0, "PID keo ve: nghieng max %.1f deg (< 60 failsafe), cuoi lech %+.2f deg",
              r.max_abs, r.mean_err);
    }

    printf("\n==== PID & AUTO-TUNE: %s (%d loi) ====\n", g_fail ? "THAT BAI" : "TAT CA DAT", g_fail);
    return g_fail ? 1 : 0;
}

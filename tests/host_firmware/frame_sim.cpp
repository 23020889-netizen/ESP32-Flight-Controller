// Kiểm thử DẤU vòng kín 3 trục trên mã firmware thật (px4_pid.cpp: Mixer + PID 2 vòng),
// với mô hình vật lý suy ra từ câu trả lời xác minh trên khung thật (2026-11-24):
//   A3: bố trí  M1 GPIO7 Trước-Trái CCW | M4 GPIO4 Trước-Phải CW
//               M3 GPIO5 Sau-Trái   CW  | M2 GPIO6 Sau-Phải   CCW   (nhìn từ trên, đầu hướng lên)
//   B1: nghiêng cánh PHẢI hạ xuống  -> Roll  dương, gx dương
//   B2: NGÓC đầu lên                -> Pitch âm,    gy âm
//   B3: xoay CÙNG chiều kim đồng hồ  -> Yaw giảm (gz âm) với dữ liệu thô; người dùng yêu cầu đảo dấu Yaw
//       (xsens_mti_set_yaw_inversion(true)) => nay xoay CÙNG chiều kim đồng hồ -> Yaw TĂNG (gz dương)
// Suy ra (lực đẩy & phản mô-men cánh quạt):
//   roll_acc  ∝ (trái - phải)      = (M1+M3) - (M2+M4)   : motor trái tăng -> cánh phải hạ -> Roll +
//   pitch_acc ∝ (sau - trước)      = (M2+M3) - (M1+M4)   : motor trước tăng -> ngóc đầu -> Pitch âm
//   yaw_acc   ∝ (CCW - CW)         = (M1+M2) - (M3+M4)   : cặp CCW quay nhanh -> thân xoay CW -> Yaw tăng
#include "Arduino.h"
#include "../../firmware/dual_imu_esp32c6/px4_pid.h"

uint32_t g_fake_millis = 0;
FakeSerial Serial;
std::deque<uint8_t> g_fake_uart_rx;
static int g_fail = 0;
#define CHECK(cond, ...) do { if (!(cond)) { g_fail++; printf("  [FAIL] " __VA_ARGS__); } \
                              else { printf("  [ OK ] " __VA_ARGS__); } printf("\n"); } while (0)

struct Acc { double roll, pitch, yaw; };
static Acc physics(const double m[4]) {       // m[0..3] = M1..M4
    return { ((m[0] + m[2]) - (m[1] + m[3])) / 4.0,
             ((m[1] + m[2]) - (m[0] + m[3])) / 4.0,
             ((m[0] + m[1]) - (m[2] + m[3])) / 4.0 };
}

int main() {
    printf("\n[1] Dau co cau chap hanh: mo-men PID duong phai tao gia toc goc DUONG (hoi tiep am)\n");
    {
        PX4_QuadMixer mx; float u[4]; double m[4];
        const char* names[3] = { "Roll ", "Pitch", "Yaw  " };
        for (int ax = 0; ax < 3; ax++) {
            mx.mixNormalized(0.5f, ax == 0 ? 0.1f : 0.f, ax == 1 ? 0.1f : 0.f, ax == 2 ? 0.1f : 0.f, u);
            for (int i = 0; i < 4; i++) m[i] = u[i];
            Acc a = physics(m);
            double v = ax == 0 ? a.roll : (ax == 1 ? a.pitch : a.yaw);
            CHECK(v > 0, "%s: tau=+0.1 -> gia toc %+.3f (u = %.2f %.2f %.2f %.2f)", names[ax], v, u[0], u[1], u[2], u[3]);
        }
    }

    printf("\n[2] Vong kin 3 truc tu nhieu ban dau (R=+10, P=-10, Yaw lech 20 deg), ga 45%%, ESC 50 Hz\n");
    {
        PX4_FlightController fc; fc.init(200.0f); fc.reset();
        fc.setTargetThrottle(45.0f);
        const double k[3] = { 60.0, 60.0, 25.0 };      // rad/s^2 cho 1.0 chênh lệch chuẩn hóa
        double ang[3] = { 10.0, -10.0, 0.0 }, rate[3] = { 0, 0, 0 };   // deg, deg/s
        double hold[4] = { 0 }, mot[4] = { 0 }, esc_t = 0, max_tilt = 0;
        const double dt = 0.005;
        for (double t = 0; t < 8.0; t += dt) {
            g_fake_millis = (uint32_t)(t * 1000);
            fc.setTargetAttitude(0.0f, 0.0f, 20.0f);
            fc.update((float)ang[0], (float)ang[1], (float)ang[2],
                      (float)rate[0], (float)rate[1], (float)rate[2], (float)dt, true, true);
            const float* u = fc.getNormalizedMotorOutputs();
            esc_t += dt; if (esc_t >= 0.02) { esc_t -= 0.02; for (int i = 0; i < 4; i++) hold[i] = u[i]; }
            for (int i = 0; i < 4; i++) mot[i] += (hold[i] - mot[i]) * (dt / 0.04);
            Acc a = physics(mot);
            double acc[3] = { a.roll * k[0], a.pitch * k[1], a.yaw * k[2] };
            for (int j = 0; j < 3; j++) {
                double r = rate[j] * DEG_TO_RAD;
                r += (acc[j] - 0.5 * r) * dt;
                rate[j] = r * RAD_TO_DEG;
                ang[j] += rate[j] * dt;
            }
            max_tilt = std::max(max_tilt, std::max(fabs(ang[0]), fabs(ang[1])));
            if (max_tilt > 60.0) break;   // firmware sẽ AUTO DISARM ở 60 deg
        }
        CHECK(max_tilt <= 60.0, "khong cham nguong failsafe 60 deg (nghieng lon nhat %.1f deg)", max_tilt);
        CHECK(fabs(ang[0]) < 1.0 && fabs(ang[1]) < 1.0, "Roll/Pitch ve 0 (cuoi: R=%.2f P=%.2f deg)", ang[0], ang[1]);
        CHECK(fabs(ang[2] - 20.0) < 2.0, "Yaw ve moc 20 deg (cuoi: %.2f deg)", ang[2]);
    }
    printf("\n==== DAU 3 TRUC: %s (%d loi) ====\n", g_fail ? "THAT BAI" : "TAT CA DAT", g_fail);
    return g_fail ? 1 : 0;
}

// Stub Arduino.h tối giản để biên dịch các module điều khiển thuần (PID, bộ lọc, JSON)
// trên máy tính (g++) và chạy mô phỏng vòng kín. KHÔNG dùng cho ESP32.
#pragma once
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstdarg>
#include <algorithm>
#include <vector>

using std::isnan;
using std::isinf;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

template <typename T, typename L, typename H>
static inline T constrain(T v, L lo, H hi) { return v < (T)lo ? (T)lo : (v > (T)hi ? (T)hi : v); }

extern uint32_t g_fake_millis;
static inline uint32_t millis() { return g_fake_millis; }
static inline uint32_t micros() { return g_fake_millis * 1000u; }

struct FakeSerial {
    bool quiet = true;
    int printf(const char* fmt, ...) {
        if (quiet) return 0;
        va_list ap; va_start(ap, fmt); int n = vprintf(fmt, ap); va_end(ap); return n;
    }
    void println(const char* s = "") { if (!quiet) ::printf("%s\n", s); }
    void print(const char* s) { if (!quiet) ::printf("%s", s); }
};
extern FakeSerial Serial;

#include <string>
class String : public std::string {
public:
    using std::string::string;
    String(const std::string& s) : std::string(s) {}
};

// UART giả lập cho kiểm thử driver Xsens: test nạp byte vào g_fake_uart_rx
#include <deque>
#define SERIAL_8N1 0
static inline void delay(uint32_t) {}
extern std::deque<uint8_t> g_fake_uart_rx;
class HardwareSerial {
public:
    explicit HardwareSerial(int) {}
    void begin(unsigned long, uint32_t = 0, int8_t = -1, int8_t = -1) {}
    size_t write(const uint8_t*, size_t n) { return n; }
    int available() { return (int)g_fake_uart_rx.size(); }
    int read() { if (g_fake_uart_rx.empty()) return -1; int b = g_fake_uart_rx.front(); g_fake_uart_rx.pop_front(); return b; }
};

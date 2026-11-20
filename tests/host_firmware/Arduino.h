// Stub Arduino.h tối giản để biên dịch các module điều khiển thuần (PID, bộ lọc, JSON)
// trên máy tính (g++) và chạy mô phỏng vòng kín. KHÔNG dùng cho ESP32.
#pragma once
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstdarg>
#include <algorithm>

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

#include "telemetry_json.h"
#include "telemetry_wifi.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static inline double jf(float v) {
    return (isnan(v) || isinf(v)) ? 0.0 : (double)v;
}

// Escape chuỗi để nhúng an toàn vào JSON (dấu ", \\ và ký tự điều khiển)
static size_t json_escape(const char* in, char* out, size_t out_len) {
    if (!out || out_len == 0) return 0;
    size_t o = 0;
    for (const char* c = in ? in : ""; *c; ++c) {
        const unsigned char ch = (unsigned char)*c;
        char rep[7] = {0};
        if (ch == '"' || ch == '\\') { rep[0] = '\\'; rep[1] = (char)ch; }
        else if (ch == '\n') { rep[0] = '\\'; rep[1] = 'n'; }
        else if (ch == '\r') { rep[0] = '\\'; rep[1] = 'r'; }
        else if (ch == '\t') { rep[0] = '\\'; rep[1] = 't'; }
        else if (ch < 0x20) { snprintf(rep, sizeof(rep), "\\u%04x", ch); }
        else { rep[0] = (char)ch; }
        const size_t n = strlen(rep);
        if (o + n >= out_len) break;
        memcpy(out + o, rep, n);
        o += n;
    }
    out[o] = '\0';
    return o;
}

size_t telemetry_build_json(const TelemetryPacket& p, bool sta_connected, const char* sta_ip,
                            const char* extra_json, char* out, size_t out_len) {
    if (!out || out_len == 0) return 0;

    char state_esc[24];
    json_escape(p.state_str ? p.state_str : "UNKNOWN", state_esc, sizeof(state_esc));
    char ip_esc[24];
    json_escape((sta_connected && sta_ip) ? sta_ip : "", ip_esc, sizeof(ip_esc));

    const bool has_extra = extra_json && extra_json[0] != '\0';

    const int n = snprintf(out, out_len,
        "{\"t\":%lu,"
        "\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,"
        "\"mpu_r\":%.1f,\"mpu_p\":%.1f,\"mpu_y\":%.1f,"
        "\"xs_r\":%.1f,\"xs_p\":%.1f,\"xs_y\":%.1f,"
        "\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,"
        "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,\"z\":%.2f,\"vz\":%.2f,"
        "\"m\":[%.1f,%.1f,%.1f,%.1f],\"us\":[%u,%u,%u,%u],\"state\":\"%s\","
        "\"sp_r\":%.1f,\"sp_p\":%.1f,\"sp_y\":%.1f,\"sp_t\":%.1f,"
        "\"rsp\":[%.1f,%.1f,%.1f],\"pid_i\":[%.3f,%.3f,%.3f],"
        "\"tau\":[%.3f,%.3f,%.3f],\"u\":[%.3f,%.3f,%.3f,%.3f],\"yaw_ok\":%d,"
        "\"imu_src\":%u,\"xs_age\":%lu,\"xs_err\":%lu,\"loop_hz\":%.1f,\"loop_max_us\":%lu,"
        "\"rc_seq\":%lu,\"sta_connected\":%d,\"sta_ip\":\"%s\",\"pwm_ok\":%d"
        "%s%s}",
        (unsigned long)p.t_ms,
        jf(p.roll), jf(p.pitch), jf(p.yaw),
        jf(p.mpu_roll), jf(p.mpu_pitch), jf(p.mpu_yaw),
        jf(p.xs_roll),  jf(p.xs_pitch),  jf(p.xs_yaw),
        jf(p.ax), jf(p.ay), jf(p.az),
        jf(p.gx), jf(p.gy), jf(p.gz),
        jf(p.z),  jf(p.vz),
        jf(p.m_pct[0]), jf(p.m_pct[1]), jf(p.m_pct[2]), jf(p.m_pct[3]),
        (unsigned)p.m_us[0], (unsigned)p.m_us[1], (unsigned)p.m_us[2], (unsigned)p.m_us[3],
        state_esc,
        jf(p.sp_roll), jf(p.sp_pitch), jf(p.sp_yaw), jf(p.sp_throttle),
        jf(p.rate_sp[0]), jf(p.rate_sp[1]), jf(p.rate_sp[2]),
        jf(p.pid_i[0]), jf(p.pid_i[1]), jf(p.pid_i[2]),
        jf(p.tau[0]), jf(p.tau[1]), jf(p.tau[2]),
        jf(p.u_norm[0]), jf(p.u_norm[1]), jf(p.u_norm[2]), jf(p.u_norm[3]),
        p.yaw_valid ? 1 : 0,
        (unsigned)p.imu_src, (unsigned long)p.xs_age_ms, (unsigned long)p.xs_err,
        jf(p.loop_hz), (unsigned long)p.loop_max_us,
        (unsigned long)p.rc_sequence,
        sta_connected ? 1 : 0, ip_esc,
        p.pwm_outputs_ready ? 1 : 0,
        has_extra ? "," : "", has_extra ? extra_json : "");

    if (n <= 0 || (size_t)n >= out_len) {
        out[0] = '\0';
        return 0; // Bị cắt cụt => không gửi (JSON hỏng sẽ bị Web bỏ qua âm thầm)
    }
    return (size_t)n;
}

size_t telemetry_build_log_json(const char* msg, char* out, size_t out_len) {
    if (!out || out_len < 12) return 0;
    char esc[200];
    json_escape(msg, esc, sizeof(esc));
    const int n = snprintf(out, out_len, "{\"log\":\"%s\"}", esc);
    if (n <= 0 || (size_t)n >= out_len) {
        out[0] = '\0';
        return 0;
    }
    return (size_t)n;
}

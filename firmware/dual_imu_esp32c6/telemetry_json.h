#ifndef TELEMETRY_JSON_H
#define TELEMETRY_JSON_H

// Đóng gói JSON telemetry thuần C (không phụ thuộc WiFi) để có thể kiểm thử trên máy tính.
#include <stddef.h>
#include <stdint.h>

struct TelemetryPacket;

/**
 * @brief Tạo chuỗi JSON telemetry hoàn chỉnh.
 * - Mọi số thực NaN/Inf được thay bằng 0 (printf "%f" sẽ in "nan" làm JSON.parse() trên Web lỗi
 *   và gói tin bị bỏ qua âm thầm).
 * - extra_json: đoạn "key":{...} bổ sung (ví dụ trạng thái auto-tune), có thể NULL.
 * @return Độ dài chuỗi, hoặc 0 nếu buffer không đủ (KHÔNG bao giờ gửi JSON bị cắt cụt).
 */
size_t telemetry_build_json(const TelemetryPacket& p, bool sta_connected, const char* sta_ip,
                            const char* extra_json, char* out, size_t out_len);

/**
 * @brief Tạo bản tin log {"log":"..."} với chuỗi được escape đúng chuẩn JSON.
 * @return Độ dài chuỗi, 0 nếu lỗi.
 */
size_t telemetry_build_log_json(const char* msg, char* out, size_t out_len);

#endif // TELEMETRY_JSON_H

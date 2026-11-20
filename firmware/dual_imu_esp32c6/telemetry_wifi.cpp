#include "telemetry_wifi.h"
#include "pid_autotuner.h"
#include "telemetry_json.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <mbedtls/sha1.h>
#include <mbedtls/base64.h>
#include <strings.h>
#include <deque>

// ============================================================================
// 1. CẤU HÌNH HỆ THỐNG MẠNG & THÔNG SỐ (STATIC IP 10.11.162.210 CHO ESP32)
// ============================================================================
static const IPAddress STA_STATIC_IP(10, 11, 162, 210);   // IP tĩnh cố định theo yêu cầu
static const IPAddress STA_GATEWAY(10, 11, 162, 1);       // Gateway mạng phòng Lab
static const IPAddress STA_SUBNET(255, 255, 255, 0);      // Subnet mask
static const IPAddress STA_DNS(10, 11, 162, 1);          // DNS server

static const IPAddress AP_LOCAL_IP(192, 168, 4, 1);
static const IPAddress AP_GATEWAY(192, 168, 4, 1);
static const IPAddress AP_SUBNET(255, 255, 255, 0);

static uint16_t g_wsPort = 81;
static uint16_t g_udpPort = 14550;

static WiFiUDP g_udp;
static WiFiServer g_wsServer(81);

#define MAX_WS_CLIENTS 4
static WiFiClient g_wsClients[MAX_WS_CLIENTS];
static bool g_wsHandshakeDone[MAX_WS_CLIENTS] = {false};

// Handshake KHÔNG CHẶN: gom request HTTP qua nhiều vòng loop() thay vì vòng while 400 ms.
// Bản cũ chặn loop() tới 400 ms mỗi khi trình duyệt kết nối => vòng PID 200 Hz đứng hình
// trong lúc đang ARMED (motor giữ nguyên lệnh cũ, mất cân bằng).
static const size_t   WS_HANDSHAKE_MAX = 1024;
static const uint32_t WS_HANDSHAKE_TIMEOUT_MS = 1500;
static char     g_wsReqBuf[MAX_WS_CLIENTS][WS_HANDSHAKE_MAX];
static size_t   g_wsReqLen[MAX_WS_CLIENTS] = {};
static uint32_t g_wsAcceptMs[MAX_WS_CLIENTS] = {};

static std::deque<String> g_cmdQueue;
static uint32_t g_lastBroadcastMs = 0;
static const uint32_t BROADCAST_INTERVAL_MS = 20; // 50 Hz cho 3D Monitor và PID Telemetry

static String g_activeIpStr = "192.168.4.1";
static bool g_staConnected = false;
static uint32_t g_lastNetCheckMs = 0;
static uint8_t g_wsRxBuffer[MAX_WS_CLIENTS][512] = {};
static size_t g_wsRxLength[MAX_WS_CLIENTS] = {};

static uint32_t g_txDropped = 0;   // Số frame bị hủy do ghi TCP không trọn vẹn
static uint32_t g_jsonErrors = 0;  // Số lần JSON không vừa buffer (đã chặn, không gửi)

static void ws_release_slot(int i) {
    if (g_wsClients[i]) g_wsClients[i].stop();
    g_wsHandshakeDone[i] = false;
    g_wsRxLength[i] = 0;   // Bản cũ không xóa => byte rác của client cũ dính vào client mới
    g_wsReqLen[i] = 0;
}

// ============================================================================
// 2. HELPER CHUẨN RFC6455 WEBSOCKET HANDSHAKE
// ============================================================================
// Trả về: 1 = xong, 0 = cần thêm dữ liệu, -1 = lỗi (đóng kết nối)
static int poll_ws_handshake(WiFiClient& client, int clientIndex) {
    size_t& len = g_wsReqLen[clientIndex];
    char* req = g_wsReqBuf[clientIndex];
    while (client.available() > 0 && len < WS_HANDSHAKE_MAX - 1) {
        int c = client.read();
        if (c < 0) break;
        req[len++] = (char)c;
    }
    req[len] = '\0';

    if (strstr(req, "\r\n\r\n") == nullptr) {
        if (len >= WS_HANDSHAKE_MAX - 1) return -1;
        if (millis() - g_wsAcceptMs[clientIndex] > WS_HANDSHAKE_TIMEOUT_MS) return -1;
        return 0;
    }

    if (strncmp(req, "GET", 3) != 0) return -1;

    // Tìm header không phân biệt hoa/thường
    const char* keyHdr = nullptr;
    for (const char* q = req; *q; ++q) {
        if (strncasecmp(q, "Sec-WebSocket-Key:", 18) == 0) { keyHdr = q + 18; break; }
    }
    if (!keyHdr) return -1;
    while (*keyHdr == ' ' || *keyHdr == '\t') ++keyHdr;
    const char* keyEnd = strstr(keyHdr, "\r\n");
    if (!keyEnd || keyEnd - keyHdr > 60 || keyEnd == keyHdr) return -1;

    char concat[128];
    const int klen = (int)(keyEnd - keyHdr);
    snprintf(concat, sizeof(concat), "%.*s258EAFA5-E914-47DA-95CA-C5AB0DC85B11", klen, keyHdr);

    unsigned char sha1_result[20];
    mbedtls_sha1((const unsigned char*)concat, strlen(concat), sha1_result);

    unsigned char base64_out[40];
    size_t olen = 0;
    mbedtls_base64_encode(base64_out, sizeof(base64_out) - 1, &olen, sha1_result, 20);
    base64_out[olen] = '\0';

    char resp[200];
    const int rlen = snprintf(resp, sizeof(resp),
                              "HTTP/1.1 101 Switching Protocols\r\n"
                              "Upgrade: websocket\r\n"
                              "Connection: Upgrade\r\n"
                              "Sec-WebSocket-Accept: %s\r\n\r\n", (char*)base64_out);
    if (client.write((const uint8_t*)resp, rlen) != (size_t)rlen) return -1;

    g_wsHandshakeDone[clientIndex] = true;
    g_wsRxLength[clientIndex] = 0;
    len = 0;
    Serial.printf("[WIFI_WS] Client #%d ket noi thanh cong tu Web Browser!\n", clientIndex);
    return 1;
}

// Gửi 1 Frame Text Unmasked Server -> Browser (RFC6455) bằng MỘT lần ghi TCP.
// Bản cũ ghi header và payload bằng 2 lệnh write() riêng => 2 segment TCP/frame (tăng trễ,
// nghẽn WiFi). Nếu ghi không trọn vẹn, luồng WS bị lệch khung => đóng client để trình duyệt
// tự kết nối lại thay vì nhận dữ liệu rác.
static bool send_ws_frame(WiFiClient& client, uint8_t opcode, const char* payload, size_t len) {
    if (!client.connected()) return false;
    static uint8_t frame[1600];
    size_t headerLen = 0;
    if (len + 4 > sizeof(frame)) return false;

    frame[0] = 0x80 | (opcode & 0x0F);
    if (len <= 125) {
        frame[1] = (uint8_t)len;
        headerLen = 2;
    } else {
        frame[1] = 126;
        frame[2] = (uint8_t)((len >> 8) & 0xFF);
        frame[3] = (uint8_t)(len & 0xFF);
        headerLen = 4;
    }
    if (len) memcpy(frame + headerLen, payload, len);
    const size_t total = headerLen + len;
    const size_t written = client.write(frame, total);
    if (written != total) {
        g_txDropped++;
        return false;
    }
    return true;
}

static bool send_ws_text_frame(WiFiClient& client, const char* textPayload, size_t len) {
    return send_ws_frame(client, 0x01, textPayload, len);
}

// Đọc Frame từ Web Browser
static void read_ws_incoming_frames(WiFiClient& client, int clientIndex) {
    size_t& buffered = g_wsRxLength[clientIndex];
    while (client.available() > 0 && buffered < sizeof(g_wsRxBuffer[clientIndex])) {
        int value = client.read();
        if (value < 0) break;
        g_wsRxBuffer[clientIndex][buffered++] = (uint8_t)value;
    }

    size_t consumed = 0;
    while (buffered - consumed >= 2) {
        const uint8_t* frame = g_wsRxBuffer[clientIndex] + consumed;
        const uint8_t opcode = frame[0] & 0x0F;
        const bool isMasked = (frame[1] & 0x80) != 0;
        uint64_t payloadLen = frame[1] & 0x7F;
        size_t headerLen = 2;
        if (payloadLen == 126) {
            if (buffered - consumed < 4) break;
            payloadLen = ((uint64_t)frame[2] << 8) | frame[3];
            headerLen = 4;
        } else if (payloadLen == 127) {
            // Command frames are capped below 256 bytes; reject oversized frame form.
            buffered = 0;
            return;
        }
        if (isMasked) headerLen += 4;
        if (payloadLen > sizeof(g_wsRxBuffer[clientIndex]) - headerLen) {
            buffered = 0; // Frame lớn hơn buffer: không bao giờ ghép đủ => hủy
            return;
        }
        if (buffered - consumed < headerLen + payloadLen) break; // Keep partial TCP frame intact.

        const uint8_t* mask = isMasked ? frame + headerLen - 4 : nullptr;
        const uint8_t* payload = frame + headerLen;

        if (opcode == 0x08) {
            ws_release_slot(clientIndex);
            return;
        }

        if (opcode == 0x09 && payloadLen <= 125) {
            // Ping -> Pong (giữ kết nối với client/proxy có keep-alive)
            char pong[126];
            for (size_t i = 0; i < payloadLen; ++i) {
                pong[i] = (char)(payload[i] ^ (isMasked ? mask[i % 4] : 0));
            }
            send_ws_frame(client, 0x0A, pong, (size_t)payloadLen);
        } else if (payloadLen <= 256 && opcode == 0x01) {
            char command[257];
            for (size_t i = 0; i < payloadLen; ++i) {
                command[i] = (char)(payload[i] ^ (isMasked ? mask[i % 4] : 0));
            }
            command[payloadLen] = '\0';
            String cmd(command);
            cmd.trim();
            if (cmd.length() > 0) {
                while (g_cmdQueue.size() >= 10) g_cmdQueue.pop_front();
                g_cmdQueue.push_back(cmd);
            }
        }
        consumed += headerLen + (size_t)payloadLen;
    }

    if (consumed > 0) {
        memmove(g_wsRxBuffer[clientIndex], g_wsRxBuffer[clientIndex] + consumed, buffered - consumed);
        buffered -= consumed;
    }
    if (buffered == sizeof(g_wsRxBuffer[clientIndex])) buffered = 0; // Drop malformed/full buffer.
}

// ============================================================================
// 3. KHỞI TẠO WIFI (KẾT NỐI MẠNG TTDB & SOFTAP DỰ PHÒNG)
// ============================================================================
void telemetry_wifi_init(const char* sta_ssid, const char* sta_pass,
                         const char* ap_ssid, const char* ap_pass,
                         uint16_t wsPort) {
    g_wsPort = wsPort;

    Serial.println("\n==================================================================");
    Serial.println("  KHOI TAO HE THONG MANG WIFI CHO ESP32-C6 FLIGHT CONTROLLER");
    Serial.println("==================================================================");

    // 1. Chế độ kép: Vừa phát SoftAP dự phòng vừa kết nối mạng phòng Lab
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAPConfig(AP_LOCAL_IP, AP_GATEWAY, AP_SUBNET);
    WiFi.softAP(ap_ssid, ap_pass);

    Serial.printf("[WIFI_AP] SoftAP du phong da san sang: SSID '%s' | IP: 192.168.4.1\n", ap_ssid);

    // 2. Kết nối vào mạng TTDB với IP tĩnh 10.11.162.210
    Serial.printf("[WIFI_STA] Dang cau hinh Static IP %s tren mang '%s'...", STA_STATIC_IP.toString().c_str(), sta_ssid);
    const bool staticIpConfigured = WiFi.config(STA_STATIC_IP, STA_GATEWAY, STA_SUBNET, STA_DNS);
    if (!staticIpConfigured) {
        Serial.println("\n[WIFI_STA] [LOI] Cau hinh IP tinh that bai; khong ket noi STA bang DHCP.");
    } else {
        WiFi.setAutoReconnect(true);
        WiFi.begin(sta_ssid, sta_pass);
    }

    uint32_t startMs = millis();
    while (staticIpConfigured && WiFi.status() != WL_CONNECTED && millis() - startMs < 6000) {
        delay(250);
        Serial.print(".");
    }
    Serial.println();

    if (staticIpConfigured && WiFi.status() == WL_CONNECTED) {
        g_activeIpStr = WiFi.localIP().toString();
        Serial.println("------------------------------------------------------------------");
        Serial.printf("  [WIFI_STA] >>> KET NOI THANH CONG MANG TTDB: '%s'!\n", sta_ssid);
        Serial.printf("  [WIFI_STA] >>> DIA CHI IP ESP32 : %s\n", g_activeIpStr.c_str());
        Serial.printf("  [WIFI_STA] >>> LINK WEB MONITOR : ws://%s:%u\n", g_activeIpStr.c_str(), wsPort);
        Serial.println("------------------------------------------------------------------\n");
    } else {
        g_activeIpStr = "192.168.4.1";
        Serial.printf("[WIFI_STA] [CANH BAO] STA '%s' chua ket noi; IP VNU %s hien chua truy cap duoc.\n", sta_ssid, STA_STATIC_IP.toString().c_str());
        Serial.printf("[WIFI_STA] Du phong: ket noi may tinh vao '%s' | ws://192.168.4.1:%u\n\n", ap_ssid, wsPort);
    }

    // 3. Khởi động WebSocket Server
    g_wsServer.begin(wsPort);
    Serial.printf("[WIFI_WS]  WebSocket Server da mo tai cong %u san sang nhan lenh tu Web!\n", wsPort);

    // 4. Khởi động UDP Transceiver
    g_udp.begin(g_udpPort);
    Serial.printf("[WIFI_UDP] UDP Port %u da san sang.\n\n", g_udpPort);
}

// ============================================================================
// 4. CHU TRÌNH PHÁT VÀ NHẬN DỮ LIỆU TELEMETRY
// ============================================================================
void telemetry_wifi_update(const TelemetryPacket& packet) {
    const uint32_t nowMs = millis();

    // 0. Trạng thái mạng: kiểm tra 1 lần/giây (bản cũ gọi WiFi.status() + localIP().toString()
    //    MỖI vòng loop() => cấp phát String liên tục, phân mảnh heap, tốn CPU vòng PID)
    if (nowMs - g_lastNetCheckMs >= 1000 || g_lastNetCheckMs == 0) {
        g_lastNetCheckMs = nowMs;
        g_staConnected = (WiFi.status() == WL_CONNECTED);
        g_activeIpStr = g_staConnected ? WiFi.localIP().toString() : AP_LOCAL_IP.toString();
    }

    // 1. Quản lý kết nối mới từ WebSocket Server (không chặn)
    if (g_wsServer.hasClient()) {
        WiFiClient newClient = g_wsServer.accept();
        bool slotFound = false;
        for (int i = 0; i < MAX_WS_CLIENTS; i++) {
            if (!g_wsClients[i] || !g_wsClients[i].connected()) {
                ws_release_slot(i);
                g_wsClients[i] = newClient;
                g_wsClients[i].setNoDelay(true); // Tắt Nagle: telemetry nhỏ, cần trễ thấp
                g_wsAcceptMs[i] = nowMs;
                slotFound = true;
                break;
            }
        }
        if (!slotFound) {
            newClient.stop();
        }
    }

    // 2. Handshake dang dở + nhận lệnh từ Web Browser
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (!g_wsClients[i]) continue;
        if (!g_wsClients[i].connected()) {
            if (g_wsHandshakeDone[i] || g_wsReqLen[i] > 0) ws_release_slot(i);
            continue;
        }
        if (!g_wsHandshakeDone[i]) {
            if (poll_ws_handshake(g_wsClients[i], i) < 0) {
                Serial.printf("[WIFI_WS] Client #%d handshake loi/timeout -> dong ket noi\n", i);
                ws_release_slot(i);
            }
            continue;
        }
        read_ws_incoming_frames(g_wsClients[i], i);
    }

    // 3. Nhận lệnh điều khiển qua mạng UDP
    int udpSize = g_udp.parsePacket();
    if (udpSize > 0) {
        char udpBuf[256];
        int len = g_udp.read(udpBuf, sizeof(udpBuf) - 1);
        if (len > 0) {
            udpBuf[len] = '\0';
            String msg = String(udpBuf);
            msg.trim();
            if (msg.length() > 0) {
                while (g_cmdQueue.size() >= 10) g_cmdQueue.pop_front();
                g_cmdQueue.push_back(msg);
            }
        }
    }

    // 4. Phát Telemetry JSON định kỳ (50 Hz)
    if (nowMs - g_lastBroadcastMs < BROADCAST_INTERVAL_MS) {
        return;
    }
    g_lastBroadcastMs = nowMs;

    bool anyClient = false;
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (g_wsClients[i] && g_wsHandshakeDone[i]) { anyClient = true; break; }
    }
    if (!anyClient) return; // Không ai xem => không tốn CPU đóng gói JSON

    // Bản cũ: buffer 800 byte + khối tune 240 byte => snprintf cắt cụt JSON khi số liệu dài,
    // Web JSON.parse() thất bại và bỏ qua gói tin âm thầm (đồ thị IMU giật/đứng).
    static char jsonMsg[1400];
    char tuneBuf[360];
    const int tn = autoTuner.buildStatusJson(tuneBuf, sizeof(tuneBuf));
    if (tn <= 0 || tn >= (int)sizeof(tuneBuf)) tuneBuf[0] = '\0';

    const size_t jsonLen = telemetry_build_json(packet, g_staConnected, g_activeIpStr.c_str(),
                                                tuneBuf, jsonMsg, sizeof(jsonMsg));
    if (jsonLen == 0) {
        if ((g_jsonErrors++ % 50) == 0) {
            Serial.println("[WIFI_WS] [LOI] JSON telemetry vuot buffer -> bo qua goi tin");
        }
        return;
    }

    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (g_wsClients[i] && g_wsClients[i].connected() && g_wsHandshakeDone[i]) {
            if (!send_ws_text_frame(g_wsClients[i], jsonMsg, jsonLen)) {
                Serial.printf("[WIFI_WS] Client #%d ghi TCP that bai (drop=%lu) -> dong ket noi\n",
                              i, (unsigned long)g_txDropped);
                ws_release_slot(i);
            }
        }
    }
}

// ============================================================================
// 5. QUẢN LÝ HÀNG ĐỢI LỆNH
// ============================================================================
bool telemetry_wifi_has_command() {
    return !g_cmdQueue.empty();
}

String telemetry_wifi_get_command() {
    if (g_cmdQueue.empty()) return "";
    String cmd = g_cmdQueue.front();
    g_cmdQueue.pop_front();
    return cmd;
}

void telemetry_wifi_broadcast_msg(const char* msg) {
    if (!msg) return;
    char text[256];
    const size_t len = telemetry_build_log_json(msg, text, sizeof(text)); // escape dấu ngoặc kép, backslash, ký tự điều khiển
    if (len == 0) return;
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (g_wsClients[i] && g_wsClients[i].connected() && g_wsHandshakeDone[i]) {
            if (!send_ws_text_frame(g_wsClients[i], text, len)) ws_release_slot(i);
        }
    }
}

String telemetry_wifi_get_ip() {
    return g_activeIpStr;
}

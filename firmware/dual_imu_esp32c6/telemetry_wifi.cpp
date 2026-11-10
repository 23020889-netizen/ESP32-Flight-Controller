#include "telemetry_wifi.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <mbedtls/sha1.h>
#include <mbedtls/base64.h>
#include <deque>

// ============================================================================
// 1. CẤU HÌNH HỆ THỐNG MẠNG & THÔNG SỐ
// ============================================================================
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

static std::deque<String> g_cmdQueue;
static uint32_t g_lastBroadcastMs = 0;
static const uint32_t BROADCAST_INTERVAL_MS = 40; // 25 Hz mượt mà cho 3D Monitor

static String g_activeIpStr = "192.168.4.1";

// ============================================================================
// 2. HELPER CHUẨN RFC6455 WEBSOCKET HANDSHAKE
// ============================================================================
static bool perform_ws_handshake(WiFiClient& client, int clientIndex) {
    String req = "";
    uint32_t startMs = millis();
    while (client.connected() && millis() - startMs < 400) {
        if (client.available()) {
            char c = (char)client.read();
            req += c;
            if (req.endsWith("\r\n\r\n")) break;
        }
    }

    if (!req.startsWith("GET")) {
        return false;
    }

    int keyIndex = req.indexOf("Sec-WebSocket-Key: ");
    if (keyIndex == -1) {
        return false;
    }

    int keyEnd = req.indexOf("\r\n", keyIndex);
    String clientKey = req.substring(keyIndex + 19, keyEnd);
    clientKey.trim();

    String concat = clientKey + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    unsigned char sha1_result[20];
    mbedtls_sha1((const unsigned char*)concat.c_str(), concat.length(), sha1_result);

    unsigned char base64_out[36];
    size_t olen = 0;
    mbedtls_base64_encode(base64_out, sizeof(base64_out), &olen, sha1_result, 20);
    base64_out[olen] = '\0';

    client.print("HTTP/1.1 101 Switching Protocols\r\n");
    client.print("Upgrade: websocket\r\n");
    client.print("Connection: Upgrade\r\n");
    client.print("Sec-WebSocket-Accept: ");
    client.print((char*)base64_out);
    client.print("\r\n\r\n");

    g_wsHandshakeDone[clientIndex] = true;
    Serial.printf("[WIFI_WS] Client #%d ket noi thanh cong tu Web Browser!\n", clientIndex);
    return true;
}

// Gửi 1 Frame Text Unmasked từ Server -> Browser (Non-blocking)
static void send_ws_text_frame(WiFiClient& client, const char* textPayload, size_t len) {
    if (!client.connected() || client.availableForWrite() < (int)(len + 10)) return;

    uint8_t header[10];
    size_t headerLen = 0;

    header[0] = 0x81; // FIN + Text frame
    if (len <= 125) {
        header[1] = (uint8_t)len;
        headerLen = 2;
    } else if (len <= 65535) {
        header[1] = 126;
        header[2] = (uint8_t)((len >> 8) & 0xFF);
        header[3] = (uint8_t)(len & 0xFF);
        headerLen = 4;
    } else {
        return;
    }

    client.write(header, headerLen);
    client.write((const uint8_t*)textPayload, len);
}

// Đọc Frame từ Web Browser
static void read_ws_incoming_frames(WiFiClient& client, int clientIndex) {
    while (client.available() >= 2) {
        uint8_t b1 = client.read();
        uint8_t b2 = client.read();

        uint8_t opcode = b1 & 0x0F;
        bool isMasked = (b2 & 0x80) != 0;
        uint64_t payloadLen = b2 & 0x7F;

        if (opcode == 0x08) { // Close frame
            client.stop();
            g_wsHandshakeDone[clientIndex] = false;
            return;
        }

        if (payloadLen == 126) {
            if (client.available() < 2) return;
            payloadLen = ((uint64_t)client.read() << 8) | client.read();
        }

        uint8_t maskKey[4] = {0};
        if (isMasked) {
            if (client.available() < 4) return;
            for (int i = 0; i < 4; i++) maskKey[i] = client.read();
        }

        if (payloadLen > 256) {
            for (uint64_t i = 0; i < payloadLen; i++) if (client.available()) client.read();
            return;
        }

        char buffer[256];
        size_t bytesRead = 0;
        while (bytesRead < payloadLen && client.available()) {
            buffer[bytesRead] = (char)client.read();
            if (isMasked) {
                buffer[bytesRead] ^= maskKey[bytesRead % 4];
            }
            bytesRead++;
        }
        buffer[bytesRead] = '\0';

        if (bytesRead > 0) {
            String cmd = String(buffer);
            cmd.trim();
            if (cmd.length() > 0) {
                // Giữ hàng đợi lệnh luôn mới nhất (xóa lệnh cũ nếu dồn quá 10)
                while (g_cmdQueue.size() >= 10) {
                    g_cmdQueue.pop_front();
                }
                g_cmdQueue.push_back(cmd);
            }
        }
    }
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

    // 2. Kết nối vào mạng TTDB
    Serial.printf("[WIFI_STA] Dang ket noi mang phong Lab '%s'...", sta_ssid);
    WiFi.begin(sta_ssid, sta_pass);

    uint32_t startMs = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startMs < 6000) {
        delay(250);
        Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        g_activeIpStr = WiFi.localIP().toString();
        Serial.println("------------------------------------------------------------------");
        Serial.printf("  [WIFI_STA] >>> KET NOI THANH CONG MANG TTDB: '%s'!\n", sta_ssid);
        Serial.printf("  [WIFI_STA] >>> DIA CHI IP ESP32 : %s\n", g_activeIpStr.c_str());
        Serial.printf("  [WIFI_STA] >>> LINK WEB MONITOR : ws://%s:%u\n", g_activeIpStr.c_str(), wsPort);
        Serial.println("------------------------------------------------------------------\n");
    } else {
        g_activeIpStr = "192.168.4.1";
        Serial.printf("[WIFI_STA] [CANH BAO] Khong tim thay song '%s' (hoac timeout).\n", sta_ssid);
        Serial.printf("[WIFI_STA] Hay bat WiFi may tinh vao: '%s' | Link: ws://192.168.4.1:%u\n\n", ap_ssid, wsPort);
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
    // 1. Quản lý kết nối mới từ WebSocket Server
    if (g_wsServer.hasClient()) {
        WiFiClient newClient = g_wsServer.accept();
        bool slotFound = false;
        for (int i = 0; i < MAX_WS_CLIENTS; i++) {
            if (!g_wsClients[i] || !g_wsClients[i].connected()) {
                g_wsClients[i] = newClient;
                g_wsHandshakeDone[i] = false;
                slotFound = true;
                perform_ws_handshake(g_wsClients[i], i);
                break;
            }
        }
        if (!slotFound) {
            newClient.stop();
        }
    }

    // 2. Nhận dữ liệu / Lệnh điều khiển từ Web Browser (Cần Analog / Nút Bấm / CLI)
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (g_wsClients[i] && g_wsClients[i].connected()) {
            if (g_wsHandshakeDone[i]) {
                read_ws_incoming_frames(g_wsClients[i], i);
            }
        }
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

    // 4. Bắn dữ liệu Telemetry JSON định kỳ lên Web (Rate Limiter: 25 Hz)
    uint32_t nowMs = millis();
    if (nowMs - g_lastBroadcastMs < BROADCAST_INTERVAL_MS) {
        return;
    }
    g_lastBroadcastMs = nowMs;

    char jsonMsg[320];
    snprintf(jsonMsg, sizeof(jsonMsg),
             "{\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f,\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,"
             "\"gx\":%.1f,\"gy\":%.1f,\"gz\":%.1f,\"z\":%.2f,\"vz\":%.2f,"
             "\"m\":[%.1f,%.1f,%.1f,%.1f],\"us\":[%u,%u,%u,%u],\"state\":\"%s\"}",
             packet.roll, packet.pitch, packet.yaw,
             packet.ax,   packet.ay,   packet.az,
             packet.gx,   packet.gy,   packet.gz,
             packet.z,    packet.vz,
             packet.m_pct[0] * 10.0f, packet.m_pct[1] * 10.0f, packet.m_pct[2] * 10.0f, packet.m_pct[3] * 10.0f,
             packet.m_us[0],  packet.m_us[1],  packet.m_us[2],  packet.m_us[3],
             packet.state_str);

    size_t jsonLen = strlen(jsonMsg);
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (g_wsClients[i] && g_wsClients[i].connected() && g_wsHandshakeDone[i]) {
            send_ws_text_frame(g_wsClients[i], jsonMsg, jsonLen);
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
    char text[128];
    snprintf(text, sizeof(text), "{\"log\":\"%s\"}", msg);
    size_t len = strlen(text);
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (g_wsClients[i] && g_wsClients[i].connected() && g_wsHandshakeDone[i]) {
            send_ws_text_frame(g_wsClients[i], text, len);
        }
    }
}

String telemetry_wifi_get_ip() {
    return g_activeIpStr;
}

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
static const IPAddress AP_BROADCAST_IP(192, 168, 4, 255);

static uint16_t g_udpPort = 14550;
static uint16_t g_wsPort = 81;

static WiFiUDP g_udp;
static WiFiServer g_wsServer(81);

#define MAX_WS_CLIENTS 4
static WiFiClient g_wsClients[MAX_WS_CLIENTS];
static bool g_wsHandshakeDone[MAX_WS_CLIENTS] = {false};

static std::deque<String> g_cmdQueue;
static uint32_t g_lastBroadcastMs = 0;
static const uint32_t BROADCAST_INTERVAL_MS = 40; // 25 Hz mượt mà cho 3D Monitor không nghẽn RAM

// Thông tin Máy Tính Chủ (Host PC EKF2 & Gamepad Server)
static IPAddress g_hostIp(0, 0, 0, 0);
static uint16_t  g_hostPort = 14550;
static bool      g_hostKnown = false;
static HostFeedback g_hostFeedback = {0};

// Rate limiter cho gói tin RAW IMU gửi lên PC
static uint32_t g_lastRawSendUs = 0;
static const uint32_t RAW_SEND_INTERVAL_US = 10000; // 100 Hz (10ms)

// ============================================================================
// 2. HELPER CHUẨN RFC6455 WEBSOCKET HANDSHAKE
// ============================================================================
static bool perform_ws_handshake(WiFiClient& client, int clientIndex) {
    String req = "";
    uint32_t startMs = millis();
    while (client.connected() && millis() - startMs < 300) {
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
    Serial.printf("[WIFI_WS] Client #%d connected & handshaked via WebSocket!\n", clientIndex);
    return true;
}

// Gửi 1 Frame dữ liệu Text Unmasked chuẩn RFC6455 từ Server -> Browser (Non-blocking)
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
        return; // Không gửi gói quá lớn
    }

    client.write(header, headerLen);
    client.write((const uint8_t*)textPayload, len);
}

// Đọc và giải mã Frame nhận được từ Browser (có chứa Mask Key)
static void read_ws_incoming_frames(WiFiClient& client, int clientIndex) {
    while (client.available() >= 2) {
        uint8_t b1 = client.read();
        uint8_t b2 = client.read();

        uint8_t opcode = b1 & 0x0F;
        bool isMasked = (b2 & 0x80) != 0;
        uint64_t payloadLen = b2 & 0x7F;

        if (opcode == 0x08) { // Connection Close Frame
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
                Serial.printf("[WIFI_WS] Received command from Web: '%s'\n", cmd.c_str());
                if (g_cmdQueue.size() < 10) {
                    g_cmdQueue.push_back(cmd);
                }
            }
        }
    }
}

// ============================================================================
// 3. KHỞI TẠO TELEMETRY WIFI (CHẾ ĐỘ KÉP AP_STA)
// ============================================================================
void telemetry_wifi_init(const char* ssid, const char* password, uint16_t udpPort, uint16_t wsPort) {
    g_udpPort = udpPort;
    g_wsPort = wsPort;

    Serial.println("\n[WIFI] ========================================================");
    Serial.println("[WIFI] DANG CAU HINH HE THONG MANG KHONG DAY CHO UAV DISTRIBUTED COMPUTING");

    // Bật chế độ kép AP_STA: Vừa phát SoftAP trực tiếp, vừa hỗ trợ bắt Router
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAPConfig(AP_LOCAL_IP, AP_GATEWAY, AP_SUBNET);
    
    if (password && strlen(password) >= 8) {
        WiFi.softAP(ssid, password);
    } else {
        WiFi.softAP(ssid);
    }

    // Thử kết nối Router phòng Lab nếu có sóng (Non-blocking hoặc nhanh)
    WiFi.begin("VNU-IS TTDBCLKT", "Ttdbclkt9");

    // Khởi động UDP Socket cho Raw Sensor Stream & PC Command Receiver
    g_udp.begin(g_udpPort);
    Serial.printf("[WIFI_UDP] Khoi dong UDP Transceiver tai Port %u\n", g_udpPort);

    // Khởi động WebSocket Server cho Giao diện 3D HITL Testbed Monitor
    g_wsServer.begin(g_wsPort);
    Serial.printf("[WIFI_WS]  Khoi dong WebSocket Server tai ws://192.168.4.1:%u\n", g_wsPort);
    Serial.printf("[WIFI]     SoftAP: '%s' | Pass: '%s' | IP: 192.168.4.1\n", ssid, (password ? password : "OPEN"));
    Serial.println("[WIFI] ========================================================\n");
}

// ============================================================================
// 4. BẮN DỮ LIỆU CẢM BIẾN THÔ (RAW IMU) LÊN MÁY TÍNH QUA UDP (100 HZ)
// ============================================================================
void telemetry_wifi_send_raw(const MpuData& mpu, const XsensData& xsens, const uint16_t m_us[4], bool is_armed) {
    uint32_t nowUs = micros();
    if (nowUs - g_lastRawSendUs < RAW_SEND_INTERVAL_US) {
        return;
    }
    g_lastRawSendUs = nowUs;

    // Gói tin RAW nhỏ gọn (~110 bytes), bắn UDP cực nhanh (< 0.1ms), không bị block bởi TCP
    // Cấu trúc: RAW,m_ax,m_ay,m_az,m_gx,m_gy,m_gz,x_r,x_p,x_y,x_ax,x_ay,x_az,x_gx,x_gy,x_gz,m1,m2,m3,m4,arm
    char rawBuf[256];
    int len = snprintf(rawBuf, sizeof(rawBuf),
        "RAW,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%u,%u,%u,%u,%d\n",
        mpu.ax, mpu.ay, mpu.az, mpu.gx, mpu.gy, mpu.gz,
        xsens.roll, xsens.pitch, xsens.yaw,
        xsens.ax, xsens.ay, xsens.az, xsens.gx, xsens.gy, xsens.gz,
        m_us[0], m_us[1], m_us[2], m_us[3],
        is_armed ? 1 : 0);

    if (len <= 0) return;

    // Nếu đã biết IP của máy tính chủ -> Bắn trực tiếp Unicast (tốc độ cao nhất)
    if (g_hostKnown) {
        g_udp.beginPacket(g_hostIp, g_hostPort);
        g_udp.write((const uint8_t*)rawBuf, len);
        g_udp.endPacket();
    } else {
        // Chưa nhận gói từ máy tính -> Bắn Broadcast để máy tính tự động bắt gói
        g_udp.beginPacket(AP_BROADCAST_IP, g_udpPort);
        g_udp.write((const uint8_t*)rawBuf, len);
        g_udp.endPacket();
    }
}

// ============================================================================
// 5. CHU TRÌNH PHÁT VÀ NHẬN DỮ LIỆU TELEMETRY
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

    // 2. Nhận dữ liệu / Lệnh điều khiển từ các Client WebSocket
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (g_wsClients[i] && g_wsClients[i].connected()) {
            if (g_wsHandshakeDone[i]) {
                read_ws_incoming_frames(g_wsClients[i], i);
            }
        }
    }

    // 3. Nhận phản hồi EKF2 & RC từ Máy Tính Chủ qua UDP (Non-blocking)
    int udpSize = g_udp.parsePacket();
    if (udpSize > 0) {
        // Tự động ghi nhớ IP và Port của Máy Tính Chủ
        g_hostIp = g_udp.remoteIP();
        g_hostPort = g_udp.remotePort();
        g_hostKnown = true;

        char udpBuf[256];
        int len = g_udp.read(udpBuf, sizeof(udpBuf) - 1);
        if (len > 0) {
            udpBuf[len] = '\0';
            String msg = String(udpBuf);
            msg.trim();

            // A. Gói tin tổ hợp siêu tốc: CMD,ekf_r,ekf_p,ekf_y,ekf_gx,ekf_gy,ekf_gz,thr,r_sp,p_sp,y_sp,arm
            if (msg.startsWith("CMD,") || msg.startsWith("cmd,")) {
                float er = 0, ep = 0, ey = 0, egx = 0, egy = 0, egz = 0;
                float thr = 0, rsp = 0, psp = 0, ysp = 0;
                int arm = 0;
                int count = sscanf(msg.c_str() + 4, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d",
                                   &er, &ep, &ey, &egx, &egy, &egz,
                                   &thr, &rsp, &psp, &ysp, &arm);
                if (count == 11) {
                    g_hostFeedback.roll = er;
                    g_hostFeedback.pitch = ep;
                    g_hostFeedback.yaw = ey;
                    g_hostFeedback.gx = egx;
                    g_hostFeedback.gy = egy;
                    g_hostFeedback.gz = egz;
                    g_hostFeedback.throttle = thr;
                    g_hostFeedback.roll_sp = rsp;
                    g_hostFeedback.pitch_sp = psp;
                    g_hostFeedback.yaw_sp = ysp;
                    g_hostFeedback.arm = arm;
                    g_hostFeedback.last_packet_ms = millis();
                    g_hostFeedback.has_ekf = true;
                    g_hostFeedback.has_rc = true;
                }
            }
            // B. Gói tin EKF2 độc lập: EKF,roll,pitch,yaw,gx,gy,gz
            else if (msg.startsWith("EKF,") || msg.startsWith("ekf,")) {
                float er = 0, ep = 0, ey = 0, egx = 0, egy = 0, egz = 0;
                int count = sscanf(msg.c_str() + 4, "%f,%f,%f,%f,%f,%f", &er, &ep, &ey, &egx, &egy, &egz);
                if (count == 6) {
                    g_hostFeedback.roll = er;
                    g_hostFeedback.pitch = ep;
                    g_hostFeedback.yaw = ey;
                    g_hostFeedback.gx = egx;
                    g_hostFeedback.gy = egy;
                    g_hostFeedback.gz = egz;
                    g_hostFeedback.last_packet_ms = millis();
                    g_hostFeedback.has_ekf = true;
                }
            }
            // C. Gói tin RC tay cầm độc lập: RC,throttle,roll_sp,pitch_sp,yaw_sp,arm
            else if (msg.startsWith("RC,") || msg.startsWith("rc,")) {
                float thr = 0, rsp = 0, psp = 0, ysp = 0;
                int arm = 0;
                int count = sscanf(msg.c_str() + 3, "%f,%f,%f,%f,%d", &thr, &rsp, &psp, &ysp, &arm);
                if (count == 5) {
                    g_hostFeedback.throttle = thr;
                    g_hostFeedback.roll_sp = rsp;
                    g_hostFeedback.pitch_sp = psp;
                    g_hostFeedback.yaw_sp = ysp;
                    g_hostFeedback.arm = arm;
                    g_hostFeedback.last_packet_ms = millis();
                    g_hostFeedback.has_rc = true;
                }
            }
            // D. Gói handshake / ping từ PC
            else if (msg.equalsIgnoreCase("CONNECT") || msg.equalsIgnoreCase("PING")) {
                Serial.printf("[WIFI_UDP] May tinh chu da ket noi tu IP: %s:%u\n", g_hostIp.toString().c_str(), g_hostPort);
            }
            // E. Lệnh điều khiển dạng CLI văn bản (arm, disarm, stop, sp, pid, throttle...)
            else if (msg.length() > 0 && g_cmdQueue.size() < 10) {
                Serial.printf("[WIFI_UDP] Received CLI command: '%s'\n", msg.c_str());
                g_cmdQueue.push_back(msg);
            }
        }
    }

    // 4. Bắn dữ liệu Telemetry JSON định kỳ cho Web 3D HITL Testbed Monitor (Rate Limiter: 25 Hz)
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
// 6. QUẢN LÝ PHẢN HỒI TỪ MÁY TÍNH CHỦ
// ============================================================================
bool telemetry_wifi_get_host_feedback(HostFeedback& out) {
    out = g_hostFeedback;
    return (millis() - g_hostFeedback.last_packet_ms < 1500 && g_hostFeedback.last_packet_ms > 0);
}

bool telemetry_wifi_is_host_connected() {
    return (g_hostKnown && (millis() - g_hostFeedback.last_packet_ms < 1500) && g_hostFeedback.last_packet_ms > 0);
}

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

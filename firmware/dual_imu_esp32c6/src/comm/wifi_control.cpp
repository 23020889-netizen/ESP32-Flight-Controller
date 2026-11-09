#include "wifi_control.hpp"
#include <ESPmDNS.h>
#include <mbedtls/sha1.h>
#include <mbedtls/base64.h>
#include <cstring>
#include <cmath>

WifiControlManager wifiControl;

static String computeWsAcceptKey(const String& key) {
    String input = key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    unsigned char sha1Result[20];
    mbedtls_sha1_context ctx;
    mbedtls_sha1_init(&ctx);
    mbedtls_sha1_starts(&ctx);
    mbedtls_sha1_update(&ctx, (const unsigned char*)input.c_str(), input.length());
    mbedtls_sha1_finish(&ctx, sha1Result);
    mbedtls_sha1_free(&ctx);

    unsigned char base64Result[40];
    size_t outLen = 0;
    mbedtls_base64_encode(base64Result, sizeof(base64Result), &outLen, sha1Result, 20);
    base64Result[outLen] = '\0';
    return String((char*)base64Result);
}

WifiControlManager::WifiControlManager()
    : server_(81),
      clientConnected_(false),
      wsHandshakeDone_(false),
      staConnected_(false),
      lastTelemetryMs_(0),
      lastRcPacketMs_(0),
      lastWifiCheckMs_(0),
      savedFlightMode_(FlightControlMode::DIRECT_MANUAL),
      wsRxSize_(0) {
}

void WifiControlManager::begin(const char* staSsid, const char* staPass,
                               const char* apSsid, const char* apPass,
                               const char* legacyBroker, uint16_t legacyPort) {
    Serial.println("\n[WIFI] ========================================================");
    Serial.println("[WIFI] KHOI TAO DIRECT WEBSOCKET FLIGHT CONTROLLER (PORT 81):");

    WiFi.mode(WIFI_AP_STA);

    // 1. Luon luon bat SoftAP du phong
    WiFi.softAP(apSsid, apPass);
    ipAddress_ = WiFi.softAPIP();
    Serial.printf("[WIFI] SoftAP Backup: '%s' (Pass: '%s') | IP: %s\n", apSsid, apPass, ipAddress_.toString().c_str());

    // 2. Fix cung dia chi Static IP 10.11.162.210 theo yeu cau
    IPAddress staticIP(10, 11, 162, 210);
    IPAddress gateway(10, 11, 162, 1);
    IPAddress subnet(255, 255, 255, 0);
    IPAddress dns(10, 11, 162, 1);
    if (!WiFi.config(staticIP, gateway, subnet, dns)) {
        Serial.println("[WIFI] [CANH BAO] Cau hinh Static IP that bai, se dung DHCP!");
    } else {
        Serial.println("[WIFI] >>> DA FIX CUNG STATIC IP: 10.11.162.210 (Gateway: 10.11.162.1)");
    }

    // 3. Quet cac mang Wi-Fi xung quanh de tim SSID chinh xac cua phong Lab
    Serial.println("[WIFI] Dang quet danh sach song Wi-Fi 2.4GHz...");
    int n = WiFi.scanNetworks();
    String chosenSsid = String(staSsid);
    Serial.printf("[WIFI] Tim thay %d mang Wi-Fi xung quanh:\n", n);

    bool foundTarget = false;
    for (int i = 0; i < n; ++i) {
        String s = WiFi.SSID(i);
        int rssi = WiFi.RSSI(i);
        Serial.printf("       [%02d] SSID: '%s' (%d dBm)\n", i + 1, s.c_str(), rssi);

        if (!foundTarget) {
            String sUpper = s;
            sUpper.toUpperCase();
            if (sUpper.indexOf("TTDBCLKT") >= 0 || s == "VNU-IS TTDBCLKT" || s == "TTDBCLKT9") {
                chosenSsid = s;
                foundTarget = true;
            }
        }
    }

    Serial.println("[WIFI] --------------------------------------------------------");
    Serial.printf("[WIFI] >>> DANG KET NOI VAO MANG: '%s' (Pass: '%s')...\n", chosenSsid.c_str(), staPass);
    WiFi.begin(chosenSsid.c_str(), staPass);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(300);
        Serial.print(".");
        attempts++;
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        staConnected_ = true;
        ipAddress_ = WiFi.localIP();
        Serial.println("[WIFI] ========================================================");
        Serial.printf("[WIFI] >>> KET NOI WIFI THANH CONG! SSID: '%s'\n", WiFi.SSID().c_str());
        Serial.printf("[WIFI] >>> IP STATION (DUNG IP NAY DE TRUY CAP TREN TRINH DUYET): %s\n", ipAddress_.toString().c_str());
        Serial.println("[WIFI] ========================================================");
    } else {
        staConnected_ = false;
        ipAddress_ = WiFi.softAPIP();
        Serial.println("[WIFI] [CHU Y] Chua nhan duoc IP tu Router. He thong duy tri SoftAP!");
        Serial.printf("[WIFI] Dung SoftAP IP: %s\n", ipAddress_.toString().c_str());
    }

    server_.begin();
    udp_.begin(8888);

    if (MDNS.begin("drone")) {
        MDNS.addService("ws", "tcp", 81);
        Serial.println("[WIFI] >>> Khoi tao mDNS thanh cong: ws://drone.local:81/ws");
    }

    Serial.printf("[WIFI] >>> WebSocket Server SAN SANG tai: ws://%s:81/ws (Direct 0ms Latency)\n", ipAddress_.toString().c_str());
    Serial.println("[WIFI] ========================================================\n");
}

void WifiControlManager::update(const FusedData& fusedData, FlightControllerManager& manager, QuadEscController& esc) {
    uint32_t now = millis();

    // Dinh ky 3s kiem tra lai trang thai Wi-Fi Station
    if (now - lastWifiCheckMs_ >= 3000) {
        lastWifiCheckMs_ = now;
        if (WiFi.status() == WL_CONNECTED) {
            if (!staConnected_) {
                staConnected_ = true;
                ipAddress_ = WiFi.localIP();
                Serial.printf("[WIFI] >>> DA KET NOI LAI WIFI STA: %s | IP: %s\n",
                              WiFi.SSID().c_str(), ipAddress_.toString().c_str());
            }
        } else {
            if (staConnected_) {
                staConnected_ = false;
                ipAddress_ = WiFi.softAPIP();
                Serial.println("[WIFI] [CANH BAO] Mat ket noi WiFi STA. Chuyen ve SoftAP IP.");
            }
        }
    }

    handleNewClient();
    handleWebSocketData(manager, esc);
    handleUdpData(manager, esc);

    // Failsafe Watchdog: Neu ARMED ma qua 3000ms hoan toan khong nhan goi RC -> Cat ga ve an toan
    if ((manager.getFlightMode() == FlightControlMode::DIRECT_MANUAL ||
         manager.getFlightMode() == FlightControlMode::ALTITUDE_HOLD) && esc.isArmed()) {
        if (lastRcPacketMs_ > 0 && (now - lastRcPacketMs_ > 3000)) {
            manager.setRcChannels(0.0f, 0.0f, 0.0f, 0.0f);
        }
    }

    // Phat telemetry dinh ky 50Hz (20ms) qua WebSocket giup Web 3D khong do tre
    if (now - lastTelemetryMs_ >= 20) {
        lastTelemetryMs_ = now;
        broadcastTelemetry(fusedData, manager, esc);
    }
}

void WifiControlManager::handleNewClient() {
    if (server_.hasClient()) {
        WiFiClient newClient = server_.accept();
        if (newClient) {
            if (client_ && client_.connected()) {
                Serial.println("[WIFI] Dong ket noi client cu de don client moi.");
                client_.stop();
            }
            client_ = newClient;
            client_.setNoDelay(true);
            clientConnected_ = true;
            wsHandshakeDone_ = false;
            wsRxSize_ = 0;
            Serial.printf("[WIFI] Client TCP moi ket noi tu: %s\n", client_.remoteIP().toString().c_str());
        }
    }

    if (client_ && !client_.connected()) {
        client_.stop();
        clientConnected_ = false;
        wsHandshakeDone_ = false;
        wsRxSize_ = 0;
        Serial.println("[WIFI] Client da ngat ket noi.");
    }
}

bool WifiControlManager::doWebSocketHandshake(const String& req) {
    int keyIdx = req.indexOf("Sec-WebSocket-Key: ");
    if (keyIdx == -1) return false;

    int keyEnd = req.indexOf("\r\n", keyIdx);
    if (keyEnd == -1) return false;

    String clientKey = req.substring(keyIdx + 19, keyEnd);
    clientKey.trim();

    String acceptKey = computeWsAcceptKey(clientKey);

    String resp = "HTTP/1.1 101 Switching Protocols\r\n";
    resp += "Upgrade: websocket\r\n";
    resp += "Connection: Upgrade\r\n";
    resp += "Sec-WebSocket-Accept: " + acceptKey + "\r\n\r\n";

    client_.print(resp);
    wsHandshakeDone_ = true;
    Serial.println("[WIFI] WebSocket Handshake RFC 6455 thanh cong! San sang dieu khien bay.");
    return true;
}

void WifiControlManager::sendWebSocketText(const String& text) {
    if (!client_ || !client_.connected() || !wsHandshakeDone_) return;

    size_t len = text.length();
    uint8_t header[10];
    size_t headerLen = 0;

    header[0] = 0x81; // FIN=1, Text frame (0x1)

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

    client_.write(header, headerLen);
    client_.write((const uint8_t*)text.c_str(), len);
}

void WifiControlManager::handleWebSocketData(FlightControllerManager& manager, QuadEscController& esc) {
    if (!client_ || !client_.connected() || client_.available() == 0) return;

    if (!wsHandshakeDone_) {
        String req = client_.readStringUntil('\n');
        while (client_.available()) {
            req += (char)client_.read();
            if (req.endsWith("\r\n\r\n")) break;
        }
        if (req.indexOf("Upgrade: websocket") != -1 || req.indexOf("upgrade: websocket") != -1) {
            doWebSocketHandshake(req);
        }
        return;
    }

    while (client_.available() > 0 && wsRxSize_ < WS_RX_BUFFER_SIZE) {
        int value = client_.read();
        if (value < 0) break;
        wsRxBuffer_[wsRxSize_++] = static_cast<uint8_t>(value);
    }

    size_t consumed = 0;
    while (wsRxSize_ - consumed >= 2) {
        const uint8_t b0 = wsRxBuffer_[consumed];
        const uint8_t b1 = wsRxBuffer_[consumed + 1];
        const int opcode = b0 & 0x0F;
        const bool masked = (b1 & 0x80) != 0;
        uint64_t payloadLen = b1 & 0x7F;
        size_t headerLen = 2;

        if (payloadLen == 126) {
            if (wsRxSize_ - consumed < 4) break;
            payloadLen = (static_cast<uint16_t>(wsRxBuffer_[consumed + 2]) << 8) |
                         wsRxBuffer_[consumed + 3];
            headerLen = 4;
        } else if (payloadLen == 127) {
            Serial.println("[WS] Tu choi frame payload 64-bit khong ho tro.");
            wsRxSize_ = 0;
            return;
        }

        const size_t maskLen = masked ? 4 : 0;
        const uint64_t frameLen = static_cast<uint64_t>(headerLen) + maskLen + payloadLen;
        if (frameLen > WS_RX_BUFFER_SIZE) {
            Serial.println("[WS] Frame vuot qua bo dem, dong ket noi de tranh mat dong bo.");
            client_.stop();
            clientConnected_ = false;
            wsHandshakeDone_ = false;
            wsRxSize_ = 0;
            return;
        }
        if (wsRxSize_ - consumed < frameLen) break;

        const size_t maskOffset = consumed + headerLen;
        const size_t payloadOffset = maskOffset + maskLen;
        char payload[513];
        if (payloadLen > sizeof(payload) - 1) {
            Serial.println("[WS] Payload command qua lon, bo qua frame.");
            consumed += static_cast<size_t>(frameLen);
            continue;
        }
        for (size_t i = 0; i < static_cast<size_t>(payloadLen); ++i) {
            uint8_t byte = wsRxBuffer_[payloadOffset + i];
            if (masked) byte ^= wsRxBuffer_[maskOffset + (i % 4)];
            payload[i] = static_cast<char>(byte);
        }
        payload[payloadLen] = '\0';

        if (opcode == 0x8) { // Close
            client_.stop();
            clientConnected_ = false;
            wsHandshakeDone_ = false;
            wsRxSize_ = 0;
            return;
        } else if (opcode == 0x9) { // Ping -> Pong
            uint8_t pong[2] = {0x8A, static_cast<uint8_t>(payloadLen)};
            client_.write(pong, sizeof(pong));
            if (payloadLen > 0) client_.write(reinterpret_cast<const uint8_t*>(payload), payloadLen);
        } else if (opcode == 0x1) { // Text command
            processCommandPayload(reinterpret_cast<const uint8_t*>(payload), payloadLen, manager, esc);
        }
        consumed += static_cast<size_t>(frameLen);
    }

    if (consumed > 0) {
        const size_t remaining = wsRxSize_ - consumed;
        memmove(wsRxBuffer_, wsRxBuffer_ + consumed, remaining);
        wsRxSize_ = remaining;
    }
}

void WifiControlManager::handleUdpData(FlightControllerManager& manager, QuadEscController& esc) {
    int packetSize = udp_.parsePacket();
    if (packetSize <= 0) return;

    char buffer[256];
    int len = udp_.read(buffer, sizeof(buffer) - 1);
    if (len > 0) {
        processCommandPayload(reinterpret_cast<const uint8_t*>(buffer), static_cast<size_t>(len), manager, esc);
    }
}

static void skipSpaces(char*& p) {
    while (*p == ' ' || *p == '\t') ++p;
}

static bool parseUInt32Field(char*& p, uint32_t& out, char delimiter) {
    skipSpaces(p);
    if (*p == '-' || *p == '+') return false;

    errno = 0;
    char* endPtr = nullptr;
    const unsigned long value = strtoul(p, &endPtr, 10);
    if (endPtr == p || errno == ERANGE || value > 0xFFFFFFFFUL) {
        return false;
    }
    p = endPtr;
    skipSpaces(p);
    if (*p != delimiter) return false;
    if (delimiter != '\0') ++p;
    out = static_cast<uint32_t>(value);
    return true;
}

static bool parseFloatField(char*& p, float& out, char delimiter) {
    skipSpaces(p);
    errno = 0;
    char* endPtr = nullptr;
    const float value = strtof(p, &endPtr);
    if (endPtr == p || errno == ERANGE || !std::isfinite(value)) {
        return false;
    }
    p = endPtr;
    skipSpaces(p);
    if (*p != delimiter) return false;
    if (delimiter != '\0') ++p;
    out = value;
    return true;
}

void WifiControlManager::processCommandString(const String& rawCmd, FlightControllerManager& manager, QuadEscController& esc) {
    processCommandPayload(reinterpret_cast<const uint8_t*>(rawCmd.c_str()), rawCmd.length(), manager, esc);
}

void WifiControlManager::processCommandPayload(const uint8_t* payload, size_t payloadLen, FlightControllerManager& manager, QuadEscController& esc) {
    constexpr size_t MAX_PACKET_LEN = 128;
    if (payload == nullptr || payloadLen == 0 || payloadLen >= MAX_PACKET_LEN) {
        return;
    }
    if (std::memchr(payload, '\0', payloadLen) != nullptr) {
        return;
    }

    char buf[MAX_PACKET_LEN];
    std::memcpy(buf, payload, payloadLen);
    buf[payloadLen] = '\0';

    while (payloadLen > 0 && (buf[payloadLen - 1] == '\r' || buf[payloadLen - 1] == '\n' ||
                              buf[payloadLen - 1] == ' '  || buf[payloadLen - 1] == '\t')) {
        buf[--payloadLen] = '\0';
    }
    if (payloadLen == 0) return;

    char* p = buf;

    // =========================================================================
    // 1. GIAO THUC DIEU KHIEN RC (HO TRO CA V2 VA V1 TRUC TIEP):
    //    V2: "RC,2,<sessionId>,<seq>,<throttle>,<roll>,<pitch>,<yaw>"
    //    V1: "RC,<throttle>,<roll>,<pitch>,<yaw>"
    // =========================================================================
    if (strncasecmp(p, "RC,", 3) == 0) {
        p += 3;
        skipSpaces(p);

        float thr = 0.0f, roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
        bool valid = false;

        if (strncmp(p, "2,", 2) == 0) {
            // Protocol V2
            p += 2;
            uint32_t sessId = 0, seq = 0;
            if (parseUInt32Field(p, sessId, ',') &&
                parseUInt32Field(p, seq, ',') &&
                parseFloatField(p, thr, ',') &&
                parseFloatField(p, roll, ',') &&
                parseFloatField(p, pitch, ',') &&
                parseFloatField(p, yaw, '\0')) {
                valid = true;
            }
        } else {
            // Protocol V1
            if (parseFloatField(p, thr, ',') &&
                parseFloatField(p, roll, ',') &&
                parseFloatField(p, pitch, ',') &&
                parseFloatField(p, yaw, '\0')) {
                valid = true;
            }
        }

        if (valid) {
            thr   = constrain(thr, 0.0f, 100.0f);
            roll  = constrain(roll, -100.0f, 100.0f);
            pitch = constrain(pitch, -100.0f, 100.0f);
            yaw   = constrain(yaw, -100.0f, 100.0f);

            uint32_t now = millis();
            lastRcPacketMs_ = now;

            // Truyen ngay lap tuc gia tri can vao flightManager
            manager.setRcChannels(thr, roll, pitch, yaw);
            esc.feedWatchdog();

            // In gia tri truc tiep ra Serial Monitor de kiem tra su thay doi cua can analog
            static uint32_t lastRcPrint = 0;
            static float lastPThr = -999.0f, lastPRoll = -999.0f, lastPPitch = -999.0f, lastPYaw = -999.0f;
            bool stickMoved = (fabsf(thr - lastPThr) >= 0.5f ||
                               fabsf(roll - lastPRoll) >= 0.5f ||
                               fabsf(pitch - lastPPitch) >= 0.5f ||
                               fabsf(yaw - lastPYaw) >= 0.5f);

            if ((stickMoved && (now - lastRcPrint >= 50)) || (now - lastRcPrint >= 1000)) {
                lastRcPrint = now;
                lastPThr = thr; lastPRoll = roll; lastPPitch = pitch; lastPYaw = yaw;

                float basePulse = 1050.0f + (thr * 9.50f);
                float dRoll  = roll * 1.8f;
                float dPitch = pitch * 1.8f;
                float dYaw   = yaw * 1.2f;
                uint16_t p1 = (uint16_t)constrain(basePulse - dRoll - dPitch + dYaw, 1050.0f, 2000.0f);
                uint16_t p2 = (uint16_t)constrain(basePulse + dRoll + dPitch + dYaw, 1050.0f, 2000.0f);
                uint16_t p3 = (uint16_t)constrain(basePulse + dRoll - dPitch - dYaw, 1050.0f, 2000.0f);
                uint16_t p4 = (uint16_t)constrain(basePulse - dRoll + dPitch - dYaw, 1050.0f, 2000.0f);

                Serial.printf(">> [RC_STICK] Thr:%5.1f%% | Roll:%+5.1f%% | Pitch:%+5.1f%% | Yaw:%+5.1f%% | %s | PWM:[%u,%u,%u,%u]us\n",
                              thr, roll, pitch, yaw,
                              esc.isArmed() ? "ARMED" : "DISARMED",
                              p1, p2, p3, p4);
            }
        }
        return;
    }

    // =========================================================================
    // 2. LỆNH MỞ KHÓA ĐỘNG CƠ: "ARM"
    // =========================================================================
    if (strcasecmp(buf, "ARM") == 0) {
        if (manager.getFlightMode() == FlightControlMode::EMERGENCY_DISARM ||
            manager.getFlightMode() == FlightControlMode::MANUAL_CLI) {
            manager.setFlightMode(savedFlightMode_);
        }
        lastRcPacketMs_ = millis();
        esc.arm();
        esc.feedWatchdog();
        Serial.println("[WIFI_CMD] >>> DA NHAN LENH ARM TUC THI! Mo khoa 4 dong co.");
        return;
    }

    // =========================================================================
    // 3. LỆNH KHÓA KHẨN CẤP: "DISARM" / "STOP" / "KILL"
    // =========================================================================
    if (strcasecmp(buf, "DISARM") == 0 || strcasecmp(buf, "STOP") == 0 || strcasecmp(buf, "KILL") == 0) {
        manager.setFlightMode(FlightControlMode::EMERGENCY_DISARM);
        esc.disarm();
        Serial.println("[WIFI_CMD] >>> DA KHOA KHAN CAP (DISARMED)! Ca 4 dong co ngat ve 900us.");
        return;
    }

    // =========================================================================
    // 4. LỆNH CHUYỂN CHẾ ĐỘ BAY: "MODE,<name>"
    // =========================================================================
    if (strncasecmp(buf, "MODE,", 5) == 0) {
        const char* modeName = buf + 5;
        while (*modeName == ' ') modeName++;
        if (strcasecmp(modeName, "DIRECT_MANUAL") == 0 || strcasecmp(modeName, "DIRECT") == 0) {
            savedFlightMode_ = FlightControlMode::DIRECT_MANUAL;
            manager.setFlightMode(FlightControlMode::DIRECT_MANUAL);
            Serial.println("[WIFI_CMD] >>> Chuyen mode: DIRECT_MANUAL (Direct Stick -> Motors).");
        } else if (strcasecmp(modeName, "ALTITUDE") == 0 || strcasecmp(modeName, "ALT_HOLD") == 0 ||
                   strcasecmp(modeName, "ALT") == 0 || strcasecmp(modeName, "ITSMC_ATT") == 0 ||
                   strcasecmp(modeName, "STABILIZED") == 0) {
            savedFlightMode_ = FlightControlMode::ALTITUDE_HOLD;
            manager.setFlightMode(FlightControlMode::ALTITUDE_HOLD);
            Serial.println("[WIFI_CMD] >>> Chuyen mode: ALTITUDE_HOLD (Cascaded PID).");
        } else if (strcasecmp(modeName, "POSHOLD") == 0 || strcasecmp(modeName, "ITSMC_POS") == 0) {
            savedFlightMode_ = FlightControlMode::ITSMC_POSITION_HOLD;
            manager.setFlightMode(FlightControlMode::ITSMC_POSITION_HOLD);
            Serial.println("[WIFI_CMD] >>> Chuyen mode: ITSMC_POSITION_HOLD.");
        } else if (strcasecmp(modeName, "MANUAL_CLI") == 0) {
            savedFlightMode_ = FlightControlMode::MANUAL_CLI;
            manager.setFlightMode(FlightControlMode::MANUAL_CLI);
            Serial.println("[WIFI_CMD] >>> Chuyen mode: MANUAL_CLI.");
        }
        return;
    }

    // =========================================================================
    // 5. LỆNH TEST MOTOR RIÊNG LẺ: "TEST,<motor_id>,<pct>,<sec>"
    // =========================================================================
    if (strncasecmp(buf, "TEST,", 5) == 0) {
        char* tPtr = buf + 5;
        uint32_t mId = 1, sec = 3;
        float pct = 10.0f;
        if (parseUInt32Field(tPtr, mId, ',') && parseFloatField(tPtr, pct, ',')) {
            (void)parseUInt32Field(tPtr, sec, '\0');
            manager.setFlightMode(FlightControlMode::MANUAL_CLI);
            esc.testSingleMotor(static_cast<uint8_t>(mId), pct, sec * 1000);
        }
        return;
    }
}

void WifiControlManager::broadcastTelemetry(const FusedData& fusedData, const FlightControllerManager& manager, const QuadEscController& esc) {
    if (!client_ || !client_.connected() || !wsHandshakeDone_) return;

    const char* modeStr = "DIRECT_MANUAL";
    FlightControlMode m = manager.getFlightMode();
    if (m == FlightControlMode::ALTITUDE_HOLD) modeStr = "ALT_HOLD";
    else if (m == FlightControlMode::ITSMC_POSITION_HOLD) modeStr = "ITSMC_POS";
    else if (m == FlightControlMode::MANUAL_CLI) modeStr = "MANUAL_CLI";
    else if (m == FlightControlMode::EMERGENCY_DISARM) modeStr = "DISARMED";

    char jsonBuf[350];
    snprintf(jsonBuf, sizeof(jsonBuf),
             "{\"r\":%.2f,\"p\":%.2f,\"y\":%.2f,\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,\"gx\":%.1f,\"gy\":%.1f,\"gz\":%.1f,\"alt\":%.2f,\"vz\":%.2f,\"m1\":%d,\"m2\":%d,\"m3\":%d,\"m4\":%d,\"state\":\"%s\",\"mode\":\"%s\",\"arm\":%d,\"ip\":\"%s\"}",
             fusedData.roll, fusedData.pitch, fusedData.yaw,
             fusedData.ax, fusedData.ay, fusedData.az,
             fusedData.gx, fusedData.gy, fusedData.gz,
             fusedData.altitudeZ, fusedData.velocityVz,
             esc.getMotor(0).getCurrentPulseUs(),
             esc.getMotor(1).getCurrentPulseUs(),
             esc.getMotor(2).getCurrentPulseUs(),
             esc.getMotor(3).getCurrentPulseUs(),
             esc.getStateStr(),
             modeStr,
             esc.isArmed() ? 1 : 0,
             ipAddress_.toString().c_str());

    String msg(jsonBuf);
    sendWebSocketText(msg);
}

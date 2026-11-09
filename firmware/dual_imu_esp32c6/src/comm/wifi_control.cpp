#include "wifi_control.hpp"
#include <ESPmDNS.h>
#include <mbedtls/sha1.h>
#include <mbedtls/base64.h>
#include <cstring>

WifiControlManager wifiControl;

static void mqttCallback(char* topic, byte* payload, unsigned int length) {
    if (length == 0 || length >= 256) return;
    wifiControl.processCommandPayload(reinterpret_cast<const uint8_t*>(payload), length, RC_SOURCE_MQTT, flightManager, quadMotors);
}

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
      mqttClient_(mqttNetClient_),
      clientConnected_(false),
      wsHandshakeDone_(false),
      staConnected_(false),
      lastTelemetryMs_(0),
      lastRcPacketMs_(0),
      activeSessionId_(0),
      lastValidRcSeq_(0),
      activeTransport_(RC_SOURCE_NONE),
      lastMqttRcMs_(0),
      rcAcceptCount_(0),
      rcDropStaleCount_(0),
      rcRejectFormatCount_(0),
      lastWifiCheckMs_(0),
      lastMqttReconnectMs_(0),
      savedFlightMode_(FlightControlMode::DIRECT_MANUAL),
      mqttBrokerPort_(1883),
      wsRxSize_(0) {
    strncpy(mqttBrokerHost_, "10.11.162.171", sizeof(mqttBrokerHost_));
}

void WifiControlManager::begin(const char* staSsid, const char* staPass,
                               const char* apSsid, const char* apPass,
                               const char* mqttBroker, uint16_t mqttPort) {
    if (mqttBroker && strlen(mqttBroker) > 0) {
        strncpy(mqttBrokerHost_, mqttBroker, sizeof(mqttBrokerHost_) - 1);
        mqttBrokerHost_[sizeof(mqttBrokerHost_) - 1] = '\0';
    }
    if (mqttPort > 0) mqttBrokerPort_ = mqttPort;

    Serial.println("\n[WIFI] ========================================================");
    Serial.println("[WIFI] KHOI TAO DUAL WI-FI & GIAO THUC KHONG DAY MQTT:");

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

        // Uu tien bat mang co chu TTDBCLKT (vi du: VNU-IS TTDBCLKT hoac TTDBCLKT9)
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

    // 4. Cau hinh MQTT Client (non-blocking timeout de khong bao gio treo WebSocket/RC)
    mqttNetClient_.setTimeout(100);
    mqttClient_.setSocketTimeout(1);
    mqttClient_.setServer(mqttBrokerHost_, mqttBrokerPort_);
    mqttClient_.setBufferSize(1024);
    mqttClient_.setCallback(mqttCallback);

    Serial.printf("[MQTT] Broker Target: %s:%d\n", mqttBrokerHost_, mqttBrokerPort_);
    Serial.println("[MQTT] Subscribe Topics: 'isilab/drone/rc', 'isilab/drone/cmd'");
    Serial.println("[MQTT] Publish Topic   : 'isilab/drone/telemetry'");
    Serial.println("[WIFI] ========================================================\n");
}

void WifiControlManager::setMqttBroker(const char* host, uint16_t port) {
    char newHost[64] = {0};
    if (host && strlen(host) > 0) {
        strncpy(newHost, host, sizeof(newHost) - 1);
        newHost[sizeof(newHost) - 1] = '\0';
    } else {
        strncpy(newHost, mqttBrokerHost_, sizeof(newHost) - 1);
    }
    uint16_t newPort = (port > 0) ? port : mqttBrokerPort_;

    if (strcmp(mqttBrokerHost_, newHost) == 0 && mqttBrokerPort_ == newPort && mqttClient_.connected()) {
        return;
    }

    strncpy(mqttBrokerHost_, newHost, sizeof(mqttBrokerHost_) - 1);
    mqttBrokerHost_[sizeof(mqttBrokerHost_) - 1] = '\0';
    mqttBrokerPort_ = newPort;

    if (mqttClient_.connected()) {
        mqttClient_.disconnect();
    }
    mqttClient_.setServer(mqttBrokerHost_, mqttBrokerPort_);
    Serial.printf("[MQTT] Cap nhat dia chi Broker moi: %s:%d\n", mqttBrokerHost_, mqttBrokerPort_);
}

void WifiControlManager::reconnectMqtt() {
    if (WiFi.status() != WL_CONNECTED && !WiFi.softAPgetStationNum()) return;

    mqttNetClient_.setTimeout(100);
    String clientId = "ESP32C6_Drone_" + String((uint32_t)ESP.getEfuseMac(), HEX);

    if (mqttClient_.connect(clientId.c_str(), "isilab/drone/status", 1, true, "offline")) {
        Serial.println("[MQTT] >>> KET NOI BROKER THANH CONG!");
        mqttClient_.publish("isilab/drone/status", "online", true);
        mqttClient_.subscribe("isilab/drone/rc", 0);
        mqttClient_.subscribe("isilab/drone/cmd", 0);
        Serial.println("[MQTT] >>> Da dang ky topic: 'isilab/drone/rc' va 'isilab/drone/cmd'");
    }
}

void WifiControlManager::handleMqtt(FlightControllerManager& manager, QuadEscController& esc) {
    uint32_t now = millis();
    if (!mqttClient_.connected()) {
        // Neu drone dang ARMED hoac WebSocket dang ket noi truc tiep, KHONG goi connect()
        // de tranh TCP SYN bi Windows Firewall chan gay nghen luong dieu khien WebSocket!
        if (esc.isArmed() || (clientConnected_ && wsHandshakeDone_)) {
            return;
        }
        if (now - lastMqttReconnectMs_ >= 5000) {
            lastMqttReconnectMs_ = now;
            reconnectMqtt();
        }
    } else {
        mqttClient_.loop();
    }
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
    handleMqtt(manager, esc);

    // Failsafe Watchdog: Neu ARMED ma qua 2000ms khong nhan goi RC -> Cat ga ve an toan
    if ((manager.getFlightMode() == FlightControlMode::DIRECT_MANUAL ||
         manager.getFlightMode() == FlightControlMode::ALTITUDE_HOLD) && esc.isArmed()) {
        if (lastRcPacketMs_ > 0 && (now - lastRcPacketMs_ > 2000)) {
            manager.setRcChannels(0.0f, 0.0f, 0.0f, 0.0f);
        }
    }

    // Phat telemetry dinh ky 50Hz (20ms) qua MQTT & WebSocket giup Web 3D khong do tre
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
    Serial.println("[WIFI] WebSocket Handshake RFC 6455 thanh cong!");
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
            processCommandPayload(reinterpret_cast<const uint8_t*>(payload), payloadLen, RC_SOURCE_WEBSOCKET, manager, esc);
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
        processCommandPayload(reinterpret_cast<const uint8_t*>(buffer), static_cast<size_t>(len), RC_SOURCE_WEBSOCKET, manager, esc);
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

static inline bool isSeqNewer(uint32_t candidate, uint32_t previous) {
    return static_cast<int32_t>(candidate - previous) > 0;
}

void WifiControlManager::processCommandString(const String& rawCmd, FlightControllerManager& manager, QuadEscController& esc, RcTransportSource source) {
    processCommandPayload(reinterpret_cast<const uint8_t*>(rawCmd.c_str()), rawCmd.length(), source, manager, esc);
}

void WifiControlManager::processCommandPayload(const uint8_t* payload, size_t payloadLen, RcTransportSource source, FlightControllerManager& manager, QuadEscController& esc) {
    constexpr size_t MAX_PACKET_LEN = 128;
    if (payload == nullptr || payloadLen == 0 || payloadLen >= MAX_PACKET_LEN) {
        return;
    }
    if (std::memchr(payload, '\0', payloadLen) != nullptr) {
        // Tu choi payload chua embedded NUL
        return;
    }

    char buf[MAX_PACKET_LEN];
    std::memcpy(buf, payload, payloadLen);
    buf[payloadLen] = '\0';

    // Bo ky tu CR, LF, space o cuoi chuoi
    while (payloadLen > 0 && (buf[payloadLen - 1] == '\r' || buf[payloadLen - 1] == '\n' ||
                              buf[payloadLen - 1] == ' '  || buf[payloadLen - 1] == '\t')) {
        buf[--payloadLen] = '\0';
    }
    if (payloadLen == 0) return;

    char* p = buf;

    // =========================================================================
    // 1. GIAO THUC DIEU KHIEN RC V2 & V1:
    //    V2: "RC,2,<sessionId>,<seq>,<throttle>,<roll>,<pitch>,<yaw>"
    //    V1: "RC,<throttle>,<roll>,<pitch>,<yaw>"
    // =========================================================================
    if (strncmp(p, "RC,", 3) == 0 || strncmp(p, "rc,", 3) == 0) {
        p += 3;
        skipSpaces(p);

        // Kiem tra neu bat dau bang "2," -> Protocol V2
        if (strncmp(p, "2,", 2) == 0) {
            p += 2;
            uint32_t sessId = 0, seq = 0;
            float thr = 0.0f, roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
            if (!parseUInt32Field(p, sessId, ','))  { rcRejectFormatCount_++; return; }
            if (!parseUInt32Field(p, seq, ','))     { rcRejectFormatCount_++; return; }
            if (!parseFloatField(p, thr, ','))      { rcRejectFormatCount_++; return; }
            if (!parseFloatField(p, roll, ','))     { rcRejectFormatCount_++; return; }
            if (!parseFloatField(p, pitch, ','))    { rcRejectFormatCount_++; return; }
            if (!parseFloatField(p, yaw, '\0'))     { rcRejectFormatCount_++; return; }

            // Range validation nghiem ngat theo protocol:
            // throttle: 0..100%, roll/pitch/yaw: -100..100%
            if (thr < 0.0f || thr > 100.0f ||
                roll < -100.0f || roll > 100.0f ||
                pitch < -100.0f || pitch > 100.0f ||
                yaw < -100.0f || yaw > 100.0f) {
                rcRejectFormatCount_++;
                return;
            }

            uint32_t now = millis();
            activeTransport_ = source;
            if (source == RC_SOURCE_MQTT) {
                lastMqttRcMs_ = now;
            }

            // SESSION & SEQUENCE DEDUPLICATION (RFC 1982):
            // Ca WebSocket va MQTT cung gui (sessionId, seq). Goi nao den truoc se duoc xu ly ngay lap tuc (0ms delay),
            // goi trung lap den sau tren kenh kia se tu dong bi deduplicate boi isSeqNewer!
            if (sessId != activeSessionId_) {
                activeSessionId_ = sessId;
                lastValidRcSeq_ = seq;
                Serial.printf("[RC_SESSION] >>> BAT DAU PHIEN MOI SessionID=0x%08X (Seq baseline: %u, Src: %s)\n",
                              sessId, seq, (source == RC_SOURCE_MQTT) ? "MQTT" : "WS");
            } else {
                if (!isSeqNewer(seq, lastValidRcSeq_)) {
                    rcDropStaleCount_++;
                    return; // Goi tin trung lap giua WS/MQTT hoac goi cu den muon -> Bo qua an toan
                }
                lastValidRcSeq_ = seq;
            }

            // Ghi goi tin hop le vao Atomic Snapshot bao ve bang FreeRTOS Critical Section
            RcSnapshot snap{};
            snap.sessionId    = sessId;
            snap.seq          = seq;
            snap.receivedAtMs = now;
            snap.throttle     = thr;
            snap.roll         = roll;
            snap.pitch        = pitch;
            snap.yaw          = yaw;
            snap.source       = source;
            snap.valid        = true;

            manager.updateRcSnapshot(snap);
            lastRcPacketMs_ = now;
            rcAcceptCount_++;

            static uint32_t lastRcPrint = 0;
            static float lastPThr = -999.0f, lastPRoll = -999.0f, lastPPitch = -999.0f, lastPYaw = -999.0f;
            bool stickMoved = (fabsf(thr - lastPThr) >= 0.5f ||
                               fabsf(roll - lastPRoll) >= 0.5f ||
                               fabsf(pitch - lastPPitch) >= 0.5f ||
                               fabsf(yaw - lastPYaw) >= 0.5f);

            // In tuc thi khi can gat di chuyen (toi da 20 dong/giay de khong nghen UART),
            // hoac dinh ky moi 1000ms neu giu nguyen (Heartbeat)
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

                Serial.printf(">> [RC_STICK] Thr:%5.1f%% | Roll:%+5.1f%% | Pitch:%+5.1f%% | Yaw:%+5.1f%% | %s | PWM:[%u,%u,%u,%u]us (%s)\n",
                              thr, roll, pitch, yaw,
                              esc.isArmed() ? "ARMED" : "DISARMED",
                              p1, p2, p3, p4,
                              (source == RC_SOURCE_MQTT) ? "MQTT" : "WS");
            }
            return;
        } else {
            // Fallback cho Protocol V1: RC,<thr>,<roll>,<pitch>,<yaw>
            char* v1Ptr = p;
            float v1Thr = 0.0f, v1Roll = 0.0f, v1Pitch = 0.0f, v1Yaw = 0.0f;
            if (parseFloatField(v1Ptr, v1Thr, ',') &&
                parseFloatField(v1Ptr, v1Roll, ',') &&
                parseFloatField(v1Ptr, v1Pitch, ',') &&
                parseFloatField(v1Ptr, v1Yaw, '\0')) {

                v1Thr   = constrain(v1Thr, 0.0f, 100.0f);
                v1Roll  = constrain(v1Roll, -100.0f, 100.0f);
                v1Pitch = constrain(v1Pitch, -100.0f, 100.0f);
                v1Yaw   = constrain(v1Yaw, -100.0f, 100.0f);

                uint32_t now = millis();
                RcSnapshot snap{};
                snap.sessionId    = 0xCAFE0001;
                snap.seq          = ++lastValidRcSeq_;
                snap.receivedAtMs = now;
                snap.throttle     = v1Thr;
                snap.roll         = v1Roll;
                snap.pitch        = v1Pitch;
                snap.yaw          = v1Yaw;
                snap.source       = source;
                snap.valid        = true;

                manager.updateRcSnapshot(snap);
                lastRcPacketMs_ = now;
                rcAcceptCount_++;

                static uint32_t lastV1Print = 0;
                if (now - lastV1Print >= 100) {
                    lastV1Print = now;
                    Serial.printf(">> [RC_STICK_V1] Thr:%5.1f%% | Roll:%+5.1f%% | Pitch:%+5.1f%% | Yaw:%+5.1f%% (%s)\n",
                                  v1Thr, v1Roll, v1Pitch, v1Yaw, (source == RC_SOURCE_MQTT) ? "MQTT" : "WS");
                }
                return;
            }

            rcRejectFormatCount_++;
            return;
        }
    }

    // =========================================================================
    // 2. LỆNH MỞ KHÓA ĐỘNG CƠ: "ARM"
    // =========================================================================
    if (strcasecmp(buf, "ARM") == 0) {
        if (manager.getFlightMode() == FlightControlMode::EMERGENCY_DISARM ||
            manager.getFlightMode() == FlightControlMode::MANUAL_CLI) {
            manager.setFlightMode(savedFlightMode_);
        }
        // Lam moi RC snapshot timestamp ngay khi ARM de tranh Failsafe Level 3 ngat som
        RcSnapshot snap{};
        if (!manager.getRcSnapshotAtomic(snap)) {
            snap.sessionId = activeSessionId_;
            snap.seq = lastValidRcSeq_;
            snap.throttle = 0.0f;
            snap.roll = 0.0f;
            snap.pitch = 0.0f;
            snap.yaw = 0.0f;
            snap.source = source;
        }
        snap.receivedAtMs = millis();
        snap.valid = true;
        manager.updateRcSnapshot(snap);
        lastRcPacketMs_ = snap.receivedAtMs;

        esc.arm();
        esc.feedWatchdog();
        Serial.println("[WIRELESS_CMD] >>> DA NHAN LENH ARM TUC THI! Mo khoa 4 dong co.");
        return;
    }

    // =========================================================================
    // 3. LỆNH KHÓA KHẨN CẤP: "DISARM" / "STOP" / "KILL"
    // =========================================================================
    if (strcasecmp(buf, "DISARM") == 0 || strcasecmp(buf, "STOP") == 0 || strcasecmp(buf, "KILL") == 0) {
        manager.setFlightMode(FlightControlMode::EMERGENCY_DISARM);
        esc.disarm();
        Serial.println("[WIRELESS_CMD] >>> DA KHOA KHAN CAP (DISARMED)! Ca 4 dong co ngat ve 900us.");
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
            Serial.println("[WIRELESS_CMD] >>> Chuyen mode: DIRECT_MANUAL (Bench Test - No Prop).");
        } else if (strcasecmp(modeName, "ALTITUDE") == 0 || strcasecmp(modeName, "ALT_HOLD") == 0 ||
                   strcasecmp(modeName, "ALT") == 0 || strcasecmp(modeName, "ITSMC_ATT") == 0 ||
                   strcasecmp(modeName, "STABILIZED") == 0) {
            savedFlightMode_ = FlightControlMode::ALTITUDE_HOLD;
            manager.setFlightMode(FlightControlMode::ALTITUDE_HOLD);
            Serial.println("[WIRELESS_CMD] >>> Chuyen mode: ALTITUDE_HOLD (Cascaded PID).");
        } else if (strcasecmp(modeName, "POSHOLD") == 0 || strcasecmp(modeName, "ITSMC_POS") == 0) {
            savedFlightMode_ = FlightControlMode::ITSMC_POSITION_HOLD;
            manager.setFlightMode(FlightControlMode::ITSMC_POSITION_HOLD);
            Serial.println("[WIRELESS_CMD] >>> Chuyen mode: ITSMC_POSITION_HOLD.");
        } else if (strcasecmp(modeName, "MANUAL_CLI") == 0) {
            savedFlightMode_ = FlightControlMode::MANUAL_CLI;
            manager.setFlightMode(FlightControlMode::MANUAL_CLI);
            Serial.println("[WIRELESS_CMD] >>> Chuyen mode: MANUAL_CLI.");
        }
        return;
    }

    // =========================================================================
    // 5. LỆNH CẤU HÌNH BROKER MQTT: "MQTT,<host>,<port>"
    // =========================================================================
    if (strncasecmp(buf, "MQTT,", 5) == 0) {
        const char* pMqtt = buf + 5;
        while (*pMqtt == ' ') pMqtt++;
        char hostBuf[64] = {0};
        uint16_t portVal = 1883;
        const char* commaPos = strchr(pMqtt, ',');
        if (commaPos != nullptr) {
            size_t hostLen = commaPos - pMqtt;
            if (hostLen >= sizeof(hostBuf)) hostLen = sizeof(hostBuf) - 1;
            strncpy(hostBuf, pMqtt, hostLen);
            hostBuf[hostLen] = '\0';
            long pNum = strtol(commaPos + 1, nullptr, 10);
            if (pNum > 0 && pNum <= 65535) portVal = static_cast<uint16_t>(pNum);
        } else {
            strncpy(hostBuf, pMqtt, sizeof(hostBuf) - 1);
            hostBuf[sizeof(hostBuf) - 1] = '\0';
        }
        if (strlen(hostBuf) > 0) {
            setMqttBroker(hostBuf, portVal);
        }
        return;
    }

    // =========================================================================
    // 6. LỆNH TEST MOTOR RIÊNG LẺ: "TEST,<motor_id>,<pct>,<sec>"
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

    // 1. Phat qua giao thuc MQTT khong day
    if (mqttClient_.connected()) {
        mqttClient_.publish("isilab/drone/telemetry", jsonBuf);
    }

    // 2. Phat qua giao thuc WebSocket khong day
    if (client_ && client_.connected() && wsHandshakeDone_) {
        String msg(jsonBuf);
        sendWebSocketText(msg);
    }
}

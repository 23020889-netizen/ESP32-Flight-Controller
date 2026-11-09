#include "wifi_control.hpp"
#include <ESPmDNS.h>
#include <mbedtls/sha1.h>
#include <mbedtls/base64.h>
#include <cstring>

WifiControlManager wifiControl;

static void mqttCallback(char* topic, byte* payload, unsigned int length) {
    if (length == 0 || length >= 512) return;
    char buf[513];
    memcpy(buf, payload, length);
    buf[length] = '\0';
    String cmd(buf);
    wifiControl.processCommandString(cmd, flightManager, quadMotors);
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

    // 4. Cau hinh MQTT Client
    mqttClient_.setServer(mqttBrokerHost_, mqttBrokerPort_);
    mqttClient_.setBufferSize(1024);
    mqttClient_.setCallback(mqttCallback);

    Serial.printf("[MQTT] Broker Target: %s:%d\n", mqttBrokerHost_, mqttBrokerPort_);
    Serial.println("[MQTT] Subscribe Topics: 'isilab/drone/rc', 'isilab/drone/cmd'");
    Serial.println("[MQTT] Publish Topic   : 'isilab/drone/telemetry'");
    Serial.println("[WIFI] ========================================================\n");
}

void WifiControlManager::setMqttBroker(const char* host, uint16_t port) {
    if (host && strlen(host) > 0) {
        strncpy(mqttBrokerHost_, host, sizeof(mqttBrokerHost_) - 1);
        mqttBrokerHost_[sizeof(mqttBrokerHost_) - 1] = '\0';
    }
    if (port > 0) mqttBrokerPort_ = port;
    mqttClient_.disconnect();
    mqttClient_.setServer(mqttBrokerHost_, mqttBrokerPort_);
    Serial.printf("[MQTT] Cap nhat dia chi Broker moi: %s:%d\n", mqttBrokerHost_, mqttBrokerPort_);
}

void WifiControlManager::reconnectMqtt() {
    if (WiFi.status() != WL_CONNECTED && !WiFi.softAPgetStationNum()) return;

    Serial.printf("[MQTT] Dang ket noi toi Broker: %s:%d...\n", mqttBrokerHost_, mqttBrokerPort_);
    String clientId = "ESP32C6_Drone_" + String((uint32_t)ESP.getEfuseMac(), HEX);

    if (mqttClient_.connect(clientId.c_str(), "isilab/drone/status", 1, true, "offline")) {
        Serial.println("[MQTT] >>> KET NOI BROKER THANH CONG!");
        mqttClient_.publish("isilab/drone/status", "online", true);
        mqttClient_.subscribe("isilab/drone/rc", 0);
        mqttClient_.subscribe("isilab/drone/cmd", 0);
        Serial.println("[MQTT] >>> Da dang ky topic: 'isilab/drone/rc' va 'isilab/drone/cmd'");
    } else {
        Serial.printf("[MQTT] Ket noi that bai, rc=%d (thu lai sau 3s)\n", mqttClient_.state());
    }
}

void WifiControlManager::handleMqtt(FlightControllerManager& manager, QuadEscController& esc) {
    uint32_t now = millis();
    if (!mqttClient_.connected()) {
        if (now - lastMqttReconnectMs_ >= 3000) {
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

    // Phat telemetry dinh ky 40Hz (25ms) qua MQTT & WebSocket
    if (now - lastTelemetryMs_ >= 25) {
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
            processCommandString(String(payload), manager, esc);
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
        buffer[len] = '\0';
        processCommandString(String(buffer), manager, esc);
    }
}

void WifiControlManager::processCommandString(const String& rawCmd, FlightControllerManager& manager, QuadEscController& esc) {
    String cmd = rawCmd;
    cmd.trim();
    if (cmd.length() == 0) return;

    // 1. Lenh RC tu Tay Cam: "RC,<throttle>,<roll>,<pitch>,<yaw>"
    if (cmd.startsWith("RC,") || cmd.startsWith("rc,")) {
        float t = 0.0f, r = 0.0f, p = 0.0f, y = 0.0f;
        int parsed = sscanf(cmd.c_str() + 3, "%f,%f,%f,%f", &t, &r, &p, &y);
        if (parsed >= 4) {
            lastRcPacketMs_ = millis();
            esc.feedWatchdog();
            manager.setRcChannels(t, r, p, y);

            static uint32_t lastRcPrint = 0;
            if (millis() - lastRcPrint > 1000) {
                lastRcPrint = millis();
                float rcT = 0.0f, rcR = 0.0f, rcP = 0.0f, rcY = 0.0f;
                manager.getRcChannels(rcT, rcR, rcP, rcY);
                const char* modeName = "MANUAL_CLI";
                switch (manager.getFlightMode()) {
                    case FlightControlMode::DIRECT_MANUAL: modeName = "DIRECT_MANUAL"; break;
                    case FlightControlMode::ALTITUDE_HOLD: modeName = "ALT_HOLD_PID"; break;
                    case FlightControlMode::ITSMC_POSITION_HOLD: modeName = "ITSMC_POSITION"; break;
                    case FlightControlMode::EMERGENCY_DISARM: modeName = "EMERGENCY_DISARM"; break;
                    case FlightControlMode::MANUAL_CLI: break;
                }
                Serial.printf("[RC_WIRELESS] T:%.1f R:%.1f P:%.1f Y:%.1f | ESC:%s MODE:%s\n",
                              rcT, rcR, rcP, rcY, esc.isArmed() ? "ARMED" : "DISARMED", modeName);
            }
        }
        return;
    }

    // 2. Lenh ARM
    if (cmd.equalsIgnoreCase("ARM") || cmd.equalsIgnoreCase("arm")) {
        if (manager.getFlightMode() == FlightControlMode::EMERGENCY_DISARM ||
            manager.getFlightMode() == FlightControlMode::MANUAL_CLI) {
            manager.setFlightMode(savedFlightMode_);
        }
        esc.arm();
        Serial.println("[WIRELESS_CMD] >>> DA NHAN LENH ARM TUC THI! Mo khoa 4 dong co, kich hoat bo dieu khien PID.");
        return;
    }

    // 3. Lenh DISARM / EMERGENCY STOP / KILL
    if (cmd.equalsIgnoreCase("DISARM") || cmd.equalsIgnoreCase("disarm") ||
        cmd.equalsIgnoreCase("STOP")   || cmd.equalsIgnoreCase("KILL")) {
        manager.setFlightMode(FlightControlMode::EMERGENCY_DISARM);
        esc.disarm();
        Serial.println("[WIRELESS_CMD] >>> DA KHOA KHAN CAP (DISARMED)! Ca 4 dong co ngat ve 900us.");
        return;
    }

    // 4. Lenh Chuyen Mode: "MODE,<name>"
    if (cmd.startsWith("MODE,") || cmd.startsWith("mode,")) {
        String modeName = cmd.substring(5);
        modeName.trim();
        if (modeName.equalsIgnoreCase("DIRECT_MANUAL") || modeName.equalsIgnoreCase("DIRECT")) {
            savedFlightMode_ = FlightControlMode::DIRECT_MANUAL;
            manager.setFlightMode(FlightControlMode::DIRECT_MANUAL);
            Serial.println("[WIRELESS_CMD] Chuyen sang che do: DIRECT_MANUAL");
        } else if (modeName.equalsIgnoreCase("ALTITUDE") || modeName.equalsIgnoreCase("ALT_HOLD") ||
                   modeName.equalsIgnoreCase("ALT") || modeName.equalsIgnoreCase("ITSMC_ATT") ||
                   modeName.equalsIgnoreCase("STABILIZED")) {
            savedFlightMode_ = FlightControlMode::ALTITUDE_HOLD;
            manager.setFlightMode(FlightControlMode::ALTITUDE_HOLD);
            Serial.println("[WIRELESS_CMD] Chuyen sang che do: ALTITUDE_HOLD");
        } else if (modeName.equalsIgnoreCase("POSHOLD") || modeName.equalsIgnoreCase("ITSMC_POS")) {
            savedFlightMode_ = FlightControlMode::ITSMC_POSITION_HOLD;
            manager.setFlightMode(FlightControlMode::ITSMC_POSITION_HOLD);
            Serial.println("[WIRELESS_CMD] Chuyen sang che do: ITSMC_POSITION_HOLD");
        } else if (modeName.equalsIgnoreCase("MANUAL_CLI")) {
            savedFlightMode_ = FlightControlMode::MANUAL_CLI;
            manager.setFlightMode(FlightControlMode::MANUAL_CLI);
            Serial.println("[WIRELESS_CMD] Chuyen sang che do: MANUAL_CLI");
        }
        return;
    }

    // 5. Lenh doi Broker MQTT: "MQTT,<host>,<port>"
    if (cmd.startsWith("MQTT,") || cmd.startsWith("mqtt,")) {
        char host[64] = {0};
        int port = 1883;
        if (sscanf(cmd.c_str() + 5, "%63[^,],%d", host, &port) >= 1) {
            setMqttBroker(host, (uint16_t)port);
        }
        return;
    }

    // 6. Lenh Test Motor don le: "TEST,<motor_id>,<pct>,<sec>"
    if (cmd.startsWith("TEST,") || cmd.startsWith("test,")) {
        int mId = 1, sec = 3;
        float pct = 10.0f;
        sscanf(cmd.c_str() + 5, "%d,%f,%d", &mId, &pct, &sec);
        manager.setFlightMode(FlightControlMode::MANUAL_CLI);
        esc.testSingleMotor(mId, pct, sec * 1000);
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

#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiServer.h>
#include <WiFiUdp.h>
#include "PubSubClient.h"
#include "../supervisor/flight_controller_manager.hpp"
#include "../actuators/esc_pwm.h"
#include "../estimation/px4_ekf.h"

/**
 * @brief Quản lý kết nối Wi-Fi Station & SoftAP, WebSocket RFC 6455 và giao thức MQTT không dây trên ESP32-C6.
 * Nhận lệnh RC và Arm từ tay cầm qua MQTT topics: 'isilab/drone/rc', 'isilab/drone/cmd'
 * Phát telemetry thời gian thực qua MQTT topic: 'isilab/drone/telemetry'
 */
class WifiControlManager {
public:
    WifiControlManager();

    // Khởi tạo Wi-Fi Station & SoftAP cùng cấu hình MQTT Broker
    void begin(const char* staSsid = "TTDBCLKT9", const char* staPass = "Ttdbclkt9",
               const char* apSsid = "ISILAB_DRONE_AP", const char* apPass = "isilab2026",
               const char* mqttBroker = "10.11.162.171", uint16_t mqttPort = 1883);

    // Xử lý gói tin mạng và phát telemetry trong vòng lặp loop()
    void update(const FusedData& fusedData, FlightControllerManager& manager, QuadEscController& esc);

    // Phát telemetry JSON tới MQTT Broker và Web Client
    void broadcastTelemetry(const FusedData& fusedData, const FlightControllerManager& manager, const QuadEscController& esc);

    bool isClientConnected() { return clientConnected_ || mqttClient_.connected(); }
    bool isStaConnected() const { return staConnected_; }
    bool isMqttConnected() { return mqttClient_.connected(); }
    IPAddress getIpAddress() const { return ipAddress_; }
    uint32_t getLastRcPacketTimeMs() const { return lastRcPacketMs_; }

    void setMqttBroker(const char* host, uint16_t port);
    void processCommandString(const String& cmd, FlightControllerManager& manager, QuadEscController& esc);

private:
    WiFiServer   server_;
    WiFiClient   client_;
    WiFiUDP      udp_;
    WiFiClient   mqttNetClient_;
    PubSubClient mqttClient_;

    bool       clientConnected_;
    bool       wsHandshakeDone_;
    bool       staConnected_;
    IPAddress  ipAddress_;
    uint32_t   lastTelemetryMs_;
    uint32_t   lastRcPacketMs_;
    uint32_t   lastWifiCheckMs_;
    uint32_t   lastMqttReconnectMs_;
    FlightControlMode savedFlightMode_;

    char       mqttBrokerHost_[64];
    uint16_t   mqttBrokerPort_;

    // TCP là byte stream: giữ lại WebSocket frame chưa nhận đủ giữa các lần update().
    static constexpr size_t WS_RX_BUFFER_SIZE = 1024;
    uint8_t wsRxBuffer_[WS_RX_BUFFER_SIZE];
    size_t wsRxSize_;

    void handleNewClient();
    void handleWebSocketData(FlightControllerManager& manager, QuadEscController& esc);
    void handleUdpData(FlightControllerManager& manager, QuadEscController& esc);
    void handleMqtt(FlightControllerManager& manager, QuadEscController& esc);
    void reconnectMqtt();

    bool doWebSocketHandshake(const String& req);
    void sendWebSocketText(const String& text);
};

extern WifiControlManager wifiControl;

#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiServer.h>
#include <WiFiUdp.h>
#include "../supervisor/flight_controller_manager.hpp"
#include "../actuators/esc_pwm.h"
#include "../estimation/px4_ekf.h"

/**
 * @brief Quản lý kết nối Wi-Fi Station (bắt mạng chung TTDBCLKT9) & SoftAP dự phòng,
 * WebSocket RFC 6455 & UDP Server trên ESP32-C6.
 * Cho phép điều khiển bay trực tiếp từ tay cầm qua Web UI và phát Telemetry thời gian thực.
 */
class WifiControlManager {
public:
    WifiControlManager();

    // Khởi tạo Wi-Fi Station kết nối mạng lab (mặc định TTDBCLKT9 / pass Ttdbclkt9) + SoftAP dự phòng
    void begin(const char* staSsid = "TTDBCLKT9", const char* staPass = "Ttdbclkt9",
               const char* apSsid = "ISILAB_DRONE_AP", const char* apPass = "isilab2026");

    // Xử lý gói tin mạng và phát telemetry trong vòng lặp loop()
    void update(const FusedData& fusedData, FlightControllerManager& manager, QuadEscController& esc);

    // Phát telemetry JSON tới Web Client
    void broadcastTelemetry(const FusedData& fusedData, const FlightControllerManager& manager, const QuadEscController& esc);

    bool isClientConnected() const { return clientConnected_; }
    bool isStaConnected() const { return staConnected_; }
    IPAddress getIpAddress() const { return ipAddress_; }
    uint32_t getLastRcPacketTimeMs() const { return lastRcPacketMs_; }

private:
    WiFiServer server_;
    WiFiClient client_;
    WiFiUDP    udp_;
    bool       clientConnected_;
    bool       wsHandshakeDone_;
    bool       staConnected_;
    IPAddress  ipAddress_;
    uint32_t   lastTelemetryMs_;
    uint32_t   lastRcPacketMs_;
    uint32_t   lastWifiCheckMs_;
    FlightControlMode savedFlightMode_;

    // TCP là byte stream: giữ lại WebSocket frame chưa nhận đủ giữa các lần update().
    static constexpr size_t WS_RX_BUFFER_SIZE = 1024;
    uint8_t wsRxBuffer_[WS_RX_BUFFER_SIZE];
    size_t wsRxSize_;

    void handleNewClient();
    void handleWebSocketData(FlightControllerManager& manager, QuadEscController& esc);
    void handleUdpData(FlightControllerManager& manager, QuadEscController& esc);
    void processCommandString(const String& cmd, FlightControllerManager& manager, QuadEscController& esc);

    bool doWebSocketHandshake(const String& req);
    void sendWebSocketText(const String& text);
};

extern WifiControlManager wifiControl;

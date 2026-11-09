#pragma once

#include <stdint.h>
#include <stdbool.h>

enum RcTransportSource : uint8_t {
    RC_SOURCE_NONE = 0,
    RC_SOURCE_MQTT = 1,
    RC_SOURCE_WEBSOCKET = 2,
    RC_SOURCE_SERIAL = 3
};

struct RcSnapshot {
    uint32_t sessionId;
    uint32_t seq;
    uint32_t receivedAtMs;
    float throttle; // 0..100 (%)
    float roll;     // -100..100 (%)
    float pitch;    // -100..100 (%)
    float yaw;      // -100..100 (%)
    RcTransportSource source;
    bool valid;
};

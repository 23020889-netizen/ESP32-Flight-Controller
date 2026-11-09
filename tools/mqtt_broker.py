"""
ISILAB UAV HITL Testbed - Standalone MQTT Broker
Chạy broker MQTT cục bộ hỗ trợ đồng thời:
  - Cổng TCP 1883: Cho Drone ESP32-C6 (Giao thức MQTT thuần)
  - Cổng WebSocket 9001: Cho Trình duyệt Web GCS (MQTT over WebSockets)
"""
import warnings
warnings.filterwarnings("ignore")

import asyncio
import logging
import sys
from amqtt.broker import Broker

config = {
    'listeners': {
        'default': {
            'type': 'tcp',
            'bind': '0.0.0.0:1883',
            'max_connections': 50,
        },
        'ws': {
            'type': 'ws',
            'bind': '0.0.0.0:9001',
            'max_connections': 50,
        },
    },
    'sys_interval': 15,
    'auth': {
        'allow-anonymous': True,
    },
    'topic-check': {
        'enabled': False
    }
}

async def start_broker():
    broker = Broker(config)
    await broker.start()
    print("\n" + "=" * 70, flush=True)
    print("🚁 [ISILAB] DUAL-CHANNEL MQTT BROKER ĐANG HOẠT ĐỘNG HOÀN HẢO:", flush=True)
    print("   📡 TCP Port 1883       : Sẵn sàng nhận kết nối từ ESP32-C6 Drone", flush=True)
    print("   🌐 WebSocket Port 9001 : Sẵn sàng nhận kết nối từ Web GCS Simulator", flush=True)
    print("=" * 70 + "\n", flush=True)
    while True:
        await asyncio.sleep(3600)

if __name__ == '__main__':
    logging.basicConfig(level=logging.ERROR)
    logging.getLogger('amqtt').setLevel(logging.ERROR)
    try:
        asyncio.run(start_broker())
    except KeyboardInterrupt:
        print("\n[BROKER] Dừng MQTT Broker an toàn.")

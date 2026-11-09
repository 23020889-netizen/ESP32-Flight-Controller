#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
===============================================================================
ISILAB UAV - TEST HAI GIA THIET TRUYEN THONG & DIEU KHIEN TAY CAM
===============================================================================
Gia thiet 1: Du lieu 2 can analog khong den duoc Broker / ESP32.
Gia thiet 2: ESP32 da nhan duoc du lieu nhung khong the xuat xung PWM tuong ung.
"""

import sys
import time
import json
try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("Thieu thu vien paho-mqtt! Chay: pip install paho-mqtt")
    sys.exit(1)

BROKER_HOST = "127.0.0.1"
BROKER_PORT = 1883

def main():
    print("=" * 75)
    print("  ISILAB UAV: SCAN HE THONG & PHAN TICH 2 TRUONG HOP LOI")
    print("=" * 75)

    client = mqtt.Client()
    web_rc_packets = []
    esp_telem_packets = []

    def on_connect(cl, ud, flags, rc):
        print(f"[OK] Ket noi thanh cong vao MQTT Broker tai {BROKER_HOST}:{BROKER_PORT}")
        cl.subscribe("isilab/drone/#")

    def on_message(cl, ud, msg):
        topic = msg.topic
        payload = msg.payload.decode("utf-8", errors="ignore")
        if topic == "isilab/drone/rc":
            web_rc_packets.append((time.time(), payload))
            print(f"  -> [WEB GCS GAI CAN] {payload.strip()}")
        elif topic == "isilab/drone/telemetry":
            try:
                d = json.loads(payload)
                esp_telem_packets.append((time.time(), d))
            except Exception:
                pass

    client.on_connect = on_connect
    client.on_message = on_message

    try:
        client.connect(BROKER_HOST, BROKER_PORT, 60)
        client.loop_start()
    except Exception as e:
        print(f"[LOI] Khong the ket noi Broker tai {BROKER_HOST}:{BROKER_PORT} ({e})")
        return

    # -------------------------------------------------------------------------
    # BƯỚC 1: KIỂM TRA TRẠNG THÁI HIỆN TẠI CỦA ESP32
    # -------------------------------------------------------------------------
    print("\n[BUOC 1] Dang do tim nhip tim (Telemetry) tu ESP32 trong 3 giay...")
    time.sleep(3.0)

    if not esp_telem_packets:
        print("  [CANH BAO] Khong co Telemetry tu ESP32 tren Broker!")
        print("  -> Nguyen nhan: ESP32 chua ket noi duoc vao Broker hoac chua bat nguon.")
    else:
        latest = esp_telem_packets[-1][1]
        print(f"  [OK] Bat duoc Telemetry ESP32:")
        print(f"       IP: {latest.get('ip')} | State: {latest.get('state')} | Mode: {latest.get('mode')} | Arm: {latest.get('arm')}")
        print(f"       Xung dong co: M1={latest.get('m1')}us | M2={latest.get('m2')}us | M3={latest.get('m3')}us | M4={latest.get('m4')}us")

    # -------------------------------------------------------------------------
    # BƯỚC 2: KIỂM TRA TRƯỜNG HỢP 1 (GAT CAN TAY CAM TREN WEB GCS)
    # -------------------------------------------------------------------------
    print("\n" + "-" * 75)
    print("[BUOC 2 - KIEM TRA TRUONG HOP 1] DỮ LIỆU TAY CẦM TỪ WEB CÓ BẮN VÀO BROKER?")
    print(">>> YEU CAU: Click vao tab Web Simulator va GAT CAN GA / ROLL / PITCH ngay bay gio! <<<")
    print("-" * 75)

    web_rc_packets.clear()
    for s in range(5, 0, -1):
        print(f"  ... Dang theo doi topic 'isilab/drone/rc' ({s}s con lai)...", end="\r", flush=True)
        time.sleep(1.0)
    print(" " * 75, end="\r")

    if len(web_rc_packets) > 0:
        print(f"  [KET LUAN TH1: AM TINH - KHONG PHAI LOI NAY]")
        print(f"  Web GCS DA truyen thanh cong {len(web_rc_packets)} goi tin RC vao Broker!")
        print(f"  Goi tin cuoi: '{web_rc_packets[-1][1].strip()}'")
    else:
        print(f"  [KET LUAN TH1: DUONG TINH - DAY LA DIEM NGHEN THU NHAT!]")
        print(f"  Khong co bat ky goi tin nao duoc Web ban vao topic 'isilab/drone/rc'!")
        print(f"  Ly do: 1) Nut 'MQTT: Local' tren Web chua Online (click vao de connect).")
        print(f"         2) Chrome dang chay ngam va pause Gamepad API (click chuot vao Web).")
        print(f"         3) Tay cam chua duoc nut ARM kich hoat.")

    # -------------------------------------------------------------------------
    # BƯỚC 3: KIỂM TRA TRƯỜNG HỢP 2 (BƠM THẲNG RC VÀO ESP32 ĐỂ XEM XUẤT XUNG)
    # -------------------------------------------------------------------------
    print("\n" + "-" * 75)
    print("[BUOC 3 - KIEM TRA TRUONG HOP 2] NẾU ESP32 NHẬN RC THÌ CÓ XUẤT XUNG KHÔNG?")
    print("Script se chu dong bom truc tiep: MODE,DIRECT_MANUAL -> ARM -> RC,50.0 (Ga 50%)...")
    print("-" * 75)

    client.publish("isilab/drone/cmd", "MODE,DIRECT_MANUAL")
    time.sleep(0.1)
    client.publish("isilab/drone/cmd", "ARM")
    time.sleep(0.3)

    esp_telem_packets.clear()
    t_start = time.time()
    while time.time() - t_start < 2.0:
        client.publish("isilab/drone/rc", "RC,50.0,0.0,0.0,0.0")
        time.sleep(0.025)

    # An toan
    client.publish("isilab/drone/cmd", "DISARM")
    time.sleep(0.3)
    client.loop_stop()

    if esp_telem_packets:
        last_m = esp_telem_packets[-1][1]
        m1_val = last_m.get("m1", 0)
        print(f"  Ket qua Telemetry khi bom Ga 50%:")
        print(f"  Xung dong co: M1={m1_val}us | M2={last_m.get('m2')}us | M3={last_m.get('m3')}us | M4={last_m.get('m4')}us")
        if m1_val >= 1400:
            print(f"\n  [KET LUAN TH2: AM TINH] ESP32 XUAT XUNG HOAN HAO ({m1_val}us) KHI NHAN RC!")
            print(f"  He thong firmware nhan RC va phan hoi xung rat tot.")
        else:
            print(f"\n  [KET LUAN TH2: DUONG TINH - DAY LA NGUYEN NHAN CHINH!]")
            print(f"  ESP32 da nhan lenh nhung van kẹt xung o {m1_val}us!")
            print(f"  Ly do: Ban build nap chua cap nhat bo parser moi vao Flash ESP32.")
    else:
        print("  Khong the do duoc phan hoi cua ESP32 trong Buoc 3.")

    print("\n" + "=" * 75)

if __name__ == "__main__":
    main()


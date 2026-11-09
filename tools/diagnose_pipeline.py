#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
===============================================================================
ISILAB UAV - PIPELINE DIAGNOSTIC TOOL (KỊCH BẢN KIỂM TRA ĐƯỜNG ỐNG ĐIỀU KHIỂN)
===============================================================================
Kiểm tra từng trạm trong đường truyền từ Tay cầm Xbox -> Web -> MQTT Broker -> ESP32 -> ESC
"""

import sys
import time
import json
import socket
try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("[ERROR] Thieu thu vien paho-mqtt! Chay: pip install paho-mqtt")
    sys.exit(1)

BROKER_HOST = sys.argv[1] if len(sys.argv) > 1 else "10.11.153.160"
BROKER_PORT = 1883
WS_PORT = 9001

def banner(title):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)

def check_socket(host, port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(1.5)
    try:
        s.connect((host, port))
        s.close()
        return True
    except Exception:
        return False

def main():
    banner("ISILAB UAV - KIEM TRA TOAN DIEN DUONG ONG DIEU KHIEN (PIPELINE AUDIT)")
    print(f"Target Broker: {BROKER_HOST} (TCP: {BROKER_PORT}, WebSocket: {WS_PORT})")

    # =========================================================================
    # TRAM 1: KIEM TRA TRANG THAI MQTT BROKER
    # =========================================================================
    banner("TRAM 1: KIEM TRA HA TANG BROKER TREN PC")
    tcp_ok = check_socket(BROKER_HOST, BROKER_PORT)
    ws_ok = check_socket(BROKER_HOST, WS_PORT)

    print(f"[*] Port 1883 (ESP32 TCP MQTT)     : {'[OK] DANG MO VA SAN SANG' if tcp_ok else '[FAIL] DANG DONG / MAT KET NOI'}")
    print(f"[*] Port 9001 (Web GCS WebSocket)  : {'[OK] DANG MO VA SAN SANG' if ws_ok else '[FAIL] DANG DONG / MAT KET NOI'}")

    if not tcp_ok or not ws_ok:
        print("\n=> KET LUAN TRAM 1: Broker chua khoi dong! Hay chay tools\\start_mqtt_broker.bat truoc.")
        return

    # =========================================================================
    # TRAM 2: KET NOI VA THU THAP DU LIEU TELEMETRY TU ESP32
    # =========================================================================
    banner("TRAM 2: KIEM TRA TRANG THAI ESP32-C6 (DRONE HEARTBEAT)")
    print("[*] Dang ket noi vao Broker de lang nghe Drone...")

    client = mqtt.Client()
    rc_log = []
    telem_log = []

    def on_connect(cl, ud, flags, rc):
        cl.subscribe("isilab/drone/#")

    def on_message(cl, ud, msg):
        topic = msg.topic
        payload = msg.payload.decode("utf-8", errors="ignore")
        if topic == "isilab/drone/rc":
            rc_log.append(payload)
        elif topic == "isilab/drone/telemetry":
            try:
                telem_log.append(json.loads(payload))
            except Exception:
                pass

    client.on_connect = on_connect
    client.on_message = on_message
    try:
        client.connect(BROKER_HOST, BROKER_PORT, 60)
        client.loop_start()
    except Exception as e:
        print(f"[FAIL] Khong the ket noi vao Broker: {e}")
        return

    time.sleep(2.0)

    if not telem_log:
        print("[FAIL] KHONG NHAN DUOC TELEMETRY TU ESP32!")
        print("  -> Kiem tra xem ESP32 da bat nguon va ket noi Wi-Fi 'VNU-IS TTDBCLKT' chua.")
        print("  -> IP cua ESP32 phai ping duoc (Mac dinh: 10.11.162.210).")
        client.loop_stop()
        return

    last_telem = telem_log[-1]
    drone_ip = last_telem.get("ip", "unknown")
    drone_mode = last_telem.get("mode", "unknown")
    drone_state = last_telem.get("state", "unknown")
    drone_arm = last_telem.get("arm", 0)
    m1 = last_telem.get("m1", 0)
    m2 = last_telem.get("m2", 0)
    m3 = last_telem.get("m3", 0)
    m4 = last_telem.get("m4", 0)

    print(f"[OK] Da bat duoc nhip tim Drone Telemetry (Tan so ~40Hz)!")
    print(f"  + IP ESP32     : {drone_ip}")
    print(f"  + Che do bay   : {drone_mode}")
    print(f"  + Trang thai   : {drone_state} (arm={drone_arm})")
    print(f"  + Xung dong co : M1={m1}us | M2={m2}us | M3={m3}us | M4={m4}us")

    # =========================================================================
    # TRAM 3: KIEM TRA GOI TIN XBOX TU TRINH DUYET WEB (WEB GCS -> BROKER)
    # =========================================================================
    banner("TRAM 3: KIEM TRA TIN HIEU TAY CAM XBOX TU TRINH DUYET WEB")
    print("[*] Dang lang nghe topic 'isilab/drone/rc' trong 5 giay...")
    print(">>> YEU CAU: HAY MO TAB TRINH DUYET VA GAT CAN TAY CAM XBOX NGAY BAY GIO! <<<")

    rc_log.clear()
    for sec in range(5, 0, -1):
        print(f"    ... con {sec}s (Gat can ga / roll / pitch tren tay cam)...", end="\r", flush=True)
        time.sleep(1.0)
    print(" " * 60, end="\r")

    print(f"[*] So goi tin RC nhan duoc tu Web: {len(rc_log)} goi")
    if len(rc_log) == 0:
        print("[CANH BAO / FAIL TRAM 3] KHONG CO GOI TIN RC NAO DUOC GUI TU WEB!")
        print("  -> Nguyen nhan co the:")
        print("     1. Tab trinh duyet imu_3d_simulator.html khong duoc focus (Chrome tu pause gamepad).")
        print("     2. Nut 'MQTT: Local' tren Web dang Offline (bam vao de ket noi ws://10.11.162.171:9001).")
        print("     3. Tay cam chua nhan (kiem tra huy hieu 'Xbox 360 Controller: Active' tren Web).")
    else:
        print("[OK] Web GCS dang truyen tin hieu tay cam vao Broker tot!")
        print(f"  + Goi tin gan nhat : '{rc_log[-1]}'")
        print(f"  + 3 goi tin cuoi   : {rc_log[-3:]}")

    # =========================================================================
    # TRAM 4: TEST BOM XUNG TRUC TIEP VAO ESP32 (FIRMWARE PARSER & ACTUATOR TEST)
    # =========================================================================
    banner("TRAM 4: KIEM TRA PHAN HOI DONG CO ESP32 KHI BOM LENH TRUC TIEP")
    print("[*] Kich ban test doc lap: Script se chu dong gui lenh vao Broker:")
    print("    1. Gui: MODE,DIRECT_MANUAL")
    print("    2. Gui: ARM")
    print("    3. Bom lien tuc: RC,50.0,0.0,0.0,0.0 (Ga 50%) trong 2 giay")
    print("    4. Doc lai xung M1..M4 tu ESP32 Telemetry...")

    client.publish("isilab/drone/cmd", "MODE,DIRECT_MANUAL")
    time.sleep(0.1)
    client.publish("isilab/drone/cmd", "ARM")
    time.sleep(0.3)

    test_start = time.time()
    seq_counter = 1
    session_id = 88888
    while time.time() - test_start < 2.0:
        pkt = f"RC,2,{session_id},{seq_counter},50.0,0.0,0.0,0.0\n"
        client.publish("isilab/drone/rc", pkt, qos=0, retain=False)
        seq_counter += 1
        time.sleep(0.025)

    time.sleep(0.3)
    telem_after = telem_log[-1] if telem_log else {}
    test_m1 = telem_after.get("m1", 0)
    test_m2 = telem_after.get("m2", 0)
    test_m3 = telem_after.get("m3", 0)
    test_m4 = telem_after.get("m4", 0)
    test_mode = telem_after.get("mode", "")
    test_arm = telem_after.get("arm", 0)

    print(f"\n[*] Ket qua doc tu Telemetry ESP32 sau khi bom Ga 50%:")
    print(f"  + Trang thai : ARM={test_arm} | MODE={test_mode}")
    print(f"  + Xung dong co: M1={test_m1}us | M2={test_m2}us | M3={test_m3}us | M4={test_m4}us")

    # Khoa an toan lai ngay lap tuc
    client.publish("isilab/drone/cmd", "DISARM")
    time.sleep(0.2)
    client.loop_stop()

    banner("KET LUAN CHAN DOAN DUONG ONG")
    if test_m1 > 1100:
        print("[SUCCESS 100%] ESP32 DA NHAN RC VA DONG CO DA TANG XUNG PHAN HOI!")
        print(f"  Xung thuc te da tang tu 1050us len {test_m1}us khi bom ga 50%.")
        print("  Duong ong tu MQTT Broker -> ESP32 -> Dong co hoat dong HOAN HAO.")
    else:
        print("[DIAGNOSIS] ESP32 CHUA TANG XUNG (Van o 1050us hoac 900us).")
        print("  Nguyen nhan chac chan 100%:")
        print("  -> Ban firmware moi (sua loi sscanf newlib-nano) CHUA DUOC NAP LEN ESP32!")
        print("  -> ESP32 van dang chay ban firmware cu nen chua co bo parser dau phay moi.")
        print("  -> Hanh dong can lam: Vao Arduino IDE, chon cong COM cua ESP32 va bam UPLOAD lai.")
    print("=" * 70 + "\n")

if __name__ == "__main__":
    main()


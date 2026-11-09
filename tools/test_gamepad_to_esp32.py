#!/usr/bin/env python3
"""
===============================================================================
🎮 ISILAB UAV - XBOX 360 GAMEPAD DIRECT TEST PIPELINE (PC -> MQTT -> ESP32)
===============================================================================
Kịch bản test nhỏ:
1. Đọc trực tiếp cần gạt analog và nút bấm từ tay cầm Xbox 360 cắm máy tính (Pygame).
2. Đóng gói chuẩn Protocol V2: RC,2,<sessionId>,<seq>,<throttle>,<roll>,<pitch>,<yaw>
3. Publish tốc độ 40 Hz qua MQTT Broker cục bộ (127.0.0.1:1883 / isilab/drone/rc).
4. ESP32 nhận qua Wi-Fi và in ra Serial Monitor Arduino IDE:
   ">> [RC_STICK] Thr: XX% | Roll: XX% | Pitch: XX% | Yaw: XX% | ARMED | PWM:[...]us"
===============================================================================
"""

import sys
import time
import os
import random

# Thiết lập driver không cần mở cửa sổ đồ họa
os.environ["SDL_VIDEODRIVER"] = "dummy"

try:
    import pygame
except ImportError:
    print("[ERROR] Thư viện 'pygame' chưa được cài đặt. Hãy chạy: pip install pygame")
    sys.exit(1)

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("[ERROR] Thư viện 'paho-mqtt' chưa được cài đặt. Hãy chạy: pip install paho-mqtt")
    sys.exit(1)

# Cấu hình Broker và Topic
BROKER_HOST = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
BROKER_PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 1883
TOPIC_RC = "isilab/drone/rc"
TOPIC_CMD = "isilab/drone/cmd"

def apply_deadband(val, threshold=3.0):
    return 0.0 if abs(val) < threshold else val

def make_bar(val, min_v=-100.0, max_v=100.0, width=16):
    norm = (val - min_v) / (max_v - min_v)
    norm = max(0.0, min(1.0, norm))
    pos = int(norm * width)
    chars = ["-"] * width
    center = width // 2
    if min_v < 0:
        chars[center] = "|"
        if pos < center:
            for i in range(pos, center):
                chars[i] = "<"
        elif pos > center:
            for i in range(center + 1, min(pos + 1, width)):
                chars[i] = ">"
    else:
        for i in range(pos):
            chars[i] = "#"
    return "".join(chars)

def main():
    print("=" * 75)
    print("🎮 ISILAB UAV - XBOX GAMEPAD TO ESP32 DIRECT TEST PIPELINE")
    print("=" * 75)
    print(f"[*] Ket noi MQTT Broker tai: {BROKER_HOST}:{BROKER_PORT} ...")

    client = mqtt.Client(client_id=f"xbox_tester_{random.randint(1000, 9999)}")
    connected = [False]

    def on_connect(c, userdata, flags, rc):
        if rc == 0:
            connected[0] = True
            print(f"[OK] Da ket noi MQTT Broker thanh cong! Topic: '{TOPIC_RC}'")
        else:
            print(f"[FAIL] Loi ket noi MQTT Broker, ma loi: {rc}")

    client.on_connect = on_connect

    try:
        client.connect(BROKER_HOST, BROKER_PORT, keepalive=30)
        client.loop_start()
    except Exception as e:
        print(f"[ERROR] Khong the ket noi toi Broker {BROKER_HOST}:{BROKER_PORT}: {e}")
        print(">> Goi y: Hay chac chan Mosquitto Broker dang chay (chay tools/start_mqtt_broker.bat)")
        sys.exit(1)

    # Chờ kết nối MQTT tối đa 2 giây
    t_wait = time.time()
    while not connected[0] and (time.time() - t_wait < 2.0):
        time.sleep(0.05)

    # Khởi tạo Pygame Joystick
    pygame.init()
    pygame.joystick.init()
    num_joysticks = pygame.joystick.get_count()

    if num_joysticks == 0:
        print("\n[!] CANH BAO: Khong tim thay tay cam nao duoc cam vao may tinh!")
        print(">> Vui long cam tay cam Xbox 360 / USB Gamepad vao may va chay lai script.")
        print(">> Chuyen sang che do SIMULATION (Tao du lieu test tu dong de kiem tra ESP32)...")
        joystick = None
    else:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"[OK] Phat hien tay cam: '{joystick.get_name()}' ({joystick.get_numaxes()} truc, {joystick.get_numbuttons()} nut)")

    session_id = random.randint(0x10000000, 0x7FFFFFFF)
    seq = 0
    armed = False
    last_arm_btn = 0
    last_disarm_btn = 0

    print("\n" + "=" * 75)
    print("HUONG DAN DIEU KHIEN TAY CAM:")
    print(" - Can trai len/xuong : Ga (Throttle 0..100%)")
    print(" - Can trai trai/phai : Huong dau (Yaw -100..+100%)")
    print(" - Can phai trai/phai : Nghieng ngang (Roll -100..+100%)")
    print(" - Can phai len/xuong : Chuoi/Ngua (Pitch -100..+100%)")
    print(" - Nut [A] (Nut 0)     : MO KHOA (ARM) dong co")
    print(" - Nut [B] (Nut 1)     : KHOA KHAN CAP (DISARM)")
    print(" - Nhan Ctrl+C de dung.")
    print("=" * 75)
    print(">> MO ARDUINO IDE SERIAL MONITOR (115200) DE XEM DU LIEU NHAY TRUC TIEP <<\n")

    sim_angle = 0.0

    try:
        while True:
            t0 = time.time()
            pygame.event.pump()

            thr = 0.0
            roll = 0.0
            pitch = 0.0
            yaw = 0.0

            if joystick is not None:
                num_axes = joystick.get_numaxes()
                # Truc 1: Can trai Y (Throttle) -> Day len = -1.0, keo xuong = +1.0
                if num_axes > 1:
                    raw_ly = joystick.get_axis(1)
                    # Quy doi: Keo het xuong = 0%, Giua = 50%, Day het len = 100%
                    thr = (-raw_ly + 1.0) * 50.0
                    thr = max(0.0, min(100.0, thr))

                # Truc 0: Can trai X (Yaw) -> Trai = -1.0, Phai = +1.0
                if num_axes > 0:
                    raw_lx = joystick.get_axis(0)
                    yaw = apply_deadband(raw_lx * 100.0)

                # Truc 2 hoặc 3: Can phai X (Roll)
                # Tren Windows SDL2: Axis 2 = Right X, Axis 3 = Right Y (hoac Axis 3, 4)
                if num_axes >= 4:
                    raw_rx = joystick.get_axis(2) if num_axes == 4 else joystick.get_axis(2)
                    raw_ry = joystick.get_axis(3)
                    # Neu raw_rx luon bang 0 ma axis 3/4 chuyen dong thi hoán đổi
                    roll = apply_deadband(raw_rx * 100.0)
                    pitch = apply_deadband(-raw_ry * 100.0)  # Day len = Pitch duong (+)

                # Kiem tra nut Bam
                num_btns = joystick.get_numbuttons()
                if num_btns > 0:
                    btn_a = joystick.get_button(0) # Nut A
                    if btn_a and not last_arm_btn:
                        armed = True
                        client.publish(TOPIC_RC, "ARM\n", qos=0, retain=False)
                        client.publish(TOPIC_CMD, "ARM\n", qos=0, retain=False)
                    last_arm_btn = btn_a

                if num_btns > 1:
                    btn_b = joystick.get_button(1) # Nut B
                    if btn_b and not last_disarm_btn:
                        armed = False
                        client.publish(TOPIC_RC, "DISARM\n", qos=0, retain=False)
                        client.publish(TOPIC_CMD, "DISARM\n", qos=0, retain=False)
                    last_disarm_btn = btn_b

            else:
                # Che do mo phong khi chua cam tay cam
                sim_angle += 0.05
                import math
                thr = 50.0 + 30.0 * math.sin(sim_angle)
                roll = 20.0 * math.cos(sim_angle)
                pitch = 15.0 * math.sin(sim_angle * 0.7)
                yaw = 0.0
                armed = True

            # Tang Sequence 16-bit
            seq = (seq + 1) & 0xFFFF

            # Dong goi Protocol V2: RC,2,<session_id>,<seq>,<throttle>,<roll>,<pitch>,<yaw>\n
            packet = f"RC,2,{session_id:08X},{seq},{thr:.1f},{roll:.1f},{pitch:.1f},{yaw:.1f}\n"

            # Publish khong retain, QoS 0 (Realt-time Low Latency)
            client.publish(TOPIC_RC, packet, qos=0, retain=False)

            # Ve thanh tien do truc quan truc tiep tren terminal
            t_bar = make_bar(thr, 0.0, 100.0, 10)
            r_bar = make_bar(roll, -100.0, 100.0, 10)
            p_bar = make_bar(pitch, -100.0, 100.0, 10)
            y_bar = make_bar(yaw, -100.0, 100.0, 10)
            arm_str = "ARMED" if armed else "DISARM"

            sys.stdout.write(
                f"\r[PC TX #{seq:04d}] T:[{t_bar}]{thr:5.1f}% | R:[{r_bar}]{roll:+5.1f}% | "
                f"P:[{p_bar}]{pitch:+5.1f}% | Y:[{y_bar}]{yaw:+5.1f}% | [{arm_str}] (40Hz)   "
            )
            sys.stdout.flush()

            # Giu chu ky 40 Hz (25 ms)
            elapsed = time.time() - t0
            sleep_time = 0.025 - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n\n[*] Da dung kịch bản test theo yeu cau cua nguoi dung.")
    finally:
        client.loop_stop()
        client.disconnect()
        pygame.quit()
        print("[*] Da dong ket noi an toan.")

if __name__ == "__main__":
    main()

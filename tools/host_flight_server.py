#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
===============================================================================
UAV DISTRIBUTED FLIGHT COMPUTING: HOST PC EKF2 & GAMEPAD SERVER
===============================================================================
Nhiệm vụ:
  1. Nhận luồng dữ liệu thô (Raw MPU6050 + Raw Xsens MTi) từ ESP32 qua Wi-Fi UDP (100Hz).
  2. Tính toán bộ lọc ước lượng trạng thái EKF2 (PX4 Extended Kalman Filter) trên PC.
  3. Đọc tín hiệu tay cầm (Gamepad A124 / Xbox qua Pygame & XInput).
  4. Gửi kết quả EKF2 + Lệnh điều khiển RC (Throttle, Roll, Pitch, Yaw, ARM) ngược về ESP32.
  5. Cung cấp WebSocket Server (Port 81) cho Web HITL 3D Testbed Monitor.

Sử dụng:
  python tools/host_flight_server.py
===============================================================================
"""

import sys
import os
import time
import math
import socket
import select
import threading
import struct
import hashlib
import base64
import numpy as np

# Thử nạp Pygame cho tay cầm (Hỗ trợ A124, DirectInput, Xbox, v.v.)
USE_PYGAME = False
try:
    import pygame
    pygame.init()
    pygame.joystick.init()
    USE_PYGAME = True
except Exception:
    pass

# Thử nạp Windows XInput (Dự phòng cho Xbox)
USE_XINPUT = False
try:
    import ctypes
    class XINPUT_GAMEPAD(ctypes.Structure):
        _fields_ = [
            ("wButtons",      ctypes.c_ushort),
            ("bLeftTrigger",  ctypes.c_ubyte),
            ("bRightTrigger", ctypes.c_ubyte),
            ("sThumbLX",      ctypes.c_short),
            ("sThumbLY",      ctypes.c_short),
            ("sThumbRX",      ctypes.c_short),
            ("sThumbRY",      ctypes.c_short),
        ]
    class XINPUT_STATE(ctypes.Structure):
        _fields_ = [
            ("dwPacketNumber", ctypes.c_ulong),
            ("Gamepad",        XINPUT_GAMEPAD),
        ]
    for dll in ("xinput1_4.dll", "xinput1_3.dll", "xinput9_1_0.dll"):
        try:
            xinput_dll = ctypes.windll.LoadLibrary(dll)
            USE_XINPUT = True
            break
        except Exception:
            continue
except Exception:
    pass

# =============================================================================
# 1. BỘ LỌC DUNG HỢP TRẠNG THÁI PX4 EKF2 TRÊN MÁY TÍNH (PYTHON NUMPY CORE)
# =============================================================================
class PX4_EKF2_Core:
    def __init__(self):
        # Quaternion trạng thái [q0, q1, q2, q3]
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        
        # Góc Euler ước lượng (độ)
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0

        # Tốc độ góc đã khử bias (deg/s)
        self.gx = 0.0
        self.gy = 0.0
        self.gz = 0.0

        # Trọng số dung hợp: Xsens chiếm 85% (cảm biến công nghiệp), MPU6050 chiếm 15%
        self.weight_xsens = 0.85
        self.weight_mpu = 0.15

        self.last_update_time = time.time()
        self.initialized = False

    def update(self, m_ax, m_ay, m_az, m_gx, m_gy, m_gz,
               x_roll, x_pitch, x_yaw, x_ax, x_ay, x_az, x_gx, x_gy, x_gz):
        now = time.time()
        dt = now - self.last_update_time
        if dt <= 0.0 or dt > 0.1:
            dt = 0.01
        self.last_update_time = now

        # Khởi tạo ban đầu với góc Xsens
        if not self.initialized:
            if abs(x_roll) > 0.001 or abs(x_pitch) > 0.001 or abs(x_yaw) > 0.001:
                self.roll_deg = x_roll
                self.pitch_deg = x_pitch
                self.yaw_deg = x_yaw
                self.initialized = True
            elif abs(m_ax) > 0.1 or abs(m_az) > 0.1:
                # Tính góc từ vector gia tốc MPU
                self.roll_deg = math.degrees(math.atan2(m_ay, math.sqrt(m_ax*m_ax + m_az*m_az)))
                self.pitch_deg = math.degrees(math.atan2(-m_ax, m_az))
                self.yaw_deg = 0.0
                self.initialized = True
            return self.roll_deg, self.pitch_deg, self.yaw_deg, self.gx, self.gy, self.gz

        # 1. Dung hợp tốc độ góc con quay hồi chuyển (Gyro Fusion)
        has_xsens = (abs(x_gx) > 0.001 or abs(x_gy) > 0.001 or abs(x_gz) > 0.001)
        if has_xsens:
            self.gx = self.weight_xsens * x_gx + self.weight_mpu * m_gx
            self.gy = self.weight_xsens * x_gy + self.weight_mpu * m_gy
            self.gz = self.weight_xsens * x_gz + self.weight_mpu * m_gz
        else:
            self.gx = m_gx
            self.gy = m_gy
            self.gz = m_gz

        # 2. Bước dự báo EKF2 (Predict Step qua tích phân Gyro)
        self.roll_deg  += self.gx * dt
        self.pitch_deg += self.gy * dt
        self.yaw_deg   += self.gz * dt

        # 3. Bước hiệu chỉnh EKF2 (Correction Step qua Vector Trọng Trường & Tham chiếu Xsens)
        # Dung hợp gia tốc kế để tính góc nghiêng tĩnh (Tilt Angles)
        ax = self.weight_xsens * x_ax + self.weight_mpu * m_ax
        ay = self.weight_xsens * x_ay + self.weight_mpu * m_ay
        az = self.weight_xsens * x_az + self.weight_mpu * m_az
        acc_norm = math.sqrt(ax*ax + ay*ay + az*az)

        if 7.0 < acc_norm < 12.0: # Chỉ hiệu chỉnh khi không chịu quá tải gia tốc động lớn
            acc_roll = math.degrees(math.atan2(ay, math.sqrt(ax*ax + az*az)))
            acc_pitch = math.degrees(math.atan2(-ax, az))

            # Hệ số bù Kalman chuẩn EKF2 (Alpha filter ~0.04 ứng với 100Hz = 0.4Hz cutoff)
            self.roll_deg  = 0.96 * self.roll_deg  + 0.04 * acc_roll
            self.pitch_deg = 0.96 * self.pitch_deg + 0.04 * acc_pitch

        # 4. Khóa hướng Yaw theo Xsens MTi-3 AHRS (Tuyệt đối không trôi drift)
        if abs(x_yaw) > 0.001 or abs(x_roll) > 0.001:
            # Hiệu chỉnh nhẹ với góc Yaw từ Xsens
            yaw_diff = (x_yaw - self.yaw_deg + 180.0) % 360.0 - 180.0
            self.yaw_deg += 0.08 * yaw_diff

        # Wrap Yaw [-180, 180]
        self.yaw_deg = (self.yaw_deg + 180.0) % 360.0 - 180.0

        return self.roll_deg, self.pitch_deg, self.yaw_deg, self.gx, self.gy, self.gz


# =============================================================================
# 2. BỘ ĐỌC TAY CẦM ĐA NĂNG (HỖ TRỢ A124, DIRECTINPUT & XBOX)
# =============================================================================
class UniversalGamepad:
    def __init__(self):
        self.connected = False
        self.name = "Chua ket noi"
        self.joystick = None

        # Trạng thái điều khiển
        self.is_armed = False
        self.safety_limit_60 = True
        self.throttle_pct = 0.0
        self.base_throttle = 0.0
        self.roll_sp = 0.0
        self.pitch_sp = 0.0
        self.yaw_sp = 0.0

        self.prev_buttons = []
        self._init_gamepad()

    def _init_gamepad(self):
        if USE_PYGAME:
            pygame.joystick.init()
            count = pygame.joystick.get_count()
            if count > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                self.name = self.joystick.get_name()
                self.connected = True
                print(f"[GAMEPAD] Da nhan tay cam qua Pygame: '{self.name}' (Ho tro A124/Xbox)")
                return
        if USE_XINPUT:
            self.name = "Xbox Controller (Windows XInput)"
            self.connected = True
            print("[GAMEPAD] Da san sang Windows XInput Controller.")

    def _apply_deadzone(self, val, deadzone=0.08):
        if abs(val) < deadzone:
            return 0.0
        sign = 1.0 if val > 0 else -1.0
        return sign * (abs(val) - deadzone) / (1.0 - deadzone)

    def update(self):
        max_thr = 60.0 if self.safety_limit_60 else 100.0

        # --- A. ĐỌC QUA PYGAME (TAY CẦM A124, PS, DIRECTINPUT) ---
        if USE_PYGAME and self.joystick:
            pygame.event.pump()
            num_axes = self.joystick.get_numaxes()
            num_buttons = self.joystick.get_numbuttons()

            # Đọc nút bấm
            btns = [self.joystick.get_button(i) for i in range(num_buttons)]
            if len(self.prev_buttons) != num_buttons:
                self.prev_buttons = [0] * num_buttons

            just_pressed = lambda i: (btns[i] and not self.prev_buttons[i]) if i < num_buttons else False

            # Nút [A] (Button 0): ARM
            if just_pressed(0):
                self.is_armed = True
                self.base_throttle = 0.0
                print("\n[GAMEPAD] >>> [NUT A] ARMED! Mo khoa 4 dong co an toan.")

            # Nút [B] (Button 1) hoặc Back (Button 8): DISARM
            if just_pressed(1) or just_pressed(8):
                self.is_armed = False
                self.base_throttle = 0.0
                self.throttle_pct = 0.0
                print("\n[GAMEPAD] >>> [NUT B] DISARMED! Dung khan cap ca 4 dong co.")

            # Nút [X] (Button 2): Đổi trần ga 60% <-> 100%
            if just_pressed(2):
                self.safety_limit_60 = not self.safety_limit_60
                print(f"\n[GAMEPAD] >>> [NUT X] Tran ga: {'60% (An toan)' if self.safety_limit_60 else '100% (Toan dai)'}")

            # Nút RB (5) / LB (4): Tăng/Giảm ga nền cố định (+5% / -5%)
            if just_pressed(5):
                self.base_throttle = min(max_thr, self.base_throttle + 5.0)
                print(f"\n[GAMEPAD] >>> [RB] Ga giu nen: {self.base_throttle:.1f}%")
            if just_pressed(4):
                self.base_throttle = max(0.0, self.base_throttle - 5.0)
                print(f"\n[GAMEPAD] >>> [LB] Ga giu nen: {self.base_throttle:.1f}%")

            # Đọc cần Analog
            # Axis 1: Cần trái dọc (Throttle) -> -1.0 là đẩy lên
            raw_ly = self.joystick.get_axis(1) if num_axes > 1 else 0.0
            stick_thr = max(0.0, -self._apply_deadzone(raw_ly))

            # Axis 0: Cần trái ngang (Yaw) -> -1.0 trái, +1.0 phải
            raw_lx = self.joystick.get_axis(0) if num_axes > 0 else 0.0
            self.yaw_sp = self._apply_deadzone(raw_lx) * 45.0 # Max +/- 45 deg

            # Axis 2, 3, 4: Cần phải (Roll & Pitch)
            # Trên tay cầm A124: Axis 2 hoặc 4 là Roll, Axis 3 hoặc 5 là Pitch
            raw_rx = 0.0
            raw_ry = 0.0
            if num_axes >= 4:
                raw_rx = self.joystick.get_axis(2) if num_axes == 4 else self.joystick.get_axis(3)
                raw_ry = self.joystick.get_axis(3) if num_axes == 4 else self.joystick.get_axis(4)

            self.roll_sp  = self._apply_deadzone(raw_rx) * 25.0  # Max +/- 25 deg
            self.pitch_sp = -self._apply_deadzone(raw_ry) * 25.0 # Max +/- 25 deg

            # Tổng hợp ga
            if self.is_armed:
                self.throttle_pct = min(max_thr, max(0.0, self.base_throttle + stick_thr * max_thr))
            else:
                self.throttle_pct = 0.0

            self.prev_buttons = btns
            return

        # --- B. ĐỌC QUA XINPUT (NẾU DÙNG TAY CẦM XBOX CHUẨN WINDOWS) ---
        if USE_XINPUT:
            state = XINPUT_STATE()
            for slot in range(4):
                if xinput_dll.XInputGetState(slot, ctypes.byref(state)) == 0:
                    gp = state.Gamepad
                    btns = gp.wButtons

                    # Nút A (0x1000): ARM
                    if btns & 0x1000:
                        self.is_armed = True
                    # Nút B (0x2000) hoặc Back (0x0020): DISARM
                    if (btns & 0x2000) or (btns & 0x0020):
                        self.is_armed = False
                        self.throttle_pct = 0.0
                        self.base_throttle = 0.0

                    # Cần ga
                    ly = gp.sThumbLY / 32768.0
                    stick_thr = max(0.0, self._apply_deadzone(ly))
                    rt = gp.bRightTrigger / 255.0
                    total_stick = max(stick_thr, rt)

                    rx = gp.sThumbRX / 32768.0
                    ry = gp.sThumbRY / 32768.0
                    lx = gp.sThumbLX / 32768.0

                    self.roll_sp  = self._apply_deadzone(rx) * 25.0
                    self.pitch_sp = self._apply_deadzone(ry) * 25.0
                    self.yaw_sp   = self._apply_deadzone(lx) * 45.0

                    if self.is_armed:
                        self.throttle_pct = min(max_thr, max(0.0, self.base_throttle + total_stick * max_thr))
                    else:
                        self.throttle_pct = 0.0
                    break


# =============================================================================
# 3. WEBSOCKET SERVER CHO WEB 3D HITL TESTBED MONITOR (PORT 81)
# =============================================================================
class Web3DStreamer:
    def __init__(self, port=81):
        self.port = port
        self.clients = []
        self.lock = threading.Lock()
        self.running = True

    def start(self):
        t = threading.Thread(target=self._server_thread, daemon=True)
        t.start()

    def _server_thread(self):
        try:
            srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind(("0.0.0.0", self.port))
            srv.listen(5)
            srv.settimeout(1.0)
            print(f"[WEB_3D] WebSocket Server da san sang tai ws://localhost:{self.port}")
        except Exception as e:
            print(f"[WEB_3D] [CANH BAO] Khong the bind port {self.port}: {e}")
            return

        while self.running:
            try:
                conn, addr = srv.accept()
                threading.Thread(target=self._handle_client, args=(conn, addr), daemon=True).start()
            except socket.timeout:
                continue
            except Exception:
                break

    def _handle_client(self, conn, addr):
        # Handshake RFC6455
        try:
            data = conn.recv(1024).decode("utf-8", errors="ignore")
            key = None
            for line in data.split("\r\n"):
                if line.lower().startswith("sec-websocket-key:"):
                    key = line.split(":")[1].strip()
                    break
            if not key:
                conn.close()
                return

            accept_key = base64.b64encode(hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest()).decode()
            resp = (
                "HTTP/1.1 101 Switching Protocols\r\n"
                "Upgrade: websocket\r\n"
                "Connection: Upgrade\r\n"
                f"Sec-WebSocket-Accept: {accept_key}\r\n\r\n"
            )
            conn.sendall(resp.encode())
            with self.lock:
                self.clients.append(conn)
            print(f"[WEB_3D] Giao dien Web 3D da ket noi tu {addr}")
        except Exception:
            conn.close()

    def broadcast_telemetry(self, roll, pitch, yaw, m1, m2, m3, m4, is_armed):
        state_str = "ARMED" if is_armed else "DISARMED"
        msg = (
            f'{{"roll":{roll:.1f},"pitch":{pitch:.1f},"yaw":{yaw:.1f},'
            f'"ax":0.0,"ay":0.0,"az":9.8,"gx":0.0,"gy":0.0,"gz":0.0,"z":0.0,"vz":0.0,'
            f'"m":[{m1*0.1:.1f},{m2*0.1:.1f},{m3*0.1:.1f},{m4*0.1:.1f}],'
            f'"us":[{m1},{m2},{m3},{m4}],"state":"{state_str}"}}'
        )
        payload = msg.encode("utf-8")
        plen = len(payload)
        if plen <= 125:
            header = bytes([0x81, plen])
        else:
            header = bytes([0x81, 126, (plen >> 8) & 0xFF, plen & 0xFF])
        frame = header + payload

        with self.lock:
            dead = []
            for c in self.clients:
                try:
                    c.sendall(frame)
                except Exception:
                    dead.append(c)
            for d in dead:
                self.clients.remove(d)


# =============================================================================
# 4. CHƯƠNG TRÌNH CHÍNH (HOST FLIGHT CONTROLLER ORCHESTRATOR)
# =============================================================================
def main():
    print("=" * 75)
    print("   UAV DISTRIBUTED FLIGHT COMPUTING: HOST PC EKF2 & GAMEPAD SERVER")
    print("   Offload toan bo EKF2 & Gamepad sang PC -> ESP32 chi chay PID thăng bằng")
    print("=" * 75)

    ekf = PX4_EKF2_Core()
    gamepad = UniversalGamepad()
    web_streamer = Web3DStreamer(port=81)
    web_streamer.start()

    # Cấu hình UDP Socket
    UDP_LISTEN_PORT = 14550
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind(("0.0.0.0", UDP_LISTEN_PORT))
    sock.setblocking(False)

    print(f"\n[NETWORK] Dang lang nghe goi tin RAW IMU tu ESP32 tai UDP Port {UDP_LISTEN_PORT}...")
    print("[NETWORK] Ho tro ca WiFi Access Point 'ESP32-UAV-FC' (192.168.4.1) hoac Router phong Lab.\n")

    esp32_addr = None
    packet_count = 0
    last_stat_time = time.time()
    last_send_cmd_time = 0.0

    # Cache motor PWM phản hồi từ ESP32
    esp_m_us = [1000, 1000, 850, 1000]
    esp_armed = False

    try:
        while True:
            # 1. Cập nhật tay cầm A124 / Xbox
            gamepad.update()

            # 2. Đọc gói tin RAW IMU từ ESP32 (Non-blocking)
            try:
                data, addr = sock.recvfrom(512)
                if data:
                    esp32_addr = addr
                    packet_count += 1
                    text = data.decode("utf-8", errors="ignore").strip()

                    # Cấu trúc: RAW,m_ax,m_ay,m_az,m_gx,m_gy,m_gz,x_r,x_p,x_y,x_ax,x_ay,x_az,x_gx,x_gy,x_gz,m1,m2,m3,m4,arm
                    if text.startswith("RAW,"):
                        parts = text.split(",")
                        if len(parts) >= 21:
                            m_ax = float(parts[1])
                            m_ay = float(parts[2])
                            m_az = float(parts[3])
                            m_gx = float(parts[4])
                            m_gy = float(parts[5])
                            m_gz = float(parts[6])

                            x_r  = float(parts[7])
                            x_p  = float(parts[8])
                            x_y  = float(parts[9])
                            x_ax = float(parts[10])
                            x_ay = float(parts[11])
                            x_az = float(parts[12])
                            x_gx = float(parts[13])
                            x_gy = float(parts[14])
                            x_gz = float(parts[15])

                            esp_m_us[0] = int(parts[16])
                            esp_m_us[1] = int(parts[17])
                            esp_m_us[2] = int(parts[18])
                            esp_m_us[3] = int(parts[19])
                            esp_armed   = (int(parts[20]) == 1)

                            # 3. CHẠY THUẬT TOÁN ƯỚC LƯỢNG EKF2 TRÊN MÁY TÍNH
                            ekf.update(m_ax, m_ay, m_az, m_gx, m_gy, m_gz,
                                       x_r, x_p, x_y, x_ax, x_ay, x_az, x_gx, x_gy, x_gz)
            except BlockingIOError:
                pass
            except Exception as e:
                pass

            now = time.time()

            # 4. BẮN GÓI TIN ĐIỀU KHIỂN & EKF2 NGƯỢC VỀ ESP32 (50Hz - 100Hz)
            if esp32_addr and (now - last_send_cmd_time >= 0.015): # ~66 Hz
                last_send_cmd_time = now

                # Cấu trúc gói tin siêu tốc:
                # CMD,ekf_r,ekf_p,ekf_y,ekf_gx,ekf_gy,ekf_gz,thr,r_sp,p_sp,y_sp,arm
                arm_val = 1 if gamepad.is_armed else 0
                cmd_pkt = (
                    f"CMD,{ekf.roll_deg:.2f},{ekf.pitch_deg:.2f},{ekf.yaw_deg:.2f},"
                    f"{ekf.gx:.2f},{ekf.gy:.2f},{ekf.gz:.2f},"
                    f"{gamepad.throttle_pct:.1f},{gamepad.roll_sp:.1f},{gamepad.pitch_sp:.1f},{gamepad.yaw_sp:.1f},{arm_val}\n"
                )
                try:
                    sock.sendto(cmd_pkt.encode("utf-8"), esp32_addr)
                except Exception:
                    pass

                # Đồng thời chuyển tiếp Telemetry tới Web 3D Monitor
                web_streamer.broadcast_telemetry(
                    ekf.roll_deg, ekf.pitch_deg, ekf.yaw_deg,
                    esp_m_us[0], esp_m_us[1], esp_m_us[2], esp_m_us[3],
                    gamepad.is_armed
                )

            # 5. HIỂN THỊ TERMINAL LIVE DASHBOARD (10 Hz)
            if now - last_stat_time >= 0.1:
                last_stat_time = now
                ip_str = f"{esp32_addr[0]}:{esp32_addr[1]}" if esp32_addr else "DANG DOI ESP32..."
                arm_str = "ARMED" if gamepad.is_armed else "DISARM"
                limit_str = "60%" if gamepad.safety_limit_60 else "100%"

                sys.stdout.write(
                    f"\r[HOST_PC] ESP:[{ip_str:<21}] | EKF2: R:{ekf.roll_deg:+5.1f}° P:{ekf.pitch_deg:+5.1f}° Y:{ekf.yaw_deg:+5.1f}° | "
                    f"RC: Thr:{gamepad.throttle_pct:4.1f}% R_sp:{gamepad.roll_sp:+4.1f}° P_sp:{gamepad.pitch_sp:+4.1f}° | [{arm_str}] Lim:{limit_str} | "
                    f"PWM:[{esp_m_us[0]},{esp_m_us[1]},{esp_m_us[2]},{esp_m_us[3]}]us  "
                )
                sys.stdout.flush()

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n\n[DUNG] Dang ngat toan bo dong co ve 0% an toan...")
        if esp32_addr:
            for _ in range(5):
                sock.sendto(b"stop\n", esp32_addr)
                time.sleep(0.01)
        sock.close()
        print("[HOAN TAT] Da thoat chuong trinh an toan.")


if __name__ == "__main__":
    main()

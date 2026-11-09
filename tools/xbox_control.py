#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
===============================================================================
PC XBOX CONTROLLER TO ESP32 USB SERIAL BRIDGE (Windows XInput + PySerial)
Kết nối: Tay cầm Xbox -> Máy tính (USB) -> Cáp USB Serial -> ESP32
Không cần cài đặt thư viện ngoài (Sử dụng XInput sẵn có của Windows + pyserial)
===============================================================================
SƠ ĐỒ NÚT BẤM TAY CẦM XBOX:
  - Nút [A]           : ARM (Mở khóa an toàn 4 ESC)
  - Nút [B] / [Back]  : DISARM / DỪNG KHẨN CẤP (Ngắt ngay lập tức về 0%)
  - Cò phải [RT]      : Bóp cò để tăng ga mượt (nhả cò tự giảm về ga nền)
  - Cần trái [LY > 0] : Đẩy cần trái lên để tăng ga
  - Nút [RB] / [LB]   : Tăng / Giảm mức ga giữ cố định (+5% / -5%)
  - [D-Pad Lên]       : Chọn riêng Motor 1 (GPIO 7 - ESC Vàng)
  - [D-Pad Phải]      : Chọn riêng Motor 2 (GPIO 6 - ESC Vàng)
  - [D-Pad Xuống]     : Chọn riêng Motor 3 (GPIO 5 - ESC Đen 40A, Min=850us)
  - [D-Pad Trái]      : Chọn riêng Motor 4 (GPIO 4 - ESC Vàng)
  - Nút [Y]           : Chọn đồng loạt CẢ 4 ĐỘNG CƠ (ALL MOTORS)
  - Nút [Start/Menu]  : Bật/Tắt trộn nghiêng Quad-X (Roll/Pitch/Yaw bằng cần phải)
  - Nút [X]           : Đổi giới hạn trần ga an toàn (60% <-> 100%)
===============================================================================
"""

import ctypes
import sys
import time
import threading
import serial
import serial.tools.list_ports

# =============================================================================
# 1. CẤU TRÚC WINDOWS XINPUT (ĐỌC TRỰC TIẾP TAY CẦM XBOX TRÊN WINDOWS)
# =============================================================================
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

# Mã bit nút bấm chuẩn XInput
XINPUT_GAMEPAD_DPAD_UP        = 0x0001
XINPUT_GAMEPAD_DPAD_DOWN      = 0x0002
XINPUT_GAMEPAD_DPAD_LEFT      = 0x0004
XINPUT_GAMEPAD_DPAD_RIGHT     = 0x0008
XINPUT_GAMEPAD_START          = 0x0010
XINPUT_GAMEPAD_BACK           = 0x0020
XINPUT_GAMEPAD_LEFT_THUMB     = 0x0040
XINPUT_GAMEPAD_RIGHT_THUMB    = 0x0080
XINPUT_GAMEPAD_LEFT_SHOULDER  = 0x0100  # LB
XINPUT_GAMEPAD_RIGHT_SHOULDER = 0x0200  # RB
XINPUT_GAMEPAD_A              = 0x1000
XINPUT_GAMEPAD_B              = 0x2000
XINPUT_GAMEPAD_X              = 0x4000
XINPUT_GAMEPAD_Y              = 0x8000


def load_xinput():
    for dll_name in ("xinput1_4.dll", "xinput1_3.dll", "xinput9_1_0.dll"):
        try:
            return ctypes.windll.LoadLibrary(dll_name)
        except OSError:
            continue
    return None


def normalize_thumb(val, deadzone=4500):
    if abs(val) < deadzone:
        return 0.0
    if val > 0:
        return min(1.0, (val - deadzone) / float(32767 - deadzone))
    return max(-1.0, (val + deadzone) / float(32768 - deadzone))


# =============================================================================
# 2. TỰ ĐỘNG DÒ CỔNG COM CỦA ESP32 (LOẠI BỎ COM BLUETOOTH ẢO)
# =============================================================================
def find_esp32_serial_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None

    print("\nDanh sach cong COM tren may tinh:")
    espressif_port = None
    usb_serial_port = None

    for idx, p in enumerate(ports):
        desc = f"{p.device} - {p.description} [{p.hwid}]"
        print(f"  [{idx + 1}] {desc}")
        hwid_lower = p.hwid.lower()
        desc_lower = p.description.lower()

        if "bthenum" in hwid_lower or "bluetooth" in desc_lower:
            continue

        if "303a" in hwid_lower or "cp210" in desc_lower or "ch340" in desc_lower or "esp32" in desc_lower:
            espressif_port = p.device
        elif "usb" in hwid_lower or "usb" in desc_lower:
            usb_serial_port = p.device

    if espressif_port:
        return espressif_port
    if usb_serial_port:
        return usb_serial_port
    return ports[0].device


# =============================================================================
# 3. CHƯƠNG TRÌNH CHÍNH
# =============================================================================
def main():
    xinput = load_xinput()
    if not xinput:
        print("[LOI] Khong tim thay thu vien XInput tren Windows!")
        sys.exit(1)

    port_name = sys.argv[1] if len(sys.argv) > 1 else find_esp32_serial_port()
    if not port_name:
        print("[LOI] Khong tim thay cong COM nao! Hay cam cap USB cua ESP32 vao may tinh.")
        sys.exit(1)

    print(f"\n[SERIAL] Dang ket noi toi ESP32 tai cong {port_name} (115200 baud)...")
    print("(Luu y: Hay TẮT cửa sổ Serial Monitor của Arduino IDE để không bị chiếm cổng COM!)")
    try:
        ser = serial.Serial(port_name, 115200, timeout=0.05)
    except Exception as e:
        print(f"\n[LOI MO CONG {port_name}]: {e}")
        print("-> Hay dong Serial Monitor trong Arduino IDE hoac truyen ten cong: python tools/xbox_control.py COM32")
        sys.exit(1)

    stop_thread = False

    def serial_reader():
        ignore_prefixes = ("[PX4_EKF]", "[ESC_QUAD]", "[MPU6050]", "[Xsens", "[PX4_PID]", "---------")
        while not stop_thread:
            try:
                if ser.in_waiting > 0:
                    line = ser.readline().decode("utf-8", errors="ignore").strip()
                    if line and not line.startswith(ignore_prefixes):
                        print(f"\r[ESP32] {line:<75}")
                else:
                    time.sleep(0.01)
            except Exception:
                break

    reader_t = threading.Thread(target=serial_reader, daemon=True)
    reader_t.start()

    def send_cmd(cmd_str):
        try:
            ser.write((cmd_str + "\n").encode("utf-8"))
            ser.flush()
        except Exception as e:
            print(f"\n[LOI GUI SERIAL]: {e}")

    print("\n=====================================================================")
    print("  DA KET NOI ESP32! DANG QUET TAY CAM XBOX TREN MAY TINH...")
    print("  - [A]: ARM (Mo khoa)     | [B] hoac [Back]: DISARM (Dung khan cap)")
    print("  - [RT] / Can Trai [LY]   : Bop co / Day can len de tang ga")
    print("  - [RB] / [LB]            : Tang / Giam ga giu co dinh (+5% / -5%)")
    print("  - [D-Pad Len/Phai/Xuong/Trai]: Chon Motor 1 / 2 / 3 / 4")
    print("  - [Y]: Chon CA 4 MOTOR   | [Start]: Bat/Tat che do Mixer 4 kenh")
    print("  - [X]: Gioi han 60%/100% | Nhan Ctrl+C de thoat (Tu dong ngat motor)")
    print("=====================================================================\n")

    is_armed = False
    safety_60_pct = True
    mixer_mode = False
    selected_target = "all"
    base_throttle = 0.0
    prev_buttons = 0
    was_connected = False
    last_sent_cmd = ""
    last_send_time = 0.0

    try:
        while True:
            state = XINPUT_STATE()
            connected_slot = -1
            for slot in range(4):
                if xinput.XInputGetState(slot, ctypes.byref(state)) == 0:
                    connected_slot = slot
                    break

            if connected_slot == -1:
                if was_connected:
                    was_connected = False
                    is_armed = False
                    base_throttle = 0.0
                    send_cmd("stop")
                    print("\n[FAILSAFE] >>> Mat ket noi tay cam Xbox tren PC! Da ngat ca 4 dong co ve 0%!")
                time.sleep(0.1)
                continue

            if not was_connected:
                was_connected = True
                print(f"\n[XBOX] >>> Da nhan tay cam Xbox (Slot #{connected_slot})! Nhan nut [A] de ARM.")

            gp = state.Gamepad
            btns = gp.wButtons
            pressed = btns & (~prev_buttons)

            # 1. Nút [B] hoặc [Back]: DỪNG KHẨN CẤP (DISARM)
            if pressed & (XINPUT_GAMEPAD_B | XINPUT_GAMEPAD_BACK):
                is_armed = False
                base_throttle = 0.0
                last_sent_cmd = ""
                send_cmd("stop")
                print("\n[XBOX] [NUT B] >>> EMERGENCY STOP / DISARM! Ca 4 motor ve 0% (M1,2,4=1000us, M3=850us).")

            # 2. Nút [A]: ARM (Mở khóa an toàn)
            elif pressed & XINPUT_GAMEPAD_A:
                rt_val = gp.bRightTrigger / 255.0
                ly_val = normalize_thumb(gp.sThumbLY)
                if rt_val > 0.05 or ly_val > 0.10:
                    print("\n[XBOX] [CANH BAO] Hay nha co RT va can ga ve 0 truoc khi nhan [A] de ARM!")
                else:
                    is_armed = True
                    base_throttle = 0.0
                    send_cmd("arm")
                    print("\n[XBOX] [NUT A] >>> ARMED! Bop co [RT], day can trai [LY] hoac bam [RB]/[LB] de len ga.")

            # 3. Nút [X]: Đổi giới hạn ga 60% <-> 100%
            if pressed & XINPUT_GAMEPAD_X:
                safety_60_pct = not safety_60_pct
                send_cmd("limit")
                print(f"\n[XBOX] [NUT X] >>> Tran ga toi da: {'60% (An toan)' if safety_60_pct else '100% (Toan dai)'}")

            # 4. Nút [Start]: Bật/Tắt chế độ Quad-X Mixer
            if pressed & XINPUT_GAMEPAD_START:
                mixer_mode = not mixer_mode
                print(f"\n[XBOX] [START] >>> Che do dieu khien: {'QUAD-X MIXER (Throttle + Roll/Pitch/Yaw)' if mixer_mode else 'TEST MOTOR CONG DONG LOAT / DON LE'}")

            # 5. Chọn động cơ bằng D-Pad và nút [Y]
            if pressed & XINPUT_GAMEPAD_DPAD_UP:
                selected_target = "m1"
                base_throttle = 0.0
                mixer_mode = False
                send_cmd("m1")
            elif pressed & XINPUT_GAMEPAD_DPAD_RIGHT:
                selected_target = "m2"
                base_throttle = 0.0
                mixer_mode = False
                send_cmd("m2")
            elif pressed & XINPUT_GAMEPAD_DPAD_DOWN:
                selected_target = "m3"
                base_throttle = 0.0
                mixer_mode = False
                send_cmd("m3")
            elif pressed & XINPUT_GAMEPAD_DPAD_LEFT:
                selected_target = "m4"
                base_throttle = 0.0
                mixer_mode = False
                send_cmd("m4")
            elif pressed & XINPUT_GAMEPAD_Y:
                selected_target = "all"
                base_throttle = 0.0
                send_cmd("all")

            # 6. Tính toán và gửi lệnh Ga khi đang ARMED
            if is_armed:
                max_cap = 60.0 if safety_60_pct else 100.0

                if pressed & XINPUT_GAMEPAD_RIGHT_SHOULDER:
                    base_throttle = min(max_cap, base_throttle + 5.0)
                    print(f"\n[XBOX] [RB] Ga giu nen: {base_throttle:.1f}%")
                if pressed & XINPUT_GAMEPAD_LEFT_SHOULDER:
                    base_throttle = max(0.0, base_throttle - 5.0)
                    print(f"\n[XBOX] [LB] Ga giu nen: {base_throttle:.1f}%")

                rt_norm = gp.bRightTrigger / 255.0 if gp.bRightTrigger > 5 else 0.0
                ly_norm = max(0.0, normalize_thumb(gp.sThumbLY))
                stick_thr = max(rt_norm, ly_norm)

                total_thr = min(max_cap, max(0.0, base_throttle + stick_thr * max_cap))

                now = time.time()
                if mixer_mode:
                    roll  = normalize_thumb(gp.sThumbRX) * 15.0
                    pitch = normalize_thumb(gp.sThumbRY) * 15.0
                    yaw   = normalize_thumb(gp.sThumbLX) * 10.0

                    if total_thr < 2.0:
                        m1 = m2 = m3 = m4 = 0.0
                    else:
                        m1 = max(0.0, min(max_cap, total_thr - roll + pitch - yaw))
                        m2 = max(0.0, min(max_cap, total_thr + roll - pitch - yaw))
                        m3 = max(0.0, min(max_cap, total_thr + roll + pitch + yaw))
                        m4 = max(0.0, min(max_cap, total_thr - roll - pitch + yaw))

                    cmd = f"x {m1:.1f} {m2:.1f} {m3:.1f} {m4:.1f}"
                else:
                    cmd = f"throttle {total_thr:.1f}"

                # Gửi ngay khi lệnh thay đổi (tối đa 40Hz) HOẶC gửi Heartbeat mỗi 0.5s để nuôi Watchdog 3s trên ESP32
                state_changed = (cmd != last_sent_cmd) and (now - last_send_time >= 0.025 or total_thr == 0.0)
                heartbeat_due = (now - last_send_time >= 0.5)
                if state_changed or heartbeat_due:
                    send_cmd(cmd)
                    last_sent_cmd = cmd
                    last_send_time = now

            prev_buttons = btns
            time.sleep(0.015)

    except KeyboardInterrupt:
        print("\n[THOAT] Dang ngat toan bo dong co ve 0% an toan...")
        send_cmd("stop")
        time.sleep(0.1)
    finally:
        stop_thread = True
        ser.close()


if __name__ == "__main__":
    main()

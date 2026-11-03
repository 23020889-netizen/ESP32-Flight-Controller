"""
Xsens MTi-3 Serial/UART Data Reader (Xbus Protocol)
Created for VIO / Robotics Project at ISILAB
"""

import sys
import time
import struct
import argparse
import serial
import serial.tools.list_ports
from serial.serialutil import SerialException

# Enforce UTF-8 for console output on Windows
if sys.stdout.encoding != 'utf-8':
    try:
        sys.stdout.reconfigure(encoding='utf-8')
    except Exception:
        pass

def list_and_find_ports():
    """List all available COM ports and detect Xsens/FTDI devices."""
    ports = list(serial.tools.list_ports.comports())
    print("\n--- Danh sach cong COM he thong phat hien ---", flush=True)
    mti_port = None
    if not ports:
        print("  (Khong co cong COM nao dang ket noi)", flush=True)
        return None

    for p in ports:
        vid_pid = f"VID:{hex(p.vid)} PID:{hex(p.pid)}" if p.vid else "No VID/PID"
        desc = p.description or ""
        print(f"  * {p.device}: {desc} [{vid_pid}]", flush=True)
        if p.vid == 0x2639 or "xsens" in desc.lower() or "motion tracker" in desc.lower():
            mti_port = p.device

    if not mti_port and ports:
        mti_port = ports[0].device
    return mti_port

def verify_checksum(packet: bytes) -> bool:
    """Verify Xbus checksum: sum(BID ... CS) % 256 == 0"""
    return (sum(packet[1:]) & 0xFF) == 0

def parse_mtdata2(payload: bytes):
    """
    Parse MTData2 payload containing data identifiers.
    MTData2 format: [DataIdentifier (2B)] [DataLength (1B)] [Data (DataLength B)] ...
    """
    idx = 0
    parsed = {}
    while idx + 3 <= len(payload):
        data_id = (payload[idx] << 8) | payload[idx + 1]
        data_len = payload[idx + 2]
        idx += 3
        if idx + data_len > len(payload):
            break
        data = payload[idx : idx + data_len]
        idx += data_len

        # 0x2030: Euler Angles (Float32: Roll, Pitch, Yaw)
        if (data_id & 0xFFF0) == 0x2030 and len(data) >= 12:
            roll, pitch, yaw = struct.unpack(">fff", data[:12])
            parsed["euler"] = (roll, pitch, yaw)

        # 0x2010: Quaternion (Float32: q0, q1, q2, q3)
        elif (data_id & 0xFFF0) == 0x2010 and len(data) >= 16:
            q0, q1, q2, q3 = struct.unpack(">ffff", data[:16])
            parsed["quaternion"] = (q0, q1, q2, q3)

        # 0x4020 / 0x4030 / 0x4040: Acceleration / Accel (Float32: accX, accY, accZ in m/s^2)
        elif ((data_id & 0xFFF0) in (0x4020, 0x4030, 0x4040, 0x4010)) and len(data) >= 12:
            ax, ay, az = struct.unpack(">fff", data[:12])
            parsed["accel"] = (ax, ay, az)

        # 0x8020 / 0x8040: Rate of Turn / Gyro (Float32: gyrX, gyrY, gyrZ in rad/s)
        elif ((data_id & 0xFFF0) in (0x8020, 0x8040)) and len(data) >= 12:
            gx, gy, gz = struct.unpack(">fff", data[:12])
            parsed["gyro"] = (gx, gy, gz)

        # 0x1020: Packet Counter (Uint16)
        elif (data_id & 0xFFF0) == 0x1020 and len(data) >= 2:
            packet_counter = struct.unpack(">H", data[:2])[0]
            parsed["packet_counter"] = packet_counter

    return parsed

def read_mti(port_name, baudrate=115200):
    print(f"\n[INFO] Dang mo cong {port_name} voi Baudrate {baudrate}...", flush=True)
    try:
        ser = serial.Serial(port=port_name, baudrate=baudrate, timeout=1.0)
    except PermissionError:
        print(f"\n[CANH BAO] LOI ACCESS IS DENIED tren {port_name}!", flush=True)
        print("-> Nguyen nhan: Phan mem MT Manager hoac 1 tien trinh khac dang mo cong nay.", flush=True)
        print("-> Khac phuc: Hay dong MT Manager, hoac rut cap USB ra roi cam lai.", flush=True)
        return
    except SerialException as e:
        print(f"\n[LOI] Khong the mo cong {port_name}: {e}", flush=True)
        return

    print(f"[OK] Da ket noi {port_name}. Gui tin hieu kich hoat Measurement Mode...", flush=True)

    # Gui lenh GoToMeasurement (FA FF 10 00 F1) de cam bien bat dau truyen MTData2
    try:
        ser.write(bytes([0xFA, 0xFF, 0x10, 0x00, 0xF1]))
    except Exception:
        pass

    print("Dang lang nghe luong du lieu Xbus. Nhan Ctrl + C de dung.\n" + "=" * 70, flush=True)

    try:
        while True:
            # Tim Preamble byte 0xFA
            try:
                b = ser.read(1)
            except (SerialException, PermissionError) as e:
                print(f"\n[CANH BAO] Mat ket noi voi {port_name} ({e}). Vui long kiem tra cap ket noi hoac dong MT Manager.", flush=True)
                break

            if not b or b[0] != 0xFA:
                continue

            # Doc Header: BID (1B), MID (1B), LEN (1B)
            header = ser.read(3)
            if len(header) < 3:
                continue

            bid = header[0]
            mid = header[1]
            length = header[2]

            # Xu ly Extended Length neu length == 0xFF
            if length == 0xFF:
                ext_len = ser.read(2)
                if len(ext_len) < 2:
                    continue
                length = (ext_len[0] << 8) | ext_len[1]

            # Doc Payload + Checksum (1B)
            remaining = ser.read(length + 1)
            if len(remaining) < length + 1:
                continue

            full_packet = bytes([0xFA]) + header + remaining
            if not verify_checksum(full_packet):
                continue

            payload = remaining[:length]

            # 0x36 = MTData2 (Goi du lieu do luong)
            if mid == 0x36:
                data = parse_mtdata2(payload)
                out_str = []
                if "packet_counter" in data:
                    out_str.append(f"Seq:{data['packet_counter']:05d}")
                if "euler" in data:
                    r, p, y = data["euler"]
                    out_str.append(f"Roll:{r:6.2f} Ptc:{p:6.2f} Yaw:{y:6.2f}")
                if "accel" in data:
                    ax, ay, az = data["accel"]
                    out_str.append(f"Acc:[{ax:5.2f},{ay:5.2f},{az:5.2f}]")
                if "gyro" in data:
                    gx, gy, gz = data["gyro"]
                    out_str.append(f"Gyr:[{gx:5.2f},{gy:5.2f},{gz:5.2f}]")

                if out_str:
                    print(" | ".join(out_str), flush=True)
                else:
                    print(f"[MTData2] Goi tin raw ({len(payload)}B): {payload.hex()[:32]}...", flush=True)

    except KeyboardInterrupt:
        print("\n[INFO] Da dung chuong trinh.", flush=True)
    finally:
        ser.close()
        print("[INFO] Da dong cong serial.", flush=True)

def main():
    parser = argparse.ArgumentParser(description="Doc du lieu Xsens MTi-3 qua UART/COM port.")
    parser.add_argument("-p", "--port", type=str, default=None, help="Ten cong COM (vi du: COM7).")
    parser.add_argument("-b", "--baud", type=int, default=115200, help="Baud rate (mac dinh: 115200).")
    parser.add_argument("-l", "--list", action="store_true", help="Liet ke cac cong COM hien co.")
    args = parser.parse_args()

    if args.list:
        list_and_find_ports()
        return

    port = args.port
    if not port:
        port = list_and_find_ports()

    if not port:
        print("\n[CANH BAO] Khong tim thay cong COM hop le!", flush=True)
        print("Hay kiem tra Device Manager xem driver da duoc nhan chua.", flush=True)
        sys.exit(1)

    read_mti(port, args.baud)

if __name__ == "__main__":
    main()

"""
Xsens MTi-3 Real-Time VIO Telemetry Reader (XDA Engine)
Configured for ISILAB Visual-Inertial Odometry Research
"""

import sys
import time

# Enforce UTF-8 for console output on Windows
if sys.stdout.encoding != 'utf-8':
    try:
        sys.stdout.reconfigure(encoding='utf-8')
    except Exception:
        pass

try:
    import xsensdeviceapi as xda
except ImportError:
    print("[LOI] Khong tim thay thu vien xsensdeviceapi.", flush=True)
    sys.exit(1)

class XdaCallback(xda.XsCallback):
    def __init__(self):
        super().__init__()
        self.count = 0

    def onLiveDataAvailable(self, dev, packet):
        self.count += 1
        acc = packet.calibratedAcceleration() if packet.containsCalibratedAcceleration() else None
        gyr = packet.calibratedGyroscopeData() if packet.containsCalibratedGyroscopeData() else None
        euler = packet.orientationEuler() if packet.containsOrientation() else None

        # Hien thi tren console 10 lan moi giay (cu 10 packet tai 100Hz in 1 dong)
        if self.count % 10 == 0:
            out = [f"Pkt {self.count:05d}"]
            if euler is not None:
                out.append(f"Roll:{euler.x():6.2f}° Ptc:{euler.y():6.2f}° Yaw:{euler.z():6.2f}°")
            if acc is not None:
                out.append(f"Acc:[{acc[0]:5.2f},{acc[1]:5.2f},{acc[2]:5.2f}] m/s²")
            if gyr is not None:
                out.append(f"Gyr:[{gyr[0]:5.2f},{gyr[1]:5.2f},{gyr[2]:5.2f}] rad/s")

            print(" | ".join(out), flush=True)

def main():
    print("=" * 75, flush=True)
    print("  Xsens MTi-3 Live VIO Telemetry Reader (100Hz Telemetry Stream)")
    print("=" * 75, flush=True)

    print("[INFO] Khoi tao XsControl...", flush=True)
    control = xda.XsControl_construct()
    if control == 0:
        print("[LOI] Khong the khoi tao XsControl.", flush=True)
        return

    port_name = "COM7"
    baudrate = 115200

    try:
        print(f"[INFO] Dang quet cam bien MTi tren {port_name} (Baudrate: {baudrate})...", flush=True)
        port_info = xda.XsScanner_scanPort(port_name, baudrate)
        if port_info.empty():
            print(f"[CANH BAO] Khong tim thay cam bien tren {port_name}. Thu quet toan bo cong COM...", flush=True)
            port_array = xda.XsScanner_scanPorts()
            for i in range(port_array.size()):
                if port_array[i].deviceId().isMti() or port_array[i].deviceId().isMtig():
                    port_info = port_array[i]
                    port_name = port_info.portName()
                    baudrate = port_info.baudrate()
                    break

        if port_info.empty():
            print("[LOI] Khong tim thay cam bien MTi nao! Vui long kiem tra lai cap cam hoac dong MT Manager.", flush=True)
            return

        print(f"[OK] Phat hien MTi ID: {port_info.deviceId().toXsString()} tai {port_name}")

        print(f"[INFO] Dang mo cong {port_name}...", flush=True)
        if not control.openPort(port_name, baudrate):
            print(f"[LOI] Khong the mo {port_name}. Hay dong phan mem MT Manager neu dang mo!", flush=True)
            return

        device = control.device(port_info.deviceId())
        if device == 0:
            print("[LOI] Khong the ket noi doi tuong MTi.", flush=True)
            return

        print(f"[OK] Da ket noi thiet bi: {device.productCode()}", flush=True)

        # Chuyen sang Config Mode de cau hinh
        device.gotoConfig()

        # Xoa cac cau hinh dong bo SyncSettings cu (nhu SendLatest/Trigger pulse lam cam bien dung cho xung)
        device.setSyncSettings(xda.XsSyncSettingArray())

        # Thiet lap cac luong du lieu IMU 100Hz cho bai toan VIO
        print("[INFO] Cau hinh luong du lieu (VIO 100Hz: Quaternion, Gyroscope, Accelerometer)...", flush=True)
        cfg = xda.XsOutputConfigurationArray()
        cfg.push_back(xda.XsOutputConfiguration(xda.XDI_PacketCounter, 0))
        cfg.push_back(xda.XsOutputConfiguration(xda.XDI_SampleTimeFine, 0))
        cfg.push_back(xda.XsOutputConfiguration(xda.XDI_Quaternion, 100))
        cfg.push_back(xda.XsOutputConfiguration(xda.XDI_RateOfTurn, 100))
        cfg.push_back(xda.XsOutputConfiguration(xda.XDI_Acceleration, 100))
        device.setOutputConfiguration(cfg)

        # Dang ky Callback xu ly du lieu
        cb = XdaCallback()
        device.addCallbackHandler(cb)

        # Chuyen sang che do do luong
        print("[INFO] Kich hoat Measurement Mode...", flush=True)
        device.gotoMeasurement()

        print("\n" + "=" * 75, flush=True)
        print("  DANG STREAM DU LIEU REAL-TIME 100Hz (Nhan Ctrl + C de dung)")
        print("=" * 75, flush=True)

        while True:
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[INFO] Nguoi dung dung chuong trinh.", flush=True)
    finally:
        print("[INFO] Dang ngat ket noi va dong cong COM an toan...", flush=True)
        try:
            device.removeCallbackHandler(cb)
        except Exception:
            pass
        try:
            control.closePort(port_name)
            control.close()
        except Exception:
            pass
        print("[OK] Hoan tat.", flush=True)

if __name__ == "__main__":
    main()

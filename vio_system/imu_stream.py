"""
VIO IMU Stream Module - ISILAB VIO / UAV Project
Supports reading real-time inertial telemetry from ESP32-C6 over:
1. USB Serial (COM port, e.g. COM13)
2. Wireless UDP Broadcast (WiFi 6 Telemetry, e.g. 'udp://14550' or 'udp')
3. Synthetic / Fallback Simulation Mode (TDD & Offline testing)

Parses [PX4_EKF] packets & JSON payloads:
- Accel (m/s^2), Gyro (deg/s -> rad/s), Euler RPY (deg).
"""

import sys
import re
import time
import json
import socket
import threading
from collections import deque
from dataclasses import dataclass
from typing import Optional, List, Tuple
import numpy as np

if hasattr(sys.stdout, 'reconfigure'):
    try:
        sys.stdout.reconfigure(encoding='utf-8')
    except Exception:
        pass

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


@dataclass
class ImuSample:
    timestamp: float          # Thời gian ghi nhận (giây)
    accel: np.ndarray         # Gia tốc 3 trục [ax, ay, az] (m/s^2)
    gyro: np.ndarray          # Vận tốc góc 3 trục [gx, gy, gz] (rad/s)
    rpy_deg: np.ndarray       # Góc Euler [roll, pitch, yaw] (độ)
    tag: str = "PX4_EKF"      # Nguồn dữ liệu


class ImuStream:
    def __init__(self, port: Optional[str] = None, baudrate: int = 115200, max_buffer_size: int = 1000):
        self.port = port
        self.baudrate = baudrate
        self.max_buffer_size = max_buffer_size
        self.samples = deque(maxlen=max_buffer_size)
        self.lock = threading.Lock()
        
        self.is_running = False
        self.thread: Optional[threading.Thread] = None
        self.serial_conn: Optional[serial.Serial] = None
        self.udp_sock: Optional[socket.socket] = None
        self.is_udp = False
        self.is_connected = False
        self.is_simulated = False
        
        # Regex khớp định dạng: [PX4_EKF] R: -9.2° P: -9.1° Y: -80.3° | Acc:[ -0.11, -0.03,  0.95] | Gyr:[ -0.7,  0.2,  0.0]
        self.regex = re.compile(
            r'^\[(.*?)\]\s*R:\s*([-\d.]+)°?\s+P:\s*([-\d.]+)°?\s+Y:\s*([-\d.]+)°?\s*\|\s*'
            r'Acc:\[\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\]\s*\|\s*'
            r'Gyr:\[\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\]'
        )

    @staticmethod
    def find_available_port() -> Optional[str]:
        """Tự động dò tìm cổng COM của ESP32 hoặc thiết bị USB Serial."""
        if not SERIAL_AVAILABLE:
            return None
        ports = list(serial.tools.list_ports.comports())
        if not ports:
            return None
        
        # Lọc bỏ hoàn toàn các cổng Bluetooth ảo (luôn bị treo timeout nếu không kết nối)
        usb_ports = []
        for p in ports:
            desc = (p.description or "").lower()
            hwid = (p.hwid or "").lower()
            if "bluetooth" in desc or "bthenum" in hwid:
                continue
            usb_ports.append(p)

        if not usb_ports:
            return None

        # Ưu tiên các cổng có định danh ESP32, JTAG, CH340, CP210x, FTDI
        for p in usb_ports:
            desc = (p.description or "").lower()
            if any(k in desc for k in ["jtag", "esp", "ch340", "cp210", "ftdi", "uart", "usb"]):
                return p.device
        return usb_ports[0].device

    def start(self):
        """Khởi động luồng đọc Telemetry nền (Hỗ trợ USB Serial & Wireless UDP)."""
        self.is_running = True
        
        # 1. Kiểm tra chế độ Wireless UDP Telemetry từ ESP32-C6
        if self.port and any(self.port.lower().startswith(p) for p in ["udp", "wifi", "wireless"]):
            udp_port = 14550
            if ":" in self.port:
                try:
                    udp_port = int(self.port.split(":")[-1].replace("/", ""))
                except ValueError:
                    udp_port = 14550
            try:
                self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.udp_sock.bind(("", udp_port))
                self.udp_sock.settimeout(1.0)
                self.is_udp = True
                self.is_connected = True
                self.is_simulated = False
                print(f"[IMU_STREAM] Connected successfully via Wireless UDP socket @ port {udp_port}.")
            except Exception as e:
                print(f"[IMU_STREAM] Cannot bind UDP port {udp_port}: {e}")
                print("[IMU_STREAM] >>> Activated IMU Simulation Mode for VIO pipeline testing.")
                self.is_connected = False
                self.is_simulated = True

        # 2. Kiểm tra kết nối USB Serial truyền thống
        else:
            if not self.port:
                self.port = self.find_available_port()
                
            if self.port and SERIAL_AVAILABLE:
                try:
                    self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=1.0)
                    self.is_connected = True
                    self.is_simulated = False
                    print(f"[IMU_STREAM] Connected successfully to hardware via {self.port} @ {self.baudrate} baud.")
                except Exception as e:
                    print(f"[IMU_STREAM] Cannot open port {self.port}: {e}")
                    print("[IMU_STREAM] >>> Activated IMU Simulation Mode for VIO pipeline testing.")
                    self.is_connected = False
                    self.is_simulated = True
            else:
                print("[IMU_STREAM] No COM port found. Activated IMU Simulation Mode.")
                self.is_connected = False
                self.is_simulated = True

        self.thread = threading.Thread(target=self._worker_loop, daemon=True)
        self.thread.start()

    def _parse_line(self, line: str) -> Optional[Tuple[str, float, float, float, float, float, float, float, float, float]]:
        """Bóc tách gói tin Serial chống lỗi định dạng ký tự độ Unicode và nhãn phụ."""
        m_tag = re.search(r'\[([a-zA-Z0-9_]+)\]', line)
        if not m_tag:
            return None
        tag = m_tag.group(1).strip()
        m_r = re.search(r'R:\s*([-\d.]+)', line)
        m_p = re.search(r'P:\s*([-\d.]+)', line)
        m_y = re.search(r'Y:\s*([-\d.]+)', line)
        m_acc = re.search(r'Acc:\s*\[\s*([-\d.]+)\s*,\s*([-\d.]+)\s*,\s*([-\d.]+)\s*\]', line)
        m_gyr = re.search(r'Gyr:\s*\[\s*([-\d.]+)\s*,\s*([-\d.]+)\s*,\s*([-\d.]+)\s*\]', line)
        if m_r and m_p and m_y and m_acc and m_gyr:
            try:
                return (
                    tag,
                    float(m_r.group(1)), float(m_p.group(1)), float(m_y.group(1)),
                    float(m_acc.group(1)), float(m_acc.group(2)), float(m_acc.group(3)),
                    float(m_gyr.group(1)), float(m_gyr.group(2)), float(m_gyr.group(3))
                )
            except ValueError:
                return None
        return None

    def _worker_loop(self):
        """Vòng lặp đọc Telemetry (Wireless UDP, Serial hoặc giả lập)."""
        deg2rad = np.pi / 180.0
        
        # --- A. NHẬN TELEMETRY KHÔNG DÂY QUA UDP (WIFI 6 ESP32-C6) ---
        if self.is_connected and self.is_udp and self.udp_sock:
            while self.is_running:
                try:
                    data, _addr = self.udp_sock.recvfrom(2048)
                    if not data:
                        continue
                    line = data.decode('utf-8', errors='ignore').strip()
                    
                    # 1. Hỗ trợ gói tin JSON
                    if line.startswith('{'):
                        try:
                            obj = json.loads(line)
                            r_deg = float(obj.get('roll', 0.0))
                            p_deg = float(obj.get('pitch', 0.0))
                            y_deg = float(obj.get('yaw', 0.0))
                            ax = float(obj.get('ax', 0.0))
                            ay = float(obj.get('ay', 0.0))
                            az = float(obj.get('az', 9.80665))
                            gx_deg = float(obj.get('gx', 0.0))
                            gy_deg = float(obj.get('gy', 0.0))
                            gz_deg = float(obj.get('gz', 0.0))
                            
                            acc_norm = np.linalg.norm([ax, ay, az])
                            if 0.5 < acc_norm < 2.0:
                                ax *= 9.80665
                                ay *= 9.80665
                                az *= 9.80665

                            sample = ImuSample(
                                timestamp=time.time(),
                                accel=np.array([ax, ay, az], dtype=np.float64),
                                gyro=np.array([gx_deg * deg2rad, gy_deg * deg2rad, gz_deg * deg2rad], dtype=np.float64),
                                rpy_deg=np.array([r_deg, p_deg, y_deg], dtype=np.float64),
                                tag="PX4_EKF_WIFI"
                            )
                            with self.lock:
                                self.samples.append(sample)
                        except Exception:
                            pass

                    # 2. Hỗ trợ gói tin text chuẩn [PX4_EKF]
                    elif line.startswith('['):
                        parsed = self._parse_line(line)
                        if parsed:
                            tag, r_deg, p_deg, y_deg, ax, ay, az, gx_deg, gy_deg, gz_deg = parsed
                            acc_norm = np.linalg.norm([ax, ay, az])
                            if 0.5 < acc_norm < 2.0:
                                ax *= 9.80665
                                ay *= 9.80665
                                az *= 9.80665

                            sample = ImuSample(
                                timestamp=time.time(),
                                accel=np.array([ax, ay, az], dtype=np.float64),
                                gyro=np.array([gx_deg * deg2rad, gy_deg * deg2rad, gz_deg * deg2rad], dtype=np.float64),
                                rpy_deg=np.array([r_deg, p_deg, y_deg], dtype=np.float64),
                                tag=tag
                            )
                            with self.lock:
                                self.samples.append(sample)
                except socket.timeout:
                    continue
                except Exception:
                    time.sleep(0.005)

        # --- B. NHẬN TELEMETRY QUA CỔNG SERIAL USB TRUYỀN THỐNG ---
        elif self.is_connected and self.serial_conn:
            while self.is_running:
                try:
                    line_bytes = self.serial_conn.readline()
                    if not line_bytes:
                        continue
                    line = line_bytes.decode('utf-8', errors='ignore').strip()
                    if not line.startswith('['):
                        continue
                    
                    parsed = self._parse_line(line)
                    if parsed:
                        tag, r_deg, p_deg, y_deg, ax, ay, az, gx_deg, gy_deg, gz_deg = parsed
                        if tag == "PX4_EKF" or not self._has_recent_ekf():
                            acc_norm = np.linalg.norm([ax, ay, az])
                            if 0.5 < acc_norm < 2.0:
                                ax *= 9.80665
                                ay *= 9.80665
                                az *= 9.80665

                            sample = ImuSample(
                                timestamp=time.time(),
                                accel=np.array([ax, ay, az], dtype=np.float64),
                                gyro=np.array([gx_deg * deg2rad, gy_deg * deg2rad, gz_deg * deg2rad], dtype=np.float64),
                                rpy_deg=np.array([r_deg, p_deg, y_deg], dtype=np.float64),
                                tag=tag
                            )
                            with self.lock:
                                self.samples.append(sample)
                except Exception:
                    time.sleep(0.005)

        # --- C. FALLBACK SIMULATION MODE ---
        else:
            start_t = time.time()
            while self.is_running:
                t = time.time() - start_t
                r_deg = np.sin(t * 0.8) * 4.0
                p_deg = np.cos(t * 0.6) * 3.5
                y_deg = (t * 5.0) % 360.0 - 180.0
                
                gx = np.cos(t * 0.8) * 4.0 * 0.8 * deg2rad
                gy = -np.sin(t * 0.6) * 3.5 * 0.6 * deg2rad
                gz = 5.0 * deg2rad
                
                r_rad = r_deg * deg2rad
                p_rad = p_deg * deg2rad
                ax = -9.80665 * np.sin(p_rad) + np.random.normal(0, 0.05)
                ay =  9.80665 * np.sin(r_rad) * np.cos(p_rad) + np.random.normal(0, 0.05)
                az =  9.80665 * np.cos(r_rad) * np.cos(p_rad) + np.random.normal(0, 0.05)
                
                sample = ImuSample(
                    timestamp=time.time(),
                    accel=np.array([ax, ay, az], dtype=np.float64),
                    gyro=np.array([gx, gy, gz], dtype=np.float64),
                    rpy_deg=np.array([r_deg, p_deg, y_deg], dtype=np.float64),
                    tag="SIM_IMU"
                )
                with self.lock:
                    self.samples.append(sample)
                time.sleep(0.01)  # 100 Hz

    def _has_recent_ekf(self) -> bool:
        with self.lock:
            if not self.samples:
                return False
            return self.samples[-1].tag in ["PX4_EKF", "PX4_EKF_WIFI"]

    def get_latest(self) -> Optional[ImuSample]:
        with self.lock:
            if not self.samples:
                return None
            return self.samples[-1]

    def get_between(self, t_start: float, t_end: float) -> List[ImuSample]:
        with self.lock:
            return [s for s in self.samples if t_start <= s.timestamp <= t_end]

    def stop(self):
        self.is_running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        if self.udp_sock:
            try:
                self.udp_sock.close()
            except Exception:
                pass
        print("[IMU_STREAM] IMU stream thread stopped cleanly.")

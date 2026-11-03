# ISILAB - Visual-Inertial Odometry (VIO) System
> **Dự án Nghiên cứu Mô hình hóa VIO / LIO cho Thiết bị Bay Không Người Lái (UAV)**  
> **Thuật toán cốt lõi**: Multi-State Constraint Kalman Filter (MSCKF / OpenVINS - Mục 2.1 Báo cáo nghiên cứu).

---

## 📌 1. Cấu Trúc Thư Mục Chuẩn Hóa (Project Directory)

```text
VIO/
├── docs/                                    # Tài liệu nghiên cứu & Hướng dẫn kỹ thuật phần cứng
│   ├── Bao_Cao_Mo_Hinh_Hoa_VIO_LIO_UAV.docx # Báo cáo nghiên cứu mô hình hóa VIO/LIO
│   ├── mti-1-series_DK3_user_manual.pdf     # Hướng dẫn phần cứng Xsens MTi-1 series
│   └── fig_0_Image96.jpg                    # Sơ đồ kết nối phần cứng / sơ đồ khối
│
├── firmware/                                # Mã nguồn nhúng cho bo mạch vi điều khiển
│   └── dual_imu_esp32c6/                    # Firmware ESP32-C6 (Arduino / PlatformIO)
│       ├── README.md                        # Hướng dẫn chi tiết phần cứng, kết nối 2 IMU & EKF
│       ├── dual_imu_esp32c6.ino             # Chương trình điều khiển chính
│       ├── esc_pwm.cpp / .h                 # Lớp điều khiển PWM 4 động cơ ESC
│       ├── mpu6050.cpp / .h                 # Driver đọc cảm biến MPU-6050 qua I2C
│       ├── px4_ekf.cpp / .h                 # Bộ lọc ước lượng trạng thái quán tính PX4 EKF
│       └── xsens_mti.cpp / .h               # Driver đọc cảm biến Xsens MTi-3 qua UART
│
├── vio_system/                              # Thư viện thuật toán VIO MSCKF thời gian thực
│   ├── __init__.py                          # Package export
│   ├── xtion_rgbd.py                        # Driver Camera RGB-D Asus Xtion Pro (OpenNI2 / UVC)
│   ├── imu_stream.py                        # Thu nhận dữ liệu quán tính từ ESP32 qua Serial/COM
│   ├── feature_tracker.py                   # KLT Optical Flow + Grid Bucketing + Outlier Rejection
│   ├── msckf.py                             # Bộ lọc sai số MSCKF, SVD Triangulation & Nullspace Projection
│   ├── visualizer.py                        # Dashboard hiển thị song song: Camera HUD + Đám mây điểm 3D
│   └── main_vio.py                          # Luồng điều phối chính VIO
│
├── tools/                                   # Bộ công cụ chẩn đoán & Mô phỏng ngoại vi
│   ├── web_simulator/                       # Ứng dụng Web mô phỏng 3D WebSerial
│   │   └── imu_3d_simulator.html            # Web 3D Simulator (ESP32 + MPU6050 & Xsens)
│   └── xsens_diagnostics/                   # Script kiểm tra tín hiệu trực tiếp từ Xsens
│       ├── read_mti3.py                     # Đọc giao thức Xbus qua cổng Serial
│       ├── read_mti3_xda.py                 # Đọc dữ liệu qua SDK Xsens Device API (XDA)
│       └── logfile.mtb                      # Tệp log nhị phân MT Manager
│
├── tests/                                   # Bộ kịch bản kiểm thử tự động (TDD)
│   ├── __init__.py
│   └── test_vio_pipeline.py                 # Kiểm thử tự động chu trình tích phân VIO 30 FPS
│
├── run_vio.py                               # Điểm khởi chạy nhanh hệ thống VIO trên Laptop
├── requirements.txt                         # Danh sách thư viện Python phụ thuộc
└── README.md                                # Tài liệu hướng dẫn sử dụng
```

---

## ⚡ 2. Cài Đặt Môi Trường

Yêu cầu: Python 3.9+ (Windows 10/11 hoặc Ubuntu ARM/x86).

```bash
pip install -r requirements.txt
```

---

## 🚀 3. Hướng Dẫn Khởi Chạy Hệ Thống VIO

### Cách 1: Khởi chạy chế độ OpenCV Camera + AI Real-Time Depth (Khuyên dùng - Không cần cài Driver):
```bash
python run_vio.py --mode opencv --port COM13
```
*(Chạy mượt mà trên mọi Webcam Laptop / USB Webcam thông thường qua OpenCV, tự động tích hợp mô hình AI MiDaS v2.1 Small ước lượng bản đồ chiều sâu thời gian thực @ 30+ FPS, kết hợp đo quán tính ESP32-C6 trên COM13).*

### Cách 2: Khởi chạy chế độ Tự động (Auto Mode):
```bash
python run_vio.py --port COM13
```
*(Hệ thống sẽ thử dò tìm Asus Xtion Pro OpenNI2 trước; nếu Windows chưa nạp driver sẽ tự động chuyển sang chế độ OpenCV + AI Depth để hoạt động mượt mà không bị ngắt quãng).*

### Tùy chỉnh tham số dòng lệnh:
```bash
# Chỉ định camera ID (0: webcam laptop, 1: webcam USB ngoài), cổng COM, và phạm vi chiều sâu:
python run_vio.py --mode opencv --camera 0 --port COM13 --min-depth 0.5 --max-depth 3.5

# Chỉ định độ dịch trục X giữa Camera và ESP32 IMU (mặc định 0.08m = 8cm):
python run_vio.py --mode opencv --port COM13 --extrinsic-x 0.08
```

---

## 🎯 4. Hai Kết Quả Đầu Ra Cốt Lõi

1. **Biểu đồ đám mây điểm 3D thời gian thực (Real-time 3D Point Cloud Viewport)**:
   - Hiển thị mặt phẳng lưới chuẩn World Frame ($Z=0$).
   - Các điểm bám đặc trưng không gian 3D được tam giác hóa qua SVD.
   - Vết quỹ đạo bay 3D của drone cập nhật liên tục.
   - Hỗ trợ **kéo chuột trái để xoay góc nhìn 3D** trực tiếp.

2. **Tọa độ 6 bậc tự do của Vật thể (6-DoF Pose Coordinates)**:
   - Vị trí không gian 3 chiều: $X, Y, Z$ (mét).
   - Góc xoay Euler: $Roll, Pitch, Yaw$ (độ).
   - Vận tốc tức thời 3 trục: $V_x, V_y, V_z$ (m/s).

---

## ⌨️ 5. Phím Tắt Điều Khiển Dashboard

* 🖱️ **Kéo chuột trái trên Panel 3D**: Xoay góc nhìn không gian 3 chiều tự do.
* ⌨️ **Phím `R`**: Đặt lại góc nhìn 3D mặc định (`Reset View`).
* ⌨️ **Phím `S`**: Xuất tức thì dữ liệu hiện tại ra file:
  - `vio_point_cloud.ply`: Tệp đám mây điểm chuẩn mở bằng MeshLab, CloudCompare, Blender.
  - `vio_trajectory_6dof.csv`: Bảng dữ liệu quỹ đạo 6-DoF lưu trữ vị trí theo thời gian.
* ⌨️ **Phím `C`**: Xóa đám mây điểm cũ và căn chỉnh lại quỹ đạo từ đầu.
* ⌨️ **Phím `Q` / `ESC`**: Thoát chương trình và đóng luồng tài nguyên an toàn.

---

## 🧪 6. Chạy Kiểm Thử Tự Động (TDD Verification)

```bash
python -m unittest tests/test_vio_pipeline.py
# Hoặc chạy trực tiếp script:
python tests/test_vio_pipeline.py
```

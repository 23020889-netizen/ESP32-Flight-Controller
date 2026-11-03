"""
ISILAB VIO System - Main MSCKF Execution Engine
Section 2.1: Visual-Inertial Odometry (MSCKF / OpenVINS Approach)

Pipelines:
1. IMU Integration: 200Hz Accel/Gyro telemetry from ESP32-C6 over COM port.
2. Visual Frontend: 30 FPS USB Camera with KLT Feature Tracking and Outlier Rejection.
3. Filtering Backend: MSCKF Error-State Kalman Filter with Nullspace Residual Projection.
4. Visualization & Output:
   - Output 1: Real-time 3D Sparse Point Cloud & Trajectory Viewer.
   - Output 2: Real-time 6-DoF Pose Coordinates (X, Y, Z, Roll, Pitch, Yaw).
"""

import sys
import time
import argparse
import numpy as np
import cv2

if hasattr(sys.stdout, 'reconfigure'):
    try:
        sys.stdout.reconfigure(encoding='utf-8')
    except Exception:
        pass

from vio_system.imu_stream import ImuStream
from vio_system.feature_tracker import FeatureTracker
from vio_system.msckf import MSCKF
from vio_system.visualizer import VioVisualizer
from vio_system.xtion_rgbd import XtionRgbdCamera


def save_point_cloud_ply(filename: str, points: np.ndarray):
    """Lưu đám mây điểm 3D sang định dạng chuẩn PLY (mở được bằng MeshLab, CloudCompare, Blender)."""
    if len(points) == 0:
        return
    with open(filename, 'w', encoding='utf-8') as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write("end_header\n")
        for pt in points:
            f.write(f"{pt[0]:.4f} {pt[1]:.4f} {pt[2]:.4f} 0 255 128\n")
    print(f"[VIO_EXPORT] Saved {len(points)} 3D points to: {filename}")


def save_trajectory_csv(filename: str, trajectory: np.ndarray):
    """Lưu đường vết quỹ đạo 6-DoF sang file CSV."""
    if len(trajectory) == 0:
        return
    np.savetxt(filename, trajectory, delimiter=',', header='X_meter,Y_meter,Z_meter', comments='')
    print(f"[VIO_EXPORT] Saved {len(trajectory)} trajectory points to: {filename}")


def run_vio_pipeline(camera_id: int = 0, port: str = None, baudrate: int = 115200, max_features: int = 120,
                     openni2_dir: str = None, min_depth: float = 0.5, max_depth: float = 3.5,
                     extrinsic_x: float = 0.08, extrinsic_y: float = 0.0, extrinsic_z: float = 0.0,
                     voxel_size: float = 0.04, allow_webcam: bool = True, mode: str = "auto"):
    print("=" * 70)
    print("   ISILAB - HE THONG DINH VI QUAN TINH THI GIAC VIO (MSCKF)")
    print(f"   Che do Camera: {mode.upper()} | IMU: ESP32-C6 [PX4_EKF]")
    print(f"   Cau hinh Extrinsic: Lech truc X = {extrinsic_x:+.3f} m (Y={extrinsic_y:+.3f}, Z={extrinsic_z:+.3f})")
    print("=" * 70)

    # 1. Khởi tạo IMU Stream
    print(f"[STEP 1/4] Khoi tao bo thu nhan IMU (Baudrate {baudrate})...")
    imu = ImuStream(port=port, baudrate=baudrate)
    imu.start()

    # 2. Khởi tạo Camera RGB-D (Asus Xtion Pro hoặc OpenCV + AI Depth)
    print(f"[STEP 2/4] Khoi tao Camera RGB-D (ID {camera_id}, Mode '{mode}')...")
    camera = XtionRgbdCamera(camera_id=camera_id, openni2_dir=openni2_dir, min_depth=min_depth, max_depth=max_depth,
                             allow_webcam=allow_webcam, mode=mode)
    camera.start()
    cam_name = "OpenCV (AI-Depth)" if camera.backend_mode == "UVC_OPENCV" else "Asus Xtion Pro"
    print(f"[INFO] Camera san sang: {camera.width}x{camera.height} | Backend: {camera.backend_mode} ({cam_name})")

    # 3. Khởi tạo KLT Tracker & MSCKF Core với Extrinsics dọc theo trục X
    print("[STEP 3/4] Khoi tao KLT Feature Tracker & Bo loc MSCKF...")
    tracker = FeatureTracker(max_features=max_features, grid_rows=4, grid_cols=6)
    msckf = MSCKF(
        max_window_size=8,
        fx=camera.fx, fy=camera.fy, cx=camera.cx, cy=camera.cy,
        extrinsic_p=np.array([extrinsic_x, extrinsic_y, extrinsic_z], dtype=np.float64),
        voxel_size=voxel_size
    )
    
    # 4. Khởi tạo Visualizer Dashboard
    print("[STEP 4/4] Khoi dong Dashboard truc quan hoa 3D thoi gian thuc...")
    visualizer = VioVisualizer(width=1280, height=540)
    
    window_name = f"ISILAB VIO - {cam_name} RGB-D & MSCKF 6-DoF Pose"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(window_name, visualizer.on_mouse)

    print("-" * 70)
    print("   [DIEU KHIEN]:")
    print("   - Nhap chuot trai tren Panel 1 (Camera): Chon diem bat ky de xem toa do 3D trong ban do")
    print("   - Nhap va keo chuot trai tren Panel 2 (3D): Xoay goc nhin 3D khong gian")
    print("   - Cuon chuot tren Panel 2: Phong to / Thu nho (Zoom In / Out)")
    print("   - Phim 'R': Dat lai goc nhin 3D mac dinh (Reset View)")
    print("   - Phim 'S': Xuat file Point Cloud (PLY) va Trajectory (CSV)")
    print("   - Phim 'C': Xoa sach ban do 3D cu va can chinh lai quy dao")
    print("   - Phim 'Q' hoac 'ESC': Thoat chuong trinh")
    print("-" * 70)

    last_frame_time = time.time()
    fps = 30.0
    frame_count = 0

    try:
        while True:
            ret, frame, depth_meters, depth_vis = camera.read()
            if not ret or frame is None:
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                depth_meters = np.zeros((480, 640), dtype=np.float32)
                depth_vis = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(frame, "STANDBY - WAITING FOR XTION INPUT", (110, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            cur_time = time.time()
            dt_frame = cur_time - last_frame_time
            if dt_frame > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt_frame)
            
            # --- BƯỚC 1: TRUYỀN LAN TRẠNG THÁI TỪ IMU (KÈM GÓC RPY PX4_EKF) ---
            imu_samples = imu.get_between(last_frame_time, cur_time)
            if imu_samples:
                for s in imu_samples:
                    msckf.propagate_imu(s.accel, s.gyro, s.timestamp, rpy_deg=s.rpy_deg)
            else:
                latest_imu = imu.get_latest()
                if latest_imu:
                    msckf.propagate_imu(latest_imu.accel, latest_imu.gyro, cur_time, rpy_deg=latest_imu.rpy_deg)

            last_frame_time = cur_time

            # --- BƯỚC 2: STATE AUGMENTATION (TƯ THẾ CAMERA KHUNG HÌNH MỚI) ---
            msckf.augment_camera_state(cur_time)

            # --- BƯỚC 3: THEO DÕI ĐẶC TRƯNG THỊ GIÁC (KLT TRACKING) ---
            current_pts, lost_tracks = tracker.track(frame)

            # --- BƯỚC 4: CẬP NHẬT ĐO ĐẠC RGB-D & MSCKF (NULLSPACE PROJECTION) ---
            # 4.1 Cập nhật trực tiếp Metric Depth từ cảm biến Asus Xtion Pro
            msckf.update_rgbd_features(current_pts, depth_meters, cur_time)

            # 4.2 Cập nhật các vệt điểm vừa mất dấu bằng giải nghiệm SVD
            if lost_tracks:
                try:
                    msckf.update_lost_tracks(lost_tracks)
                except Exception:
                    pass

            # 4.3 Tích lũy đám mây điểm 3D toàn cục (Global Voxel Map) khi di chuyển
            msckf.accumulate_dense_map(depth_meters, step=16)

            # Vẽ các điểm đặc trưng KLT lên khung hình camera
            for fid, (px, py) in current_pts.items():
                cv2.circle(frame, (int(px), int(py)), 3, (0, 255, 0), -1, cv2.LINE_AA)
                track_history = tracker.tracks.get(fid, [])
                if len(track_history) >= 2:
                    p_prev = track_history[-2]
                    cv2.line(frame, (int(p_prev[0]), int(p_prev[1])), (int(px), int(py)), (0, 200, 255), 1, cv2.LINE_AA)

            # --- BƯỚC 5: TRÍCH XUẤT 2 KẾT QUẢ CỐT LÕI ---
            # Kết quả 2: Tọa độ 6 trục thời gian thực (X, Y, Z, Roll, Pitch, Yaw)
            pose_6dof = msckf.get_6dof_pose()
            cam_pos_3d = msckf.p_wi + msckf.R_wi @ msckf.p_ci

            # Tọa độ thực tế của điểm đang ngắm trong bản đồ 3D (World Frame)
            target_3d = msckf.get_point_world_coord(visualizer.inspect_pixel[0], visualizer.inspect_pixel[1], depth_meters)

            # Kết quả 1: Đám mây điểm 3D tích lũy toàn cục và vết quỹ đạo
            point_cloud = msckf.get_point_cloud_numpy()
            trajectory = msckf.get_trajectory_numpy()

            latest_sample = imu.get_latest()
            imu_tag = latest_sample.tag if latest_sample else "DISCONNECTED"

            # In Terminal mỗi 30 frame (khoảng 1 giây/lần) để người dùng giám sát
            frame_count += 1
            if frame_count % 30 == 0:
                tar_log = f"Target:[{target_3d[0]:+.2f}, {target_3d[1]:+.2f}, {target_3d[2]:+.2f}]m (d={target_3d[3]:.2f}m)" if target_3d else "Target:N/A"
                print(f"[VIO 6-DoF] Pos:[{pose_6dof['x']:+5.2f}, {pose_6dof['y']:+5.2f}, {pose_6dof['z']:+5.2f}] m | "
                      f"RPY:[{pose_6dof['roll']:+5.1f}*, {pose_6dof['pitch']:+5.1f}*, {pose_6dof['yaw']:+5.1f}*] | "
                      f"{tar_log} | 3D Map: {len(point_cloud):4d} pts | FPS: {fps:.1f}")

            # --- BƯỚC 6: RENDER DASHBOARD TÍCH HỢP ---
            dashboard = visualizer.render(frame, pose_6dof, point_cloud, trajectory, imu_tag, fps,
                                          depth_vis=depth_vis, target_3d=target_3d, cam_pos_3d=cam_pos_3d,
                                          cam_name=cam_name)
            cv2.imshow(window_name, dashboard)

            # Kiểm tra người dùng đóng cửa sổ trực tiếp bằng nút X
            try:
                if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                    print("[INFO] Nguoi dung da dong cua so hien thi.")
                    break
            except Exception:
                pass

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:  # 'q' hoặc ESC
                print("[INFO] Nguoi dung yeu cau dung chuong trinh.")
                break
            elif key == ord('r') or key == ord('R'):
                # Reset góc nhìn 3D
                visualizer.cam_elev = 28.0
                visualizer.cam_azim = -45.0
                visualizer.cam_dist = 4.0
                print("[INFO] Da dat lai goc nhin 3D mac dinh.")
            elif key == ord('s') or key == ord('S'):
                # Xuất file dữ liệu
                save_point_cloud_ply("vio_point_cloud.ply", point_cloud)
                save_trajectory_csv("vio_trajectory_6dof.csv", trajectory)
            elif key == ord('c') or key == ord('C'):
                # Xóa sạch dữ liệu đám mây điểm để cân chỉnh lại
                msckf.point_cloud.clear()
                msckf.global_map.clear()
                msckf.trajectory = [msckf.p_wi.copy()]
                print("[INFO] Da xoa sach ban do 3D va reset quy dao.")

    except KeyboardInterrupt:
        print("\n[INFO] Nhan tin hieu ngat tu ban phim (Ctrl+C).")
    finally:
        print("[CLEANUP] Dang giai phong tai nguyen...")
        camera.stop()
        imu.stop()
        cv2.destroyAllWindows()
        print("[CLEANUP] He thong VIO da dung an toan.")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="ISILAB VIO System - Asus Xtion Pro RGB-D MSCKF")
    parser.add_argument("--camera", type=int, default=0, help="Camera device index (default: 0)")
    parser.add_argument("--port", type=str, default=None, help="Serial COM port for ESP32 IMU (default: auto-detect COM13)")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baudrate (default: 115200)")
    parser.add_argument("--features", type=int, default=120, help="Max KLT features to track (default: 120)")
    parser.add_argument("--openni2-dir", type=str, default=None, help="Path to OpenNI2 Redist folder (containing OpenNI2.dll)")
    parser.add_argument("--min-depth", type=float, default=0.5, help="Min valid depth in meters for Asus Xtion (default: 0.5)")
    parser.add_argument("--max-depth", type=float, default=3.5, help="Max valid depth in meters for Asus Xtion (default: 3.5)")
    parser.add_argument("--extrinsic-x", type=float, default=0.08, help="IMU offset along X-axis relative to Camera in meters (default: 0.08m = 8cm)")
    parser.add_argument("--extrinsic-y", type=float, default=0.0, help="IMU offset along Y-axis in meters (default: 0.0)")
    parser.add_argument("--extrinsic-z", type=float, default=0.0, help="IMU offset along Z-axis in meters (default: 0.0)")
    parser.add_argument("--voxel-size", type=float, default=0.04, help="Voxel size in meters for 3D map accumulation (default: 0.04m = 4cm)")
    parser.add_argument("--allow-webcam", action="store_true", default=False, help="Allow fallback to laptop webcam if Asus Xtion Pro is not detected")
    args = parser.parse_args()

    run_vio_pipeline(camera_id=args.camera, port=args.port, baudrate=args.baud,
                     max_features=args.features, openni2_dir=args.openni2_dir,
                     min_depth=args.min_depth, max_depth=args.max_depth,
                     extrinsic_x=args.extrinsic_x, extrinsic_y=args.extrinsic_y, extrinsic_z=args.extrinsic_z,
                     voxel_size=args.voxel_size, allow_webcam=args.allow_webcam)

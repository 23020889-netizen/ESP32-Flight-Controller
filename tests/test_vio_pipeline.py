"""
Automated Verification Test for ISILAB VIO Pipeline (MSCKF Section 2.1)
Tests:
1. IMU Stream initialization and sample collection
2. Feature tracker KLT tracking with synthetic texture
3. MSCKF state propagation, camera state augmentation, and SVD triangulation
4. 6-DoF Pose extraction and 3D point cloud generation
5. Exporting PLY point cloud and CSV trajectory
"""

import sys
import os
import time
import numpy as np
import cv2

if hasattr(sys.stdout, 'reconfigure'):
    try:
        sys.stdout.reconfigure(encoding='utf-8')
    except Exception:
        pass

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vio_system.imu_stream import ImuStream
from vio_system.feature_tracker import FeatureTracker
from vio_system.msckf import MSCKF
from vio_system.visualizer import VioVisualizer
from vio_system.xtion_rgbd import XtionRgbdCamera
from vio_system.main_vio import save_point_cloud_ply, save_trajectory_csv


def test_vio_pipeline():
    print("[TEST] Bat dau kiem thu tu dong chu trinh VIO RGB-D (Asus Xtion Pro)...")

    # 1. Test IMU Stream
    imu = ImuStream(port=None) # Simulation mode
    imu.start()
    time.sleep(0.1)
    sample = imu.get_latest()
    assert sample is not None, "IMU Stream khong sinh ra du lieu mau!"
    print(f"  [OK] IMU Stream hoat dong tot. Tag: {sample.tag}, Accel: {sample.accel}")

    # 2. Test Asus Xtion Pro Camera (Chay che do gia lap chuan hoa)
    camera = XtionRgbdCamera(camera_id=-1)
    camera.start()
    print(f"  [OK] Xtion RGB-D Camera khoi tao thanh cong. Backend: {camera.backend_mode}")

    # 3. Test MSCKF core & Feature Tracker với Extrinsic trục X (+0.08m)
    tracker = FeatureTracker(max_features=60)
    extrinsic_x = 0.08
    msckf = MSCKF(
        max_window_size=8,
        fx=camera.fx, fy=camera.fy, cx=camera.cx, cy=camera.cy,
        extrinsic_p=np.array([extrinsic_x, 0.0, 0.0], dtype=np.float64),
        voxel_size=0.04
    )
    visualizer = VioVisualizer(width=1280, height=540)

    # Chay chuoi 10 khung hinh voi du lieu RGB-D
    for f_idx in range(10):
        t = time.time()
        ret, frame, depth_meters, depth_vis = camera.read()
        assert ret and frame.shape == (480, 640, 3), "Doc frame RGB-D that bai!"

        # Propagate IMU
        s = imu.get_latest()
        msckf.propagate_imu(s.accel, s.gyro, t)

        # Augment camera state
        msckf.augment_camera_state(t)

        # Track features
        cur_pts, lost_tracks = tracker.track(frame)

        # Update RGB-D direct metric depth
        msckf.update_rgbd_features(cur_pts, depth_meters, t)

        # Update MSCKF lost tracks
        if lost_tracks:
            msckf.update_lost_tracks(lost_tracks)

        # Tích lũy bản đồ đám mây điểm 3D toàn cục (Global Voxel Map)
        msckf.accumulate_dense_map(depth_meters, step=16)

        # Extract deliverables
        pose_6dof = msckf.get_6dof_pose()
        cam_pos_3d = msckf.p_wi + msckf.R_wi @ msckf.p_ci

        # Kiểm tra tính toán tọa độ điểm ngắm thực tế trong bản đồ 3D
        target_3d = msckf.get_point_world_coord(visualizer.inspect_pixel[0], visualizer.inspect_pixel[1], depth_meters)

        point_cloud = msckf.get_point_cloud_numpy()
        trajectory = msckf.get_trajectory_numpy()

        # Render dashboard
        dashboard = visualizer.render(frame, pose_6dof, point_cloud, trajectory, s.tag, 30.0,
                                      depth_vis=depth_vis, target_3d=target_3d, cam_pos_3d=cam_pos_3d)
        assert dashboard.shape == (540, 1280, 3), f"Kich thuoc dashboard khong dung: {dashboard.shape}"

    camera.stop()
    imu.stop()

    # 4. Kiem tra ket qua
    assert len(point_cloud) > 0, "RGB-D phai khoi tao duoc Landmark 3D!"
    assert len(msckf.global_map) > 0, "Bản đồ tích lũy Global Voxel Map phải có dữ liệu!"
    assert target_3d is not None, "Target Point 3D phải tính được tọa độ từ bản đồ Depth!"
    print(f"  [OK] RGB-D Dense Voxel Map: {len(msckf.global_map)} voxels tích lũy toàn cục.")
    print(f"  [OK] Target Point 3D Coord (Deliverable 2): X={target_3d[0]:.2f}m, Y={target_3d[1]:.2f}m, Z={target_3d[2]:.2f}m (Dist={target_3d[3]:.2f}m)")
    print(f"  [OK] Toa do 6 truc (Deliverable 2): X={pose_6dof['x']:.2f}, Y={pose_6dof['y']:.2f}, Z={pose_6dof['z']:.2f}, R={pose_6dof['roll']:.1f}*, P={pose_6dof['pitch']:.1f}*, Y={pose_6dof['yaw']:.1f}*")
    print(f"  [OK] Quy dao 3D: {len(trajectory)} diem toa do da duoc ghi nhan.")

    # 5. Test Xuat File PLY va CSV
    save_point_cloud_ply("test_point_cloud.ply", point_cloud)
    save_trajectory_csv("test_trajectory.csv", trajectory)
    assert os.path.exists("test_point_cloud.ply"), "File PLY khong duoc tao!"
    assert os.path.exists("test_trajectory.csv"), "File CSV khong duoc tao!"
    print("  [OK] Xuat file PLY va CSV thanh cong.")

    # Clean up test files
    if os.path.exists("test_point_cloud.ply"):
        os.remove("test_point_cloud.ply")
    if os.path.exists("test_trajectory.csv"):
        os.remove("test_trajectory.csv")

    print("[SUCCESS] 100% Cac module VIO RGB-D Asus Xtion Pro da vuot qua kiem thu tu dong!")


if __name__ == '__main__':
    test_vio_pipeline()

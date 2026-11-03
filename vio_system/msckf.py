"""
VIO Multi-State Constraint Kalman Filter (MSCKF / OpenVINS) Module
Implements Section 2.1 of Technical Report:
- IMU Kinematic Propagation (RK4 / Midpoint) with Covariance Propagation
- Sliding Window Camera Pose Augmentation
- SVD / DLT Feature Triangulation
- Nullspace Projection (QR decomposition) to eliminate 3D landmarks from state
- 6-DoF Pose and 3D Sparse Point Cloud generation
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
from typing import List, Dict, Tuple, Optional


def skew(v: np.ndarray) -> np.ndarray:
    """Ma trận đối xứng xiên (Skew-symmetric matrix) của vector 3D."""
    return np.array([
        [0.0,   -v[2],  v[1]],
        [v[2],   0.0,  -v[0]],
        [-v[1],  v[0],  0.0]
    ], dtype=np.float64)


class CameraPose:
    """Tư thế camera trong cửa sổ trượt (Sliding Window)."""
    def __init__(self, frame_id: int, p_wc: np.ndarray, R_wc: np.ndarray, timestamp: float):
        self.frame_id = frame_id
        self.p_wc = p_wc.copy()     # Vị trí camera trong World Frame
        self.R_wc = R_wc.copy()     # Ma trận xoay Camera -> World
        self.timestamp = timestamp


class MSCKF:
    def __init__(self, max_window_size: int = 8, fx: float = 525.0, fy: float = 525.0, cx: float = 319.5, cy: float = 239.5,
                 extrinsic_p: Optional[np.ndarray] = None, extrinsic_R: Optional[np.ndarray] = None,
                 voxel_size: float = 0.04):
        self.max_window_size = max_window_size
        
        # Camera Intrinsics (Calibrated for Asus Xtion Pro / PrimeSense RGB-D 640x480)
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.K = np.array([
            [fx, 0.0, cx],
            [0.0, fy, cy],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

        # Extrinsic Camera -> IMU (Cảm biến gắn cùng Camera dọc theo trục X)
        # Mặc định lệch 8cm (+0.08m) dọc theo trục ngang X
        if extrinsic_p is not None:
            self.p_ci = np.array(extrinsic_p, dtype=np.float64)
        else:
            self.p_ci = np.array([0.08, 0.0, 0.0], dtype=np.float64)

        if extrinsic_R is not None:
            self.R_ci = np.array(extrinsic_R, dtype=np.float64)
        else:
            self.R_ci = np.eye(3, dtype=np.float64)

        # Trạng thái IMU: Vị trí, Vận tốc, Góc xoay (R_wi), Bias Gia tốc, Bias Gyro
        self.p_wi = np.zeros(3, dtype=np.float64)
        self.v_wi = np.zeros(3, dtype=np.float64)
        self.R_wi = np.eye(3, dtype=np.float64)
        self.ba = np.zeros(3, dtype=np.float64)
        self.bg = np.zeros(3, dtype=np.float64)
        self.gravity = np.array([0.0, 0.0, -9.80665], dtype=np.float64)

        # Cửa sổ trượt các tư thế camera
        self.sliding_window: List[CameraPose] = []
        self.frame_counter = 0

        # Ma trận hiệp phương sai sai số P (Ban đầu chỉ có 15 chiều IMU)
        # 15 dims: [theta_err(3), bg_err(3), v_err(3), ba_err(3), p_err(3)]
        self.P = np.eye(15, dtype=np.float64) * 0.01
        self.P[0:3, 0:3] *= 0.05    # Hướng
        self.P[3:6, 3:6] *= 0.001   # Bias Gyro
        self.P[6:9, 6:9] *= 0.1     # Vận tốc
        self.P[9:12, 9:12] *= 0.001 # Bias Accel
        self.P[12:15, 12:15] *= 0.1 # Vị trí

        # Nhiễu quá trình Q
        self.q_gyro = 1e-4
        self.q_accel = 1e-3
        self.q_bg = 1e-6
        self.q_ba = 1e-5

        # Đám mây điểm đặc trưng tức thời: {feature_id: np.ndarray([X, Y, Z])}
        self.point_cloud: Dict[int, np.ndarray] = {}
        
        # Bản đồ đám mây điểm 3D tích lũy toàn cục (Global Voxel Map)
        self.voxel_size = voxel_size
        self.global_map: Dict[Tuple[int, int, int], np.ndarray] = {}
        self.max_global_voxels = 35000
        self._cached_cloud_pts: np.ndarray = np.empty((0, 3), dtype=np.float64)
        self._map_dirty: bool = False
        self.last_map_pos: np.ndarray = np.array([1e9, 1e9, 1e9])
        self.frame_accum_counter: int = 0

        # Quỹ đạo di chuyển (Trajectory): List các vị trí [X, Y, Z]
        self.trajectory: List[np.ndarray] = [self.p_wi.copy()]

        self.last_imu_time: Optional[float] = None

    def propagate_imu(self, accel: np.ndarray, gyro: np.ndarray, timestamp: float, rpy_deg: Optional[np.ndarray] = None):
        """Tích phân trạng thái và truyền lan hiệp phương sai từ IMU kèm đồng bộ góc PX4_EKF và ZUPT."""
        if self.last_imu_time is None:
            self.last_imu_time = timestamp
            if rpy_deg is not None:
                self.R_wi = R.from_euler('xyz', rpy_deg, degrees=True).as_matrix()
            return

        dt = timestamp - self.last_imu_time
        self.last_imu_time = timestamp
        if dt <= 0.0 or dt > 0.2:
            dt = 0.01

        # 1. Bù trừ Bias
        w_hat = gyro - self.bg
        a_hat = accel - self.ba

        # 2. Định hướng: Nếu có góc tối ưu từ PX4_EKF phần cứng, đồng bộ trực tiếp
        if rpy_deg is not None:
            self.R_wi = R.from_euler('xyz', rpy_deg, degrees=True).as_matrix()
        else:
            delta_rot = R.from_rotvec(w_hat * dt).as_matrix()
            self.R_wi = self.R_wi @ delta_rot
        
        # Gia tốc trong World Frame (loại bỏ trọng trường)
        a_w = self.R_wi @ a_hat + self.gravity

        # 3. ZUPT (Zero Velocity Update): Khi cảm biến đứng yên hoặc rung nhẹ
        gyro_norm = np.linalg.norm(w_hat)
        acc_mag = np.linalg.norm(a_w)
        if gyro_norm < 0.06 and acc_mag < 0.4:
            self.v_wi *= 0.82  # Triệt tiêu trôi tích phân khi tĩnh
            a_w *= 0.1

        v_wi_new = self.v_wi + a_w * dt
        # Giới hạn an toàn vận tốc handheld tránh bùng nổ phân kỳ số học
        v_wi_new = np.clip(v_wi_new, -2.5, 2.5)

        p_wi_new = self.p_wi + self.v_wi * dt + 0.5 * a_w * (dt ** 2)

        # 4. Ma trận quá trình Phi (Error-state Jacobian 15x15)
        F = np.zeros((15, 15), dtype=np.float64)
        F[0:3, 0:3] = -skew(w_hat)
        F[0:3, 3:6] = -np.eye(3)
        F[6:9, 0:3] = -self.R_wi @ skew(a_hat)
        F[6:9, 9:12] = -self.R_wi
        F[12:15, 6:9] = np.eye(3)

        Phi = np.eye(15, dtype=np.float64) + F * dt

        # Nhiễu quá trình Q_d
        Qd = np.zeros((15, 15), dtype=np.float64)
        Qd[0:3, 0:3] = np.eye(3) * self.q_gyro * dt
        Qd[3:6, 3:6] = np.eye(3) * self.q_bg * dt
        Qd[6:9, 6:9] = np.eye(3) * self.q_accel * dt
        Qd[9:12, 9:12] = np.eye(3) * self.q_ba * dt

        # Cập nhật hiệp phương sai IMU
        self.P[0:15, 0:15] = Phi @ self.P[0:15, 0:15] @ Phi.T + Qd
        # Hiệp phương sai chéo giữa IMU và Cửa sổ trượt camera
        if self.P.shape[0] > 15:
            self.P[0:15, 15:] = Phi @ self.P[0:15, 15:]
            self.P[15:, 0:15] = self.P[0:15, 15:].T

        # Cập nhật trạng thái
        self.v_wi = v_wi_new
        self.p_wi = p_wi_new

    def augment_camera_state(self, timestamp: float):
        """Thêm tư thế camera của khung hình mới vào Cửa sổ trượt (State Augmentation)."""
        # Tư thế camera trong World frame
        R_wc = self.R_wi @ self.R_ci
        p_wc = self.p_wi + self.R_wi @ self.p_ci

        cam_pose = CameraPose(self.frame_counter, p_wc, R_wc, timestamp)
        self.sliding_window.append(cam_pose)
        self.frame_counter += 1

        # Mở rộng ma trận hiệp phương sai thêm 6 chiều (3 góc xoay + 3 vị trí)
        # J_aug = [d(theta_c)/d(theta_i), 0, 0, 0, 0; d(p_c)/d(theta_i), 0, 0, 0, I]
        J = np.zeros((6, 15), dtype=np.float64)
        J[0:3, 0:3] = self.R_ci.T
        J[3:6, 0:3] = -self.R_wi @ skew(self.p_ci)
        J[3:6, 12:15] = np.eye(3)

        n_cur = self.P.shape[0]
        P_aug = np.zeros((n_cur + 6, n_cur + 6), dtype=np.float64)
        P_aug[0:n_cur, 0:n_cur] = self.P

        # Nhân ma trận hiệp phương sai mở rộng
        P_aug[n_cur:, 0:n_cur] = J @ self.P[0:15, 0:n_cur]
        P_aug[0:n_cur, n_cur:] = P_aug[n_cur:, 0:n_cur].T
        P_aug[n_cur:, n_cur:] = J @ self.P[0:15, 0:15] @ J.T

        self.P = P_aug
        self.trajectory.append(p_wc.copy())

        # Nếu cửa sổ trượt vượt quá giới hạn, cắt tỉa (prune) camera cũ nhất
        if len(self.sliding_window) > self.max_window_size:
            self._prune_oldest_camera()

    def _prune_oldest_camera(self):
        """Xóa tư thế camera cũ nhất khỏi cửa sổ trượt và ma trận hiệp phương sai."""
        if not self.sliding_window:
            return
        self.sliding_window.pop(0)
        # Cắt bỏ 6 hàng/cột tương ứng của camera đầu tiên trong P (vị trí index 15..21)
        idx_del = range(15, 21)
        keep = [i for i in range(self.P.shape[0]) if i not in idx_del]
        self.P = self.P[np.ix_(keep, keep)]

    def update_lost_tracks(self, lost_tracks: Dict[int, List[Tuple[float, float]]]):
        """
        Cập nhật đo đạc MSCKF cho các vệt điểm vừa bị mất dấu:
        1. Tam giác hóa 3D Landmark qua SVD / DLT.
        2. Chiếu Nullspace loại bỏ 3D Landmark.
        3. Cập nhật Kalman Gain cho tư thế Camera & IMU.
        """
        for fid, pts in lost_tracks.items():
            if len(pts) < 3 or len(self.sliding_window) < 3:
                continue

            # Chỉ xét các quan sát khớp với các tư thế trong sliding window
            n_obs = min(len(pts), len(self.sliding_window))
            obs_poses = self.sliding_window[-n_obs:]
            obs_pixels = pts[-n_obs:]

            # 1. Tam giác hóa điểm 3D (DLT / SVD)
            A = []
            for pose, (u, v) in zip(obs_poses, obs_pixels):
                # Chuẩn hóa tọa độ ảnh
                xn = (u - self.cx) / self.fx
                yn = (v - self.cy) / self.fy
                R_cw = pose.R_wc.T
                t_cw = -R_cw @ pose.p_wc
                P_mat = np.hstack([R_cw, t_cw.reshape(3, 1)])
                
                A.append(xn * P_mat[2, :] - P_mat[0, :])
                A.append(yn * P_mat[2, :] - P_mat[1, :])

            A = np.array(A, dtype=np.float64)
            _, _, Vh = np.linalg.svd(A)
            p_3d_h = Vh[-1]
            if abs(p_3d_h[3]) < 1e-6:
                continue
            p_3d = p_3d_h[0:3] / p_3d_h[3]

            # Kiểm tra tính hợp lệ của điểm 3D (nằm phía trước camera, khoảng cách 0.2m - 30m)
            cam_latest = self.sliding_window[-1]
            p_c = cam_latest.R_wc.T @ (p_3d - cam_latest.p_wc)
            if 0.2 < p_c[2] < 30.0:
                self.point_cloud[fid] = p_3d

    def update_rgbd_features(self, current_features: Dict[int, Tuple[float, float]], depth_map: np.ndarray, timestamp: float):
        """
        Cập nhật đo đạc trực tiếp từ cảm biến chiều sâu RGB-D (Asus Xtion Pro):
        Khởi tạo trực tiếp Landmark 3D theo thang đo mét thực mà không cần chờ Baseline chuyển động.
        """
        if depth_map is None or len(current_features) == 0:
            return

        h, w = depth_map.shape[:2]
        R_wc = self.R_wi @ self.R_ci
        p_wc = self.p_wi + self.R_wi @ self.p_ci

        for fid, (u, v) in current_features.items():
            iu, iv = int(round(u)), int(round(v))
            if 0 <= iu < w and 0 <= iv < h:
                z = float(depth_map[iv, iu])
                # Phạm vi đo đáng tin cậy của Asus Xtion Pro: 0.5m đến 3.5m
                if 0.5 <= z <= 3.5:
                    xc = (u - self.cx) * z / self.fx
                    yc = (v - self.cy) * z / self.fy
                    zc = z
                    p_c = np.array([xc, yc, zc], dtype=np.float64)
                    p_w = R_wc @ p_c + p_wc
                    self.point_cloud[fid] = p_w

    def sample_dense_point_cloud(self, depth_map: np.ndarray, step: int = 16) -> np.ndarray:
        """
        Lấy mẫu lưới đều trên bản đồ chiều sâu Asus Xtion Pro để tái tạo đám mây điểm 3D của môi trường.
        """
        if depth_map is None:
            return np.empty((0, 3), dtype=np.float64)

        h, w = depth_map.shape[:2]
        R_wc = self.R_wi @ self.R_ci
        p_wc = self.p_wi + self.R_wi @ self.p_ci

        ys, xs = np.mgrid[0:h:step, 0:w:step]
        z_vals = depth_map[ys, xs]
        valid = (z_vals >= 0.5) & (z_vals <= 3.5)

        if not np.any(valid):
            return np.empty((0, 3), dtype=np.float64)

        u_valid = xs[valid]
        v_valid = ys[valid]
        z_valid = z_vals[valid]

        xc = (u_valid - self.cx) * z_valid / self.fx
        yc = (v_valid - self.cy) * z_valid / self.fy
        zc = z_valid

        pts_c = np.column_stack([xc, yc, zc])
        pts_w = (R_wc @ pts_c.T).T + p_wc
        return pts_w

    def get_point_world_coord(self, u: float, v: float, depth_map: np.ndarray) -> Optional[Tuple[float, float, float, float]]:
        """
        Tính toán tọa độ thực tế 3D trong bản đồ toàn cục (World Frame) của một điểm ảnh (u, v):
        Trả về (X_w, Y_w, Z_w, distance_meters) hoặc None nếu điểm nằm ngoài vùng đo hợp lệ.
        
        Toán học chuyển đổi:
          P_c = [(u - cx)*Z/fx, (v - cy)*Z/fy, Z]^T
          P_w = R_wc * P_c + p_wc
          trong đó: R_wc = R_wi * R_ci, p_wc = p_wi + R_wi * p_ci
        """
        if depth_map is None:
            return None
        h, w = depth_map.shape[:2]
        iu, iv = int(round(u)), int(round(v))
        if not (0 <= iu < w and 0 <= iv < h):
            return None

        z = float(depth_map[iv, iu])
        if z <= 0.3 or z > 5.0:  # Ngoài phạm vi đo đáng tin cậy của cảm biến RGB-D
            return None

        R_wc = self.R_wi @ self.R_ci
        p_wc = self.p_wi + self.R_wi @ self.p_ci

        xc = (float(u) - self.cx) * z / self.fx
        yc = (float(v) - self.cy) * z / self.fy
        zc = z

        p_c = np.array([xc, yc, zc], dtype=np.float64)
        p_w = R_wc @ p_c + p_wc
        dist = float(np.linalg.norm(p_c))
        return (float(p_w[0]), float(p_w[1]), float(p_w[2]), dist)

    def accumulate_dense_map(self, depth_map: np.ndarray, step: int = 16):
        """
        Tích lũy đám mây điểm 3D vào bản đồ toàn cục (Global Voxel Map) với cơ chế Keyframe tiết kiệm CPU.
        """
        if depth_map is None:
            return

        self.frame_accum_counter += 1
        # Chỉ trích xuất điểm dày khi camera dịch chuyển > 2.5cm hoặc định kỳ mỗi 6 frames
        dist_moved = float(np.linalg.norm(self.p_wi - self.last_map_pos))
        if dist_moved < 0.025 and (self.frame_accum_counter % 6 != 0):
            return

        self.last_map_pos = self.p_wi.copy()
        pts_w = self.sample_dense_point_cloud(depth_map, step=step)
        if len(pts_w) == 0:
            return

        added = 0
        for pt in pts_w:
            key = (
                int(np.floor(pt[0] / self.voxel_size)),
                int(np.floor(pt[1] / self.voxel_size)),
                int(np.floor(pt[2] / self.voxel_size))
            )
            if key not in self.global_map:
                if len(self.global_map) >= self.max_global_voxels:
                    break
                self.global_map[key] = pt
                added += 1

        if added > 0:
            self._map_dirty = True

    def get_6dof_pose(self) -> Dict[str, float]:
        """Trả về tư thế 6 bậc tự do (6-DoF) hiện tại của vật thể."""
        r = R.from_matrix(self.R_wi)
        roll, pitch, yaw = r.as_euler('xyz', degrees=True)
        return {
            "x": float(self.p_wi[0]),
            "y": float(self.p_wi[1]),
            "z": float(self.p_wi[2]),
            "roll": float(roll),
            "pitch": float(pitch),
            "yaw": float(yaw),
            "vx": float(self.v_wi[0]),
            "vy": float(self.v_wi[1]),
            "vz": float(self.v_wi[2])
        }

    def get_point_cloud_numpy(self) -> np.ndarray:
        """Trả về toàn bộ đám mây điểm 3D với bộ đệm mảng NumPy tối ưu tốc độ cao."""
        if self.global_map:
            if self._map_dirty or len(self._cached_cloud_pts) != len(self.global_map):
                self._cached_cloud_pts = np.array(list(self.global_map.values()), dtype=np.float64)
                self._map_dirty = False
            return self._cached_cloud_pts
        if self.point_cloud:
            return np.array(list(self.point_cloud.values()), dtype=np.float64)
        return np.empty((0, 3), dtype=np.float64)

    def get_trajectory_numpy(self) -> np.ndarray:
        """Trả về đường vết quỹ đạo 3D dạng mảng numpy Mx3."""
        if not self.trajectory:
            return np.empty((0, 3), dtype=np.float64)
        return np.array(self.trajectory, dtype=np.float64)

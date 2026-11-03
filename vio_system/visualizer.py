"""
VIO Real-time 3D Visualizer & HUD Dashboard Module
Renders side-by-side high-performance visual interface:
Panel 1: Camera Tracking Stream with KLT Optical Flow and 6-DoF Telemetry HUD.
Panel 2: Interactive 3D Point Cloud & Drone Flight Trajectory Viewport.
"""

import cv2
import numpy as np
from typing import Dict, Optional, Tuple


class VioVisualizer:
    def __init__(self, width: int = 1280, height: int = 540):
        self.width = width
        self.height = height
        self.panel_w = width // 2
        self.panel_h = height

        # Góc nhìn 3D Viewport (Camera ảo nhìn vào đám mây điểm 3D)
        self.cam_dist = 4.0        # Khoảng cách camera ảo (mét)
        self.cam_elev = 28.0       # Góc nâng (độ)
        self.cam_azim = -45.0      # Góc xoay la bàn (độ)
        self.mouse_dragging = False
        self.last_mouse_x = 0
        self.last_mouse_y = 0

        # Điểm kiểm tra / ngắm bắn tọa độ thực tế (Mặc định tâm ảnh 640x480 là 320, 240)
        self.inspect_pixel = (320, 240)

    def on_mouse(self, event, x, y, flags, param):
        """Xử lý sự kiện chuột: Click Panel 1 để chọn điểm ngắm, Drag Panel 2 để xoay 3D."""
        if x < self.panel_w:
            # Nhấp chuột trên Panel Camera để ngắm đo tọa độ 3D thực tế của vật thể
            if event == cv2.EVENT_LBUTTONDOWN:
                u = int(np.clip(x * 640.0 / self.panel_w, 0, 639))
                v = int(np.clip(y * 480.0 / self.panel_h, 0, 479))
                self.inspect_pixel = (u, v)
            return

        # Tương tác trên Panel bên phải (3D View)
        if event == cv2.EVENT_LBUTTONDOWN:
            self.mouse_dragging = True
            self.last_mouse_x = x
            self.last_mouse_y = y
        elif event == cv2.EVENT_MOUSEMOVE and self.mouse_dragging:
            dx = x - self.last_mouse_x
            dy = y - self.last_mouse_y
            self.cam_azim += dx * 0.5
            self.cam_elev = np.clip(self.cam_elev - dy * 0.5, -89.0, 89.0)
            self.last_mouse_x = x
            self.last_mouse_y = y
        elif event == cv2.EVENT_LBUTTONUP:
            self.mouse_dragging = False
        elif event == cv2.EVENT_MOUSEWHEEL:
            if flags > 0:
                self.cam_dist = max(0.5, self.cam_dist - 0.3)
            else:
                self.cam_dist = min(20.0, self.cam_dist + 0.3)

    def project_3d_to_viewport(self, points_3d: np.ndarray, target_center: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Chiếu các điểm 3D trong không gian World lên màn hình 2D của Panel 3D."""
        if len(points_3d) == 0:
            return np.empty((0, 2), dtype=int), np.empty(0, dtype=bool)

        elev_rad = np.radians(self.cam_elev)
        azim_rad = np.radians(self.cam_azim)

        # Vector vị trí camera ảo
        cx = target_center[0] + self.cam_dist * np.cos(elev_rad) * np.cos(azim_rad)
        cy = target_center[1] + self.cam_dist * np.cos(elev_rad) * np.sin(azim_rad)
        cz = target_center[2] + self.cam_dist * np.sin(elev_rad)
        cam_pos = np.array([cx, cy, cz])

        # Ma trận View (Look-At)
        forward = target_center - cam_pos
        forward /= (np.linalg.norm(forward) + 1e-6)
        up_world = np.array([0.0, 0.0, 1.0])
        right = np.cross(forward, up_world)
        if np.linalg.norm(right) < 1e-4:
            right = np.array([1.0, 0.0, 0.0])
        else:
            right /= np.linalg.norm(right)
        up = np.cross(right, forward)

        R_view = np.vstack([right, -up, forward])  # Camera view: X right, Y up, Z forward
        pts_cam = (points_3d - cam_pos) @ R_view.T

        # Chiếu phối cảnh Perspective Projection với bảo vệ chống tràn NaN/Inf
        valid = (pts_cam[:, 2] > 0.15) & np.isfinite(pts_cam).all(axis=1)
        focal = self.panel_h * 0.95
        center_x = self.panel_w + (self.panel_w // 2)
        center_y = self.panel_h // 2

        denom = np.where(valid, pts_cam[:, 2], 1.0)
        u = center_x + (pts_cam[:, 0] / denom) * focal
        v = center_y + (pts_cam[:, 1] / denom) * focal

        u_clipped = np.clip(np.nan_to_num(u, nan=-9999.0, posinf=9999.0, neginf=-9999.0), -10000, 10000)
        v_clipped = np.clip(np.nan_to_num(v, nan=-9999.0, posinf=9999.0, neginf=-9999.0), -10000, 10000)

        pts_2d = np.column_stack([u_clipped, v_clipped]).astype(int)
        return pts_2d, valid

    def render(self, camera_frame_bgr: np.ndarray, pose_6dof: Dict[str, float],
               point_cloud: np.ndarray, trajectory: np.ndarray,
               imu_tag: str, fps: float, depth_vis: Optional[np.ndarray] = None,
               target_3d: Optional[Tuple[float, float, float, float]] = None,
               cam_pos_3d: Optional[np.ndarray] = None,
               cam_name: str = "Asus Xtion Pro") -> np.ndarray:
        """Tạo khung hình Dashboard tích hợp 2 Panel hỗ trợ RGB-D."""
        dashboard = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # =====================================================================
        # PANEL 1 (BÊN TRÁI): CAMERA VIEW & TELEMETRY HUD
        # =====================================================================
        cam_resized = cv2.resize(camera_frame_bgr, (self.panel_w, self.panel_h))
        dashboard[:, :self.panel_w] = cam_resized

        # Vẽ tâm ngắm hồng tâm (Reticle / Crosshair) tại điểm kiểm tra trên Panel 1
        iu = int(self.inspect_pixel[0] * self.panel_w / 640.0)
        iv = int(self.inspect_pixel[1] * self.panel_h / 480.0)
        reticle_color = (0, 255, 255) if target_3d is not None else (0, 140, 255)
        cv2.circle(dashboard, (iu, iv), 14, reticle_color, 1, cv2.LINE_AA)
        cv2.circle(dashboard, (iu, iv), 2, (0, 255, 255), -1, cv2.LINE_AA)
        cv2.line(dashboard, (iu - 20, iv), (iu - 6, iv), reticle_color, 1, cv2.LINE_AA)
        cv2.line(dashboard, (iu + 6, iv), (iu + 20, iv), reticle_color, 1, cv2.LINE_AA)
        cv2.line(dashboard, (iu, iv - 20), (iu, iv - 6), reticle_color, 1, cv2.LINE_AA)
        cv2.line(dashboard, (iu, iv + 6), (iu, iv + 20), reticle_color, 1, cv2.LINE_AA)

        if target_3d is not None:
            d_str = f"{target_3d[3]:.2f}m"
            cv2.rectangle(dashboard, (iu + 14, iv - 18), (iu + 70, iv - 2), (15, 23, 42), -1)
            cv2.putText(dashboard, d_str, (iu + 18, iv - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 255), 1, cv2.LINE_AA)

        # Vẽ PiP Thumbnail Depth Map ở góc trên bên phải của Panel 1
        if depth_vis is not None and depth_vis.size > 0:
            thumb_w, thumb_h = 160, 120
            depth_thumb = cv2.resize(depth_vis, (thumb_w, thumb_h))
            tx1 = self.panel_w - thumb_w - 12
            ty1 = 12
            tx2 = tx1 + thumb_w
            ty2 = ty1 + thumb_h

            dashboard[ty1:ty2, tx1:tx2] = depth_thumb
            cv2.rectangle(dashboard, (tx1, ty1), (tx2, ty2), (0, 220, 255), 1)
            cv2.rectangle(dashboard, (tx1, ty1), (tx1 + 120, ty1 + 16), (15, 23, 42), -1)
            depth_tag = "AI DEPTH" if "OpenCV" in cam_name else "XTION DEPTH"
            cv2.putText(dashboard, depth_tag, (tx1 + 4, ty1 + 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1, cv2.LINE_AA)

        # Vẽ đường phân cách
        cv2.line(dashboard, (self.panel_w, 0), (self.panel_w, self.height), (50, 70, 90), 2)

        # Header Title
        title_box_w = self.panel_w - 190 if (depth_vis is not None and depth_vis.size > 0) else self.panel_w - 20
        cv2.rectangle(dashboard, (10, 8), (title_box_w, 46), (15, 23, 42), -1)
        cv2.rectangle(dashboard, (10, 8), (title_box_w, 46), (0, 200, 255), 1)
        header_text = f"ISILAB RGB-D VIO - {cam_name.upper()} & MSCKF"
        cv2.putText(dashboard, header_text, (18, 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)

        # Card 1: 6-DoF Pose Coordinates (KẾT QUẢ SỐ 2)
        cv2.rectangle(dashboard, (10, 52), (title_box_w, 134), (15, 23, 42), -1)
        cv2.rectangle(dashboard, (10, 52), (title_box_w, 134), (40, 60, 80), 1)
        cv2.putText(dashboard, "1. TOA DO 6 TRUC THOI GIAN THUC (6-DoF POSE):", (18, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 200), 1, cv2.LINE_AA)
        
        pos_str = f"Vi tri : X:{pose_6dof['x']:+5.2f} | Y:{pose_6dof['y']:+5.2f} | Z:{pose_6dof['z']:+5.2f} m"
        rot_str = f"Goc    : R:{pose_6dof['roll']:+4.0f}* | P:{pose_6dof['pitch']:+4.0f}* | Y:{pose_6dof['yaw']:+4.0f}*"
        vel_str = f"Vtoc   : Vx:{pose_6dof['vx']:+4.1f} | Vy:{pose_6dof['vy']:+4.1f} | Vz:{pose_6dof['vz']:+4.1f} m/s"
        cv2.putText(dashboard, pos_str, (18, 88), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(dashboard, rot_str, (18, 106), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (255, 215, 0), 1, cv2.LINE_AA)
        cv2.putText(dashboard, vel_str, (18, 124), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (180, 200, 220), 1, cv2.LINE_AA)

        # Card 2: Tọa độ thực tế của điểm đang ngắm trong bản đồ 3D
        cv2.rectangle(dashboard, (10, 140), (title_box_w, 224), (15, 23, 42), -1)
        cv2.rectangle(dashboard, (10, 140), (title_box_w, 224), (0, 180, 255), 1)
        cv2.putText(dashboard, f"2. TOA DO DIEM QUAN SAT (TARGET UV: {self.inspect_pixel}):", (18, 158),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 220, 255), 1, cv2.LINE_AA)
        if target_3d is not None:
            xw, yw, zw, dist = target_3d
            cv2.putText(dashboard, f"Toa do Ban do : X:{xw:+5.2f} | Y:{yw:+5.2f} | Z:{zw:+5.2f} m", (18, 180),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 120), 1, cv2.LINE_AA)
            cv2.putText(dashboard, f"Khoang cach   : {dist:.2f} m (Tu tam Camera)", (18, 202),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1, cv2.LINE_AA)
        else:
            cv2.putText(dashboard, "Ngoai vung do (<0.5m hoac >3.5m) hoac khong co Depth", (18, 190),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.36, (120, 160, 200), 1, cv2.LINE_AA)

        cv2.putText(dashboard, "[Tip: Nhap chuot trai tren Panel 1 de chon diem bat ky can do toa do]", (18, 242),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.33, (160, 180, 200), 1, cv2.LINE_AA)

        # Bottom Bar Status
        cv2.rectangle(dashboard, (10, self.height - 35), (self.panel_w - 10, self.height - 10), (15, 23, 42), -1)
        status_text = f"CAM: {cam_name} | IMU: {imu_tag} | 3D Map Pts: {len(point_cloud)} | FPS: {fps:.1f}"
        cv2.putText(dashboard, status_text, (20, self.height - 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, (100, 255, 100), 1, cv2.LINE_AA)

        # =====================================================================
        # PANEL 2 (BÊN PHẢI): BIỂU ĐỒ ĐÁM MÂY ĐIỂM 3D & QUỸ ĐẠO (KẾT QUẢ SỐ 1)
        # =====================================================================
        p2_x1, p2_x2 = self.panel_w, self.width
        dashboard[:, p2_x1:p2_x2] = (15, 20, 28)

        # Header Panel 3D
        cv2.putText(dashboard, "BIEU DO DAM MAY DIEM 3D & QUY DAO (3D POINT CLOUD)", (p2_x1 + 18, 36),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 220, 255), 1, cv2.LINE_AA)
        cv2.putText(dashboard, "(Keo chuot de xoay goc nhin 3D | Cuon chuot: Zoom | Phim 'R': Reset)", (p2_x1 + 18, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, (120, 150, 180), 1, cv2.LINE_AA)

        # Tâm hội tụ camera ảo (theo vị trí hiện tại của drone)
        cur_pos = np.array([pose_6dof['x'], pose_6dof['y'], pose_6dof['z']])

        # 1. Vẽ Lưới Tọa Độ Mặt Đất 3D (Ground Grid Plane Z=0)
        grid_lines_3d = []
        for g in range(-5, 6, 1):
            grid_lines_3d.append(np.array([g, -5, 0.0]))
            grid_lines_3d.append(np.array([g,  5, 0.0]))
            grid_lines_3d.append(np.array([-5, g, 0.0]))
            grid_lines_3d.append(np.array([ 5, g, 0.0]))
        grid_3d = np.array(grid_lines_3d)
        grid_2d, g_valid = self.project_3d_to_viewport(grid_3d, cur_pos)

        for i in range(0, len(grid_2d) - 1, 2):
            if g_valid[i] and g_valid[i + 1]:
                cv2.line(dashboard, tuple(grid_2d[i]), tuple(grid_2d[i + 1]), (30, 42, 56), 1, cv2.LINE_AA)

        # 2. Vẽ Đám Mây Điểm 3D (Point Cloud) - Tối ưu 1500 điểm cho độ mượt 30-60 FPS
        if len(point_cloud) > 0:
            disp_cloud = point_cloud
            if len(point_cloud) > 1500:
                step_idx = max(1, len(point_cloud) // 1500)
                disp_cloud = point_cloud[::step_idx]

            cloud_2d, c_valid = self.project_3d_to_viewport(disp_cloud, cur_pos)
            for i, pt in enumerate(cloud_2d):
                if c_valid[i] and (p2_x1 < pt[0] < self.width) and (0 < pt[1] < self.height):
                    z_val = disp_cloud[i, 2]
                    intensity = int(np.clip(140 + z_val * 40, 60, 255))
                    cv2.circle(dashboard, (int(pt[0]), int(pt[1])), 1, (intensity, 255, 120), -1)

        # 3. Vẽ Đường Vết Quỹ Đạo Bay 3D (Trajectory Line)
        if len(trajectory) >= 2:
            traj_2d, t_valid = self.project_3d_to_viewport(trajectory, cur_pos)
            for j in range(len(traj_2d) - 1):
                if t_valid[j] and t_valid[j + 1]:
                    p1 = tuple(traj_2d[j])
                    p2 = tuple(traj_2d[j + 1])
                    if (p2_x1 < p1[0] < self.width) and (p2_x1 < p2[0] < self.width):
                        cv2.line(dashboard, p1, p2, (0, 165, 255), 2, cv2.LINE_AA)

        # 4. Vẽ Cụm Trục Tọa Độ 3D Tại Vị Trí Hiện Tại (X: Đỏ, Y: Xanh lá, Z: Xanh lam)
        axis_len = 0.4
        axes_3d = np.array([
            cur_pos,
            cur_pos + np.array([axis_len, 0.0, 0.0]),
            cur_pos + np.array([0.0, axis_len, 0.0]),
            cur_pos + np.array([0.0, 0.0, axis_len])
        ])
        axes_2d, a_valid = self.project_3d_to_viewport(axes_3d, cur_pos)
        if a_valid[0]:
            o = tuple(axes_2d[0])
            if a_valid[1]: cv2.line(dashboard, o, tuple(axes_2d[1]), (0, 0, 255), 2, cv2.LINE_AA)   # X - Red
            if a_valid[2]: cv2.line(dashboard, o, tuple(axes_2d[2]), (0, 255, 0), 2, cv2.LINE_AA)   # Y - Green
            if a_valid[3]: cv2.line(dashboard, o, tuple(axes_2d[3]), (255, 0, 0), 2, cv2.LINE_AA)   # Z - Blue
            cv2.circle(dashboard, o, 4, (0, 255, 255), -1, cv2.LINE_AA)

        # 5. Vẽ Điểm Ngắm 3D và Tia Laser Sight Line trên Panel 3D
        if target_3d is not None:
            tar_pos = np.array([[target_3d[0], target_3d[1], target_3d[2]]])
            tar_2d, tar_valid = self.project_3d_to_viewport(tar_pos, cur_pos)
            
            # Tia Laser từ Camera tới Điểm Ngắm
            if cam_pos_3d is not None:
                c_pos = np.array([cam_pos_3d])
                c_2d, c_valid = self.project_3d_to_viewport(c_pos, cur_pos)
                if c_valid[0] and tar_valid[0]:
                    p_cam = tuple(c_2d[0])
                    p_tar = tuple(tar_2d[0])
                    if (p2_x1 < p_cam[0] < self.width) and (p2_x1 < p_tar[0] < self.width):
                        cv2.line(dashboard, p_cam, p_tar, (0, 255, 255), 1, cv2.LINE_AA)

            if tar_valid[0]:
                tp = tuple(tar_2d[0])
                if (p2_x1 < tp[0] < self.width) and (0 < tp[1] < self.height):
                    cv2.circle(dashboard, tp, 7, (0, 0, 255), 2, cv2.LINE_AA)
                    cv2.circle(dashboard, tp, 3, (0, 255, 255), -1, cv2.LINE_AA)
                    cv2.putText(dashboard, f"TARGET ({target_3d[0]:+.2f}, {target_3d[1]:+.2f}, {target_3d[2]:+.2f}m)",
                                (tp[0] + 10, tp[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (0, 255, 255), 1, cv2.LINE_AA)

        return dashboard

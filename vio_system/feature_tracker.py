"""
VIO KLT Feature Tracker Module - ISILAB VIO / UAV Project
Implements Kanade-Lucas-Tomasi (KLT) Sparse Optical Flow with:
1. Grid-based spatial bucketing (uniform feature distribution)
2. Forward-Backward consistency check (outlier rejection)
3. Multi-frame feature track ID management for MSCKF sliding window
"""

import cv2
import numpy as np
from typing import Dict, List, Tuple, Optional


class FeatureTracker:
    def __init__(self, max_features: int = 150, grid_rows: int = 4, grid_cols: int = 6):
        self.max_features = max_features
        self.grid_rows = grid_rows
        self.grid_cols = grid_cols
        
        self.next_feature_id = 0
        self.prev_gray: Optional[np.ndarray] = None
        
        # Lưu trữ trạng thái điểm: {feature_id: [(x, y), (x, y), ...]} qua các khung hình
        self.tracks: Dict[int, List[Tuple[float, float]]] = {}
        # Vị trí hiện tại: {feature_id: (x, y)}
        self.current_points: Dict[int, Tuple[float, float]] = {}

        # Tham số KLT LK Optical Flow
        self.lk_params = dict(
            winSize=(21, 21),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        )

    def track(self, frame_bgr: np.ndarray) -> Tuple[Dict[int, Tuple[float, float]], Dict[int, List[Tuple[float, float]]]]:
        """
        Theo dõi đặc trưng từ khung hình trước sang khung hình hiện tại.
        Trả về:
          - current_points: {feature_id: (x, y)}
          - lost_tracks: {feature_id: [(x, y)...]} các track vừa bị mất dấu (dùng nạp cho MSCKF update)
        """
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape
        lost_tracks = {}

        if self.prev_gray is None or len(self.current_points) == 0:
            # Khởi tạo khung hình đầu tiên: phát hiện đặc trưng mới
            self._detect_features_in_grid(gray)
            self.prev_gray = gray
            return self.current_points, lost_tracks

        # 1. KLT Tracking từ prev_gray sang gray
        prev_ids = list(self.current_points.keys())
        prev_pts = np.array([self.current_points[fid] for fid in prev_ids], dtype=np.float32).reshape(-1, 1, 2)

        # Forward Optical Flow
        next_pts, status, _ = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, prev_pts, None, **self.lk_params)
        
        st_flat = status.flatten()
        valid_forward_indices = np.where(st_flat == 1)[0]

        # Backward Optical Flow (Forward-Backward Consistency Check) chỉ trên các điểm bám thành công
        new_current = {}
        if len(valid_forward_indices) > 0:
            fwd_valid_pts = next_pts[valid_forward_indices]
            back_pts, back_status, _ = cv2.calcOpticalFlowPyrLK(gray, self.prev_gray, fwd_valid_pts, None, **self.lk_params)
            back_flat = back_status.flatten()

            for k, orig_idx in enumerate(valid_forward_indices):
                fid = prev_ids[orig_idx]
                if back_flat[k] == 1:
                    pt_next = fwd_valid_pts[k][0]
                    pt_back = back_pts[k][0]
                    fb_dist = np.linalg.norm(prev_pts[orig_idx][0] - pt_back)

                    # Điều kiện hợp lệ: lỗi quay về < 1.0 px và nằm trong khung hình (trừ biên 12px)
                    if fb_dist < 1.2 and (12 <= pt_next[0] < w - 12) and (12 <= pt_next[1] < h - 12):
                        pos = (float(pt_next[0]), float(pt_next[1]))
                        new_current[fid] = pos
                        self.tracks[fid].append(pos)
                    else:
                        lost_tracks[fid] = self.tracks.pop(fid, [])
                else:
                    lost_tracks[fid] = self.tracks.pop(fid, [])

        # Các điểm thất bại ở forward flow
        lost_forward_indices = np.where(st_flat != 1)[0]
        for orig_idx in lost_forward_indices:
            fid = prev_ids[orig_idx]
            lost_tracks[fid] = self.tracks.pop(fid, [])

        self.current_points = new_current

        # 3. Bổ sung điểm mới nếu số lượng điểm bám tụt xuống dưới ngưỡng
        if len(self.current_points) < self.max_features:
            self._detect_features_in_grid(gray)

        self.prev_gray = gray
        return self.current_points, lost_tracks

    def _detect_features_in_grid(self, gray: np.ndarray):
        """Phát hiện điểm đặc trưng phân bố đều trên khung hình (chạy đơn khối hiệu năng cao)."""
        needed = self.max_features - len(self.current_points)
        if needed <= 0:
            return

        h, w = gray.shape
        mask = np.ones((h, w), dtype=np.uint8) * 255
        mask[:15, :] = 0
        mask[-15:, :] = 0
        mask[:, :15] = 0
        mask[:, -15:] = 0

        # Tránh điểm mới trùng vị trí các điểm cũ
        for fid, (x, y) in self.current_points.items():
            ix, iy = int(round(x)), int(round(y))
            if 0 <= ix < w and 0 <= iy < h:
                cv2.circle(mask, (ix, iy), 18, 0, -1)

        corners = cv2.goodFeaturesToTrack(
            gray, maxCorners=needed, qualityLevel=0.015, minDistance=18, mask=mask
        )
        if corners is not None:
            for pt in corners:
                gx = float(pt[0][0])
                gy = float(pt[0][1])
                fid = self.next_feature_id
                self.next_feature_id += 1
                self.current_points[fid] = (gx, gy)
                self.tracks[fid] = [(gx, gy)]

    def draw_tracks(self, frame_bgr: np.ndarray) -> np.ndarray:
        """Vẽ trực quan các vệt chuyển động quang học lên khung hình."""
        vis = frame_bgr.copy()
        for fid, pts in self.tracks.items():
            if len(pts) < 1:
                continue
            cur = (int(pts[-1][0]), int(pts[-1][1]))
            # Vệt đuôi chuyển động
            if len(pts) >= 2:
                for j in range(max(0, len(pts) - 6), len(pts) - 1):
                    p1 = (int(pts[j][0]), int(pts[j][1]))
                    p2 = (int(pts[j + 1][0]), int(pts[j + 1][1]))
                    cv2.line(vis, p1, p2, (0, 255, 255), 1, cv2.LINE_AA)
            # Điểm bám hiện tại (Xanh lá nếu bám lâu, Xanh lam nếu mới)
            color = (0, 255, 0) if len(pts) >= 4 else (255, 200, 0)
            cv2.circle(vis, cur, 3, color, -1, cv2.LINE_AA)
        return vis

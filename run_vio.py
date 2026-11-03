"""
Launcher script for ISILAB VIO System
Run this directly with: python run_vio.py
"""

import sys
import os

# Đảm bảo đường dẫn import chính xác
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from vio_system.main_vio import run_vio_pipeline
import argparse

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="ISILAB VIO System - RGB-D & MSCKF Runner")
    parser.add_argument("--mode", type=str, default="auto", choices=["auto", "opencv", "xtion"],
                        help="Camera mode: 'opencv' (UVC + Real-time AI Depth), 'xtion' (OpenNI2), 'auto' (default: auto)")
    parser.add_argument("--camera", type=int, default=0, help="Camera device index (default: 0)")
    parser.add_argument("--port", type=str, default=None, help="COM port (default: auto detect COM13)")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate (default: 115200)")
    parser.add_argument("--features", type=int, default=120, help="Feature count (default: 120)")
    parser.add_argument("--openni2-dir", type=str, default=None, help="Path to OpenNI2 Redist folder")
    parser.add_argument("--min-depth", type=float, default=0.5, help="Min valid depth in meters (default: 0.5)")
    parser.add_argument("--max-depth", type=float, default=3.5, help="Max valid depth in meters (default: 3.5)")
    parser.add_argument("--extrinsic-x", type=float, default=0.08, help="IMU offset along X-axis relative to Camera in meters (default: 0.08m = 8cm)")
    parser.add_argument("--extrinsic-y", type=float, default=0.0, help="IMU offset along Y-axis in meters (default: 0.0)")
    parser.add_argument("--extrinsic-z", type=float, default=0.0, help="IMU offset along Z-axis in meters (default: 0.0)")
    parser.add_argument("--voxel-size", type=float, default=0.04, help="Voxel size in meters for 3D map accumulation (default: 0.04m = 4cm)")
    parser.add_argument("--allow-webcam", action="store_true", default=True, help="Allow OpenCV camera mode (default: True)")
    args = parser.parse_args()

    run_vio_pipeline(camera_id=args.camera, port=args.port, baudrate=args.baud,
                     max_features=args.features, openni2_dir=args.openni2_dir,
                     min_depth=args.min_depth, max_depth=args.max_depth,
                     extrinsic_x=args.extrinsic_x, extrinsic_y=args.extrinsic_y, extrinsic_z=args.extrinsic_z,
                     voxel_size=args.voxel_size, allow_webcam=args.allow_webcam, mode=args.mode)

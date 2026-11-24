#!/usr/bin/env python3
"""
tools/plot_telemetry_runaway.py
Tự động đọc file CSV telemetry của UAV HITL / ESP32-Flight-Controller,
phân tích chuỗi dữ liệu Attitude (Roll/Pitch/Yaw) và Actuators (M1..M4),
phát hiện hiện tượng hồi tiếp dương (Runaway) và vẽ đồ thị phân tích trực quan.

Cách dùng:
    python tools/plot_telemetry_runaway.py [duong_dan_file.csv] [--start T_START] [--end T_END] [--output plot.png]
"""

import sys
import os
import argparse
import csv
import matplotlib.pyplot as plt


def find_csv_file(explicit_path=None):
    if explicit_path and os.path.isfile(explicit_path):
        return explicit_path

    # Tìm kiếm các file CSV phổ biến trong thư mục monitor
    candidates = [
        "tools/hitl_monitor_web/monitor_Web_resource/assets/IMU_Telemetry_Log_20261102_231104.csv",
    ]
    for c in candidates:
        if os.path.isfile(c):
            return c

    # Tìm file CSV bất kỳ trong thư mục hiện tại
    for f in os.listdir("."):
        if f.lower().endswith(".csv") and os.path.isfile(f):
            return f

    return None


def parse_telemetry_csv(csv_path, t_start=None, t_end=None):
    time_s = []
    rolls = []
    pitches = []
    yaws = []
    m1_list = []
    m2_list = []
    m3_list = []
    m4_list = []

    with open(csv_path, mode="r", encoding="utf-8-sig") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if not header:
            return None

        # Xây dựng bản đồ cột
        col_map = {name.strip(): idx for idx, name in enumerate(header)}

        # Tìm chỉ số cột thời gian
        t_col = col_map.get("Time_Elapsed_s") or col_map.get("time") or col_map.get("t") or 0
        r_col = col_map.get("Roll_deg") or col_map.get("roll") or col_map.get("Roll")
        p_col = col_map.get("Pitch_deg") or col_map.get("pitch") or col_map.get("Pitch")
        y_col = col_map.get("Yaw_deg") or col_map.get("yaw") or col_map.get("Yaw")

        m1_col = col_map.get("M1_rads") or col_map.get("M1") or col_map.get("m1")
        m2_col = col_map.get("M2_rads") or col_map.get("M2") or col_map.get("m2")
        m3_col = col_map.get("M3_rads") or col_map.get("M3") or col_map.get("m3")
        m4_col = col_map.get("M4_rads") or col_map.get("M4") or col_map.get("m4")

        row_idx = 1
        for row in reader:
            row_idx += 1
            if not row or len(row) < 4:
                continue
            try:
                t = float(row[t_col]) if t_col is not None else float(row_idx * 0.02)
                if t_start is not None and t < t_start:
                    continue
                if t_end is not None and t > t_end:
                    break

                r = float(row[r_col]) if r_col is not None else 0.0
                p = float(row[p_col]) if p_col is not None else 0.0
                y = float(row[y_col]) if y_col is not None else 0.0

                m1 = float(row[m1_col]) if m1_col is not None else 0.0
                m2 = float(row[m2_col]) if m2_col is not None else 0.0
                m3 = float(row[m3_col]) if m3_col is not None else 0.0
                m4 = float(row[m4_col]) if m4_col is not None else 0.0

                time_s.append(t)
                rolls.append(r)
                pitches.append(p)
                yaws.append(y)
                m1_list.append(m1)
                m2_list.append(m2)
                m3_list.append(m3)
                m4_list.append(m4)
            except (ValueError, IndexError):
                continue

    return {
        "time": time_s,
        "roll": rolls,
        "pitch": pitches,
        "yaw": yaws,
        "m1": m1_list,
        "m2": m2_list,
        "m3": m3_list,
        "m4": m4_list,
    }


def analyze_and_plot(data, output_png="pitch_runaway_analysis.png", title="Phân Tích Dữ Liệu Runaway Drone"):
    t = data["time"]
    if not t:
        print("[LỖI] Không có dữ liệu trong khoảng thời gian đã chọn.")
        return False

    pitch = data["pitch"]
    roll = data["roll"]
    yaw = data["yaw"]
    m1 = data["m1"]
    m2 = data["m2"]
    m3 = data["m3"]
    m4 = data["m4"]

    # Tìm điểm vượt ngưỡng failsafe 60 độ
    failsafe_idx = None
    for i, p in enumerate(pitch):
        if abs(p) >= 60.0:
            failsafe_idx = i
            break

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig.patch.set_facecolor("#0b1329")

    # Subplot 1: Attitude
    ax1.set_facecolor("#111c38")
    ax1.plot(t, roll, label="Roll (Nghiêng)", color="#38bdf8", lw=1.5)
    ax1.plot(t, pitch, label="Pitch (Chúc/Ngóc)", color="#f59e0b", lw=2.0)
    ax1.plot(t, yaw, label="Yaw (Xoay)", color="#a855f7", lw=1.2, alpha=0.8)

    ax1.axhline(60, color="#ef4444", linestyle="--", alpha=0.7, label="Ngưỡng Failsafe ±60°")
    ax1.axhline(-60, color="#ef4444", linestyle="--", alpha=0.7)
    ax1.axhline(0, color="#94a3b8", linestyle=":", alpha=0.5)

    if failsafe_idx is not None:
        ax1.axvline(t[failsafe_idx], color="#ef4444", lw=1.5, linestyle="-.")
        ax1.annotate(
            f"Failsafe Cắt Ga\nt={t[failsafe_idx]:.2f}s | Pitch={pitch[failsafe_idx]:.1f}°",
            xy=(t[failsafe_idx], pitch[failsafe_idx]),
            xytext=(t[failsafe_idx] - 0.8, -45),
            color="#ef4444",
            fontweight="bold",
            arrowprops=dict(facecolor="#ef4444", shrink=0.05, width=1.5, headwidth=6),
            bbox=dict(boxstyle="round,pad=0.3", fc="#1e293b", ec="#ef4444", lw=1),
        )

    ax1.set_ylabel("Góc Thái Độ (độ)", color="#f8fafc", fontsize=11, fontweight="bold")
    ax1.grid(True, color="#334155", linestyle="--", alpha=0.4)
    ax1.tick_params(colors="#cbd5e1")
    leg1 = ax1.legend(loc="upper left", facecolor="#1e293b", edgecolor="#475569")
    for text in leg1.get_texts():
        text.set_color("#f8fafc")
    ax1.set_title(title, color="#f8fafc", fontsize=13, fontweight="bold", pad=10)

    # Subplot 2: Motor Outputs
    ax2.set_facecolor("#111c38")
    ax2.plot(t, m1, label="M1 Front-Left (CCW)", color="#38bdf8", lw=1.5)
    ax2.plot(t, m4, label="M4 Front-Right (CW)", color="#34d399", lw=1.5)
    ax2.plot(t, m2, label="M2 Rear-Right (CCW)", color="#f43f5e", lw=1.5)
    ax2.plot(t, m3, label="M3 Rear-Left (CW)", color="#fb923c", lw=1.5)

    if failsafe_idx is not None:
        ax2.axvline(t[failsafe_idx], color="#ef4444", lw=1.5, linestyle="-.")

    ax2.set_xlabel("Thời gian (giây)", color="#f8fafc", fontsize=11, fontweight="bold")
    ax2.set_ylabel("Động cơ (PWM / Tốc độ)", color="#f8fafc", fontsize=11, fontweight="bold")
    ax2.grid(True, color="#334155", linestyle="--", alpha=0.4)
    ax2.tick_params(colors="#cbd5e1")
    leg2 = ax2.legend(loc="upper left", facecolor="#1e293b", edgecolor="#475569")
    for text in leg2.get_texts():
        text.set_color("#f8fafc")

    plt.tight_layout()
    plt.savefig(output_png, dpi=180, facecolor=fig.get_facecolor(), edgecolor="none")
    plt.close()
    print(f"[THÀNH CÔNG] Đã lưu đồ thị phân tích tại: {output_png}")
    return True


def main():
    parser = argparse.ArgumentParser(description="Phân tích telemetry runaway UAV HITL")
    parser.add_argument("csv_path", nargs="?", default=None, help="Đường dẫn file CSV log telemetry")
    parser.add_argument("--start", type=float, default=None, help="Thời gian bắt đầu (giây)")
    parser.add_argument("--end", type=float, default=None, help="Thời gian kết thúc (giây)")
    parser.add_argument("--out", default="pitch_runaway_analysis.png", help="Đường dẫn file ảnh đồ thị xuất ra")
    args = parser.parse_args()

    csv_file = find_csv_file(args.csv_path)
    if not csv_file:
        print("[LỖI] Không tìm thấy file CSV log telemetry. Vui lòng truyền đường dẫn file.")
        sys.exit(1)

    print(f"[*] Đang phân tích file: {csv_file}")
    data = parse_telemetry_csv(csv_file, t_start=args.start, t_end=args.end)
    if not data or not data["time"]:
        print("[LỖI] Dữ liệu rỗng hoặc không phân tích được.")
        sys.exit(1)

    print(f"[*] Tổng số mẫu đọc được: {len(data['time'])}")
    print(f"[*] Khoảng thời gian: {data['time'][0]:.2f}s -> {data['time'][-1]:.2f}s")
    analyze_and_plot(data, output_png=args.out)


if __name__ == "__main__":
    main()

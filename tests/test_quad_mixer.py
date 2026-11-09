#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UNIT TEST FOR QUAD-X MOTOR MIXER (TEST D - SECTION 8.4 & APPENDIX A1)
"""

import sys

def compute_direct_manual_mixer(t, r, p, y):
    base_pulse = 1050.0 + (t * 9.50)
    d_roll  = r * 1.8
    d_pitch = p * 1.8
    d_yaw   = y * 1.2

    pwm1 = base_pulse - d_roll - d_pitch + d_yaw  # M1 Front Right (CCW)
    pwm2 = base_pulse + d_roll + d_pitch + d_yaw  # M2 Rear Left   (CCW)
    pwm3 = base_pulse + d_roll - d_pitch - d_yaw  # M3 Front Left  (CW)
    pwm4 = base_pulse - d_roll + d_pitch - d_yaw  # M4 Rear Right  (CW)

    m1 = max(1050, min(2000, round(pwm1)))
    m2 = max(1050, min(2000, round(pwm2)))
    m3 = max(1050, min(2000, round(pwm3)))
    m4 = max(1050, min(2000, round(pwm4)))
    return m1, m2, m3, m4

def run_mixer_tests():
    print("=" * 70)
    print("RUNNING QUAD-X MOTOR MIXER UNIT TESTS (TEST D - SECTION 8.4)")
    print("=" * 70)

    # 1. Zero torque + Idle thrust (t=0, r=0, p=0, y=0)
    m1, m2, m3, m4 = compute_direct_manual_mixer(0, 0, 0, 0)
    assert (m1, m2, m3, m4) == (1050, 1050, 1050, 1050), f"Failed idle thrust: {(m1, m2, m3, m4)}"
    print("[PASS] Test 1: Zero torque + Idle thrust (t=0) => All motors 1050 us")

    # 2. Pure throttle 50% (t=50, r=0, p=0, y=0)
    m1, m2, m3, m4 = compute_direct_manual_mixer(50, 0, 0, 0)
    assert (m1, m2, m3, m4) == (1525, 1525, 1525, 1525), f"Failed mid thrust: {(m1, m2, m3, m4)}"
    print("[PASS] Test 2: Pure throttle 50% => All motors 1525 us")

    # 3. Pure Roll Right (+r = 10%, t=50)
    # Roll phai (+r): Left motors (M3, M2) tang, Right motors (M1, M4) giam
    m1, m2, m3, m4 = compute_direct_manual_mixer(50, 10, 0, 0)
    assert m3 > 1525 and m2 > 1525 and m1 < 1525 and m4 < 1525, "Failed Roll Right"
    print(f"[PASS] Test 3: Roll Right (+10%) => M2,M3 (Left)={m2},{m3}us > M1,M4 (Right)={m1},{m4}us")

    # 4. Pure Pitch Forward (+p = 10%, t=50)
    # Pitch forward (+p): Rear motors (M2, M4) tang, Front motors (M1, M3) giam
    m1, m2, m3, m4 = compute_direct_manual_mixer(50, 0, 10, 0)
    assert m2 > 1525 and m4 > 1525 and m1 < 1525 and m3 < 1525, "Failed Pitch Forward"
    print(f"[PASS] Test 4: Pitch Forward (+10%) => M2,M4 (Rear)={m2},{m4}us > M1,M3 (Front)={m1},{m3}us")

    # 5. Pure Yaw Right (+y = 22%, t=50) - APPENDIX A1 AUDIT
    # Yaw right (+y): CCW motors (M1, M2) tang, CW motors (M3, M4) giam
    m1, m2, m3, m4 = compute_direct_manual_mixer(50, 0, 0, 22)
    assert m1 > 1525 and m2 > 1525 and m3 < 1525 and m4 < 1525, "Failed Yaw Right differential"
    print(f"[PASS] Test 5 (Appendix A1): Yaw Right (+22%) => M1,M2 (CCW)={m1},{m2}us != M3,M4 (CW)={m3},{m4}us")

    # 6. Max throttle saturation (t=100%, +y=22%)
    m1, m2, m3, m4 = compute_direct_manual_mixer(100, 0, 0, 22)
    assert m1 == 2000 and m2 == 2000 and m3 <= 2000 and m4 <= 2000, "Failed max throttle clamp"
    print(f"[PASS] Test 6: Max throttle saturation => Clamped cleanly within [1050, 2000] us: {(m1, m2, m3, m4)}")

    print("-" * 70)
    print("ALL QUAD-X MOTOR MIXER TESTS PASSED!")
    print("=" * 70)
    return 0

if __name__ == "__main__":
    sys.exit(run_mixer_tests())


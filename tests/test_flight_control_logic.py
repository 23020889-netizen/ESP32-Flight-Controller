"""
Unit tests for ESP32 Flight Controller rebuild logic
Testing Section 8.A criteria from ESP32_FLIGHT_CONTROL_REBUILD_GUIDE.md:
1. Yaw shortest angular wrap
2. Attitude Outer loop (Angle -> Rate setpoint)
3. Rate PID (Inner loop, anti-windup, derivative filtering)
4. Mixer Quad-X Geometry & sign verification
5. Saturation / Desaturation preserving roll/pitch
6. Per-motor PWM mapping (Uniform 1000..2000 us for all 4 motors)
7. RC command parsing and boundary validation
"""

import math
import unittest

DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi


def wrap_angle_180(angle_deg: float) -> float:
    """Wraps angle error to [-180, 180] shortest path"""
    if math.isnan(angle_deg) or math.isinf(angle_deg):
        return 0.0
    while angle_deg > 180.0:
        angle_deg -= 360.0
    while angle_deg < -180.0:
        angle_deg += 360.0
    return angle_deg


class AttitudeController:
    def __init__(self, kp_roll=6.5, kp_pitch=6.5, kp_yaw=2.8):
        self.kp_roll = kp_roll
        self.kp_pitch = kp_pitch
        self.kp_yaw = kp_yaw
        self.max_roll_rate_rad = 220.0 * DEG_TO_RAD
        self.max_pitch_rate_rad = 220.0 * DEG_TO_RAD
        self.max_yaw_rate_rad = 150.0 * DEG_TO_RAD

    def update(self, target_euler, actual_euler, yaw_valid=True):
        err_roll_rad = (target_euler[0] - actual_euler[0]) * DEG_TO_RAD
        err_pitch_rad = (target_euler[1] - actual_euler[1]) * DEG_TO_RAD
        if yaw_valid:
            err_yaw_deg = wrap_angle_180(target_euler[2] - actual_euler[2])
            err_yaw_rad = err_yaw_deg * DEG_TO_RAD
        else:
            err_yaw_rad = 0.0

        p_sp = max(-self.max_roll_rate_rad, min(self.max_roll_rate_rad, err_roll_rad * self.kp_roll))
        q_sp = max(-self.max_pitch_rate_rad, min(self.max_pitch_rate_rad, err_pitch_rad * self.kp_pitch))
        r_sp = max(-self.max_yaw_rate_rad, min(self.max_yaw_rate_rad, err_yaw_rad * self.kp_yaw))
        return [p_sp, q_sp, r_sp]


class RatePID:
    def __init__(self, kp=0.15, ki=0.20, kd=0.003, i_max=0.30):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_max = i_max
        self.integral = 0.0
        self.last_rate_meas = 0.0
        self.first_run = True

    def reset(self):
        self.integral = 0.0
        self.last_rate_meas = 0.0
        self.first_run = True

    def update(self, rate_sp: float, rate_meas: float, dt: float, allow_integral: bool) -> float:
        if math.isnan(rate_meas) or math.isnan(rate_sp):
            return 0.0
        if dt <= 0.0 or dt > 0.1:
            dt = 0.005

        err = rate_sp - rate_meas
        p_out = self.kp * err

        if allow_integral:
            self.integral += self.ki * err * dt
            self.integral = max(-self.i_max, min(self.i_max, self.integral))
        else:
            self.integral = 0.0

        if self.first_run:
            self.last_rate_meas = rate_meas
            self.first_run = False
            d_out = 0.0
        else:
            # Derivative on measurement
            d_raw = -self.kd * ((rate_meas - self.last_rate_meas) / dt)
            self.last_rate_meas = rate_meas
            d_out = d_raw  # simplified without LPF for mathematical test

        torque = p_out + self.integral + d_out
        return max(-1.0, min(1.0, torque))


class QuadXMixer:
    """
    Quad-X Mixer conforming to section 3.4 & 6 of the guide:
    Motor 1: Front-Right (CCW) -> u[0] = T - tau_roll + tau_pitch + tau_yaw
    Motor 2: Rear-Left   (CCW) -> u[1] = T + tau_roll - tau_pitch + tau_yaw
    Motor 3: Front-Left  (CW)  -> u[2] = T + tau_roll + tau_pitch - tau_yaw
    Motor 4: Rear-Right  (CW)  -> u[3] = T - tau_roll - tau_pitch - tau_yaw
    """
    def mix(self, throttle: float, roll_torque: float, pitch_torque: float, yaw_torque: float):
        if throttle < 0.02:
            return [0.0, 0.0, 0.0, 0.0]

        # Authority limit based on throttle
        max_authority = max(0.06, min(0.35, throttle * 0.70))
        roll_torque = max(-max_authority, min(max_authority, roll_torque))
        pitch_torque = max(-max_authority, min(max_authority, pitch_torque))
        yaw_torque = max(-max_authority * 0.5, min(max_authority * 0.5, yaw_torque))

        u = [
            throttle - roll_torque + pitch_torque + yaw_torque,
            throttle + roll_torque - pitch_torque + yaw_torque,
            throttle + roll_torque + pitch_torque - yaw_torque,
            throttle - roll_torque - pitch_torque - yaw_torque,
        ]

        # Desaturation with priority for roll & pitch
        max_val = max(u)
        min_val = min(u)
        min_floor = 0.02
        span = max_val - min_val

        if span > (1.0 - min_floor):
            scale = (1.0 - min_floor) / span
            center = (max_val + min_val) * 0.5
            u = [center + (val - center) * scale for val in u]
            max_val = max(u)
            min_val = min(u)

        if max_val > 1.0:
            shift = max_val - 1.0
            u = [val - shift for val in u]
        elif min_val < min_floor:
            shift = min_floor - min_val
            u = [val + shift for val in u]

        return [max(0.0, min(1.0, val)) for val in u]


def map_motor_pwm(motor_id: int, u: float, min_us: int, max_us: int, spin_min_us: int) -> int:
    """Per-motor pulse mapping from normalized u [0.0 .. 1.0] to microseconds"""
    if u <= 0.001:
        return min_us
    pulse = spin_min_us + u * (max_us - spin_min_us)
    return int(round(pulse))


def parse_rc_command(cmd_str: str):
    """
    Parses RC command: RC,<thr>,<roll>,<pitch>,<yaw>,<arm>
    Returns (valid, thr, roll, pitch, yaw, arm)
    """
    cmd = cmd_str.strip()
    if not (cmd.startswith("RC,") or cmd.startswith("rc,")):
        return False, 0.0, 0.0, 0.0, 0.0, 0

    parts = cmd[3:].split(",")
    if len(parts) != 5:
        return False, 0.0, 0.0, 0.0, 0.0, 0

    try:
        thr = float(parts[0])
        roll = float(parts[1])
        pitch = float(parts[2])
        yaw = float(parts[3])
        arm = int(parts[4])
    except ValueError:
        return False, 0.0, 0.0, 0.0, 0.0, 0

    for val in [thr, roll, pitch, yaw]:
        if math.isnan(val) or math.isinf(val):
            return False, 0.0, 0.0, 0.0, 0.0, 0

    # Bounds validation
    if not (0.0 <= thr <= 100.0):
        return False, 0.0, 0.0, 0.0, 0.0, 0
    if not (-35.0 <= roll <= 35.0):
        return False, 0.0, 0.0, 0.0, 0.0, 0
    if not (-35.0 <= pitch <= 35.0):
        return False, 0.0, 0.0, 0.0, 0.0, 0
    if not (-180.0 <= yaw <= 180.0):
        return False, 0.0, 0.0, 0.0, 0.0, 0
    if arm not in [0, 1]:
        return False, 0.0, 0.0, 0.0, 0.0, 0

    return True, thr, roll, pitch, yaw, arm


class TestFlightControlRebuild(unittest.TestCase):
    def test_yaw_shortest_wrap(self):
        # Target 179, actual -179 -> diff is +358, wrap is -2 deg
        diff = 179.0 - (-179.0)
        self.assertAlmostEqual(wrap_angle_180(diff), -2.0, places=4)

        # Target -179, actual 179 -> diff is -358, wrap is +2 deg
        diff2 = -179.0 - 179.0
        self.assertAlmostEqual(wrap_angle_180(diff2), 2.0, places=4)

        # Target 10, actual -10 -> diff is +20 deg
        self.assertAlmostEqual(wrap_angle_180(20.0), 20.0, places=4)

        # Target 0, actual 360 -> wrap is 0
        self.assertAlmostEqual(wrap_angle_180(360.0), 0.0, places=4)

    def test_attitude_controller(self):
        att = AttitudeController()
        # Zero error -> zero rate setpoint
        rates = att.update([10.0, -5.0, 45.0], [10.0, -5.0, 45.0], yaw_valid=True)
        self.assertAlmostEqual(rates[0], 0.0, places=4)
        self.assertAlmostEqual(rates[1], 0.0, places=4)
        self.assertAlmostEqual(rates[2], 0.0, places=4)

        # Positive roll error -> positive roll rate setpoint
        rates = att.update([10.0, 0.0, 0.0], [0.0, 0.0, 0.0], yaw_valid=True)
        self.assertGreater(rates[0], 0.0)

        # Invalidate yaw -> yaw rate setpoint must be 0
        rates_noyaw = att.update([0.0, 0.0, 90.0], [0.0, 0.0, 0.0], yaw_valid=False)
        self.assertEqual(rates_noyaw[2], 0.0)

    def test_rate_pid(self):
        pid = RatePID(kp=0.15, ki=0.20, kd=0.003, i_max=0.30)
        # Error = 1.0 rad/s
        out = pid.update(rate_sp=1.0, rate_meas=0.0, dt=0.005, allow_integral=True)
        self.assertGreater(out, 0.0)

        # Anti-windup check
        for _ in range(500):
            pid.update(rate_sp=10.0, rate_meas=0.0, dt=0.005, allow_integral=True)
        self.assertLessEqual(pid.integral, 0.30)
        self.assertGreaterEqual(pid.integral, -0.30)

    def test_quad_x_mixer_geometry(self):
        mixer = QuadXMixer()
        thr = 0.50

        # Positive roll torque (lean right recovery -> right motors M1, M4 increase, left M2, M3 decrease)
        # Note: In negative feedback, if actual roll > target, error is negative.
        # Here we test the direct torque sign response:
        # u[0] = T - tau_r + tau_p + tau_y (M1: FR)
        # u[1] = T + tau_r - tau_p + tau_y (M2: RL)
        # u[2] = T + tau_r + tau_p - tau_y (M3: FL)
        # u[3] = T - tau_r - tau_p - tau_y (M4: RR)
        u_roll = mixer.mix(throttle=thr, roll_torque=0.10, pitch_torque=0.0, yaw_torque=0.0)
        self.assertGreater(u_roll[1], u_roll[0]) # M2 > M1
        self.assertGreater(u_roll[2], u_roll[3]) # M3 > M4

        # Positive pitch torque (M1, M3 front increase; M2, M4 rear decrease)
        u_pitch = mixer.mix(throttle=thr, roll_torque=0.0, pitch_torque=0.10, yaw_torque=0.0)
        self.assertGreater(u_pitch[0], u_pitch[1]) # M1 > M2
        self.assertGreater(u_pitch[2], u_pitch[3]) # M3 > M4

        # Positive yaw torque (CCW motors M1, M2 increase; CW motors M3, M4 decrease)
        u_yaw = mixer.mix(throttle=thr, roll_torque=0.0, pitch_torque=0.0, yaw_torque=0.10)
        self.assertGreater(u_yaw[0], u_yaw[2]) # M1 (CCW) > M3 (CW)
        self.assertGreater(u_yaw[1], u_yaw[3]) # M2 (CCW) > M4 (CW)

    def test_mixer_saturation(self):
        mixer = QuadXMixer()
        # Extreme throttle and torques
        u_sat = mixer.mix(throttle=0.95, roll_torque=0.50, pitch_torque=-0.50, yaw_torque=0.20)
        for val in u_sat:
            self.assertFalse(math.isnan(val))
            self.assertFalse(math.isinf(val))
            self.assertGreaterEqual(val, 0.0)
            self.assertLessEqual(val, 1.0)

    def test_per_motor_pwm_mapping(self):
        # All 4 Motors now use uniform standard ESC range (Min 1000, SpinMin 1000, Max 2000)
        for m_id in [1, 2, 3, 4]:
            pwm_idle = map_motor_pwm(m_id, 0.0, 1000, 2000, 1000)
            self.assertEqual(pwm_idle, 1000)
            pwm_half = map_motor_pwm(m_id, 0.5, 1000, 2000, 1000)
            self.assertEqual(pwm_half, 1500)
            pwm_full = map_motor_pwm(m_id, 1.0, 1000, 2000, 1000)
            self.assertEqual(pwm_full, 2000)

    def test_rc_command_parser(self):
        valid, thr, roll, pitch, yaw, arm = parse_rc_command("RC,50.0,5.0,-2.5,15.0,1")
        self.assertTrue(valid)
        self.assertEqual(thr, 50.0)
        self.assertEqual(roll, 5.0)
        self.assertEqual(pitch, -2.5)
        self.assertEqual(yaw, 15.0)
        self.assertEqual(arm, 1)

        # Reject out of bounds
        self.assertFalse(parse_rc_command("RC,150.0,0,0,0,1")[0]) # thr > 100
        self.assertFalse(parse_rc_command("RC,50.0,40.0,0,0,1")[0]) # roll > 35
        self.assertFalse(parse_rc_command("RC,50.0,0,0,0,3")[0]) # arm not 0 or 1
        self.assertFalse(parse_rc_command("RC,nan,0,0,0,1")[0]) # NaN
        self.assertFalse(parse_rc_command("INVALID_COMMAND")[0])

    def test_rc_zero_throttle_arming_safety_gate(self):
        """Verify safety gate: Reject ARM when throttle > 3.0%"""
        def simulate_arm_transition(current_state: str, thr: float, arm: int):
            if arm == 0:
                return "DISARMED", 0.0, True
            if current_state != "ARMED":
                if thr > 3.0:
                    return current_state, 0.0, False  # Rejected
                return "ARMED", 0.0, True  # Armed safely, throttle starts at 0
            return "ARMED", thr, True

        # Case 1: Arm with raised throttle (e.g. 50%) -> MUST BE REJECTED
        state, thr_sp, ok = simulate_arm_transition("DISARMED", 50.0, 1)
        self.assertFalse(ok)
        self.assertEqual(state, "DISARMED")

        # Case 2: Arm with zero throttle (0.0%) -> MUST BE ACCEPTED
        state, thr_sp, ok = simulate_arm_transition("DISARMED", 0.0, 1)
        self.assertTrue(ok)
        self.assertEqual(state, "ARMED")
        self.assertEqual(thr_sp, 0.0)

        # Case 3: Throttle increase after already ARMED -> MUST BE ACCEPTED
        state, thr_sp, ok = simulate_arm_transition("ARMED", 25.0, 1)
        self.assertTrue(ok)
        self.assertEqual(state, "ARMED")
        self.assertEqual(thr_sp, 25.0)

    def test_manual_throttle_setpoint_preservation_vs_rc_watchdog(self):
        """
        Verify that manual UI/CLI throttle setpoint is preserved and NOT wiped by RC watchdog.
        RC Watchdog only applies when active RC streaming is enabled.
        """
        class MockSetpointManager:
            def __init__(self):
                self.throttle_pct = 0.0
                self.rc_stream_active = False
                self.last_rc_ms = 0
                self.is_armed = True

            def set_manual_throttle(self, pct: float):
                self.throttle_pct = pct
                self.rc_stream_active = False

            def set_rc_stream_packet(self, pct: float, now_ms: int):
                self.throttle_pct = pct
                self.rc_stream_active = True
                self.last_rc_ms = now_ms

            def update_watchdog(self, now_ms: int, timeout_ms: int = 1500):
                if self.is_armed and self.rc_stream_active and (now_ms - self.last_rc_ms > timeout_ms):
                    self.throttle_pct = 0.0
                    self.rc_stream_active = False

        mgr = MockSetpointManager()

        # Step 1: User sets manual throttle from Web UI: 30%
        mgr.set_manual_throttle(30.0)
        self.assertEqual(mgr.throttle_pct, 30.0)
        self.assertFalse(mgr.rc_stream_active)

        # Step 2: 5 seconds pass (5000 ms > 1500 ms)
        mgr.update_watchdog(now_ms=5000)
        # MUST STAY AT 30.0% (strictly obeys setpoint)
        self.assertEqual(mgr.throttle_pct, 30.0)

        # Step 3: Now user switches to Gamepad streaming mode at 50%
        mgr.set_rc_stream_packet(50.0, now_ms=6000)
        self.assertEqual(mgr.throttle_pct, 50.0)
        self.assertTrue(mgr.rc_stream_active)

        # Step 4: Gamepad stream lost for > 1500 ms (now_ms = 8000)
        mgr.update_watchdog(now_ms=8000)
        # Failsafe MUST trigger and safely cut throttle to 0.0
        self.assertEqual(mgr.throttle_pct, 0.0)
        self.assertFalse(mgr.rc_stream_active)


if __name__ == "__main__":
    unittest.main()

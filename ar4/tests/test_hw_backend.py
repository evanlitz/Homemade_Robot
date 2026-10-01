"""backends/hw.py against a simulated Teensy. No hardware, no serial port.

The fake integrates the joints under the firmware's own speed and
acceleration rules, so the tracking tests measure the real control loop in
hw.py. What they cannot tell you is whether the offsets and signs match the
physical arm -- that is the bring-up list at the bottom of hw.py.
"""
import numpy as np
import pytest

from backends.hw import (EStopError, FW_MAX_ACCEL_DEG_S2, FW_MAX_SPEED_DEG_S,
                         HardwareError, HwBackend, LIMIT_MARGIN,
                         TrajectoryLimitError, parse_values, required_slowdown,
                         trajectory_demand)
from fake_teensy import FakeClock, FakeNano, FakeTeensy
from motion.trajectory import Trajectory, plan_line

# The chess demo's traverse height and a ~45 deg wrist yaw keep J5 well clear
# of zero; any regular configuration in the middle of the envelope will do.
Q_READY = np.array([0.0, 20.0, -10.0, 0.0, 40.0, 0.0])


def make(homed=True, gripper=True, **kw):
    clock = FakeClock()
    teensy = FakeTeensy(clock)
    nano = FakeNano() if gripper else None
    arm = HwBackend(transport=teensy, gripper_transport=nano,
                    clock=clock, sleep=clock.sleep, **kw)
    arm.connect()
    if homed:
        arm.calibrate()
    return arm, teensy, nano, clock


def smooth(a, b, duration, dt=0.02):
    """Joint-space smoothstep a -> b as a Trajectory."""
    n = int(round(duration / dt)) + 1
    s = np.linspace(0.0, 1.0, n)
    s = s * s * (3.0 - 2.0 * s)
    return Trajectory(np.asarray(a) + (np.asarray(b) - np.asarray(a)) * s[:, None], dt)


# --- frames and wire -------------------------------------------------------

def test_frame_round_trip():
    arm = HwBackend(transport=object())
    q = np.array([12.0, -30.0, 40.0, -150.0, 60.0, 170.0])
    assert np.allclose(arm.to_dh(arm.to_fw(q)), q)


def test_homed_park_reads_zero_in_dh():
    """Upstream parks the arm at URDF ~0 after homing. With the offsets and
    J1's sign applied that is DH zero on every joint."""
    arm, *_ = make()
    assert np.allclose(arm.get_joints(), 0.0, atol=1e-6)


def test_j1_is_reversed_and_the_rest_are_not():
    arm = HwBackend(transport=object())
    fw0 = arm.to_fw(np.zeros(6))
    step = arm.to_fw(np.full(6, 10.0)) - fw0
    assert np.allclose(step, [-10, 10, 10, 10, 10, 10])


def test_parse_values_rejects_missing_joint():
    with pytest.raises(HardwareError, match="missing joint"):
        parse_values("JPA1B2C3D4E5", "JP")


# --- handshake and safety interlocks -----------------------------------------

def test_handshake_sends_version_and_model():
    arm, teensy, nano, _ = make(homed=False)
    assert teensy.sent[0] == "STA2.1.0Bmk5\n"
    assert nano.sent[0] == "ST"
    assert arm.connected


def test_version_mismatch_fails_before_anything_moves():
    clock = FakeClock()
    teensy = FakeTeensy(clock, version="2.0.0")
    arm = HwBackend(transport=teensy, clock=clock, sleep=clock.sleep)
    with pytest.raises(HardwareError, match="firmware is 2.0.0"):
        arm.connect()
    assert len(teensy.sent) == 1


def test_mk3_uses_its_own_offsets():
    """The two models differ on J1's offset; homed-park must still read zero."""
    clock = FakeClock()
    arm = HwBackend(transport=FakeTeensy(clock, model="mk3"), model="mk3",
                    clock=clock, sleep=clock.sleep)
    arm.connect()
    arm.calibrate()
    assert np.allclose(arm.get_joints(), 0.0, atol=1e-6)
    assert arm.offsets[0] == 170.0


def test_unknown_model_fails():
    # no offsets for it here: refused before a port is touched
    with pytest.raises(ValueError, match="mk9"):
        HwBackend(transport=object(), model="mk9")
    # offsets supplied, but the firmware does not know it either -- e.g. an
    # mk5 sent to ycheng517's upstream sketch, which stops at mk3
    clock = FakeClock()
    arm = HwBackend(transport=FakeTeensy(clock), model="mk9",
                    offsets_deg=(0.0,) * 6, clock=clock, sleep=clock.sleep)
    with pytest.raises(HardwareError, match="model"):
        arm.connect()


def test_handshake_retries_a_lost_reply_and_only_ever_sends_st():
    class Deaf(FakeTeensy):
        """Misses the first two handshakes, as a Teensy still booting does."""
        missed = 0

        def write(self, data):
            if self.missed < 2:
                self.missed += 1
                self.sent.append(data.decode())
                return
            super().write(data)
    clock = FakeClock()
    teensy = Deaf(clock)
    arm = HwBackend(transport=teensy, clock=clock, sleep=clock.sleep)
    arm.connect()
    assert arm.connected
    assert teensy.sent == ["STA2.1.0Bmk5\n"] * 3


def test_error_state_says_to_reset_the_teensy():
    clock = FakeClock()
    teensy = FakeTeensy(clock)
    teensy.inject = ["ER: Unrecoverable error state entered. Please reset."]
    arm = HwBackend(transport=teensy, clock=clock, sleep=clock.sleep)
    with pytest.raises(HardwareError, match="reset button"):
        arm.connect()


def test_motion_refused_until_homed():
    arm, teensy, *_ = make(homed=False)
    with pytest.raises(HardwareError, match="not homed"):
        arm.move_joints(Q_READY)
    assert not any(s.startswith(("MT", "MV")) for s in teensy.sent)


def test_debug_and_warning_lines_are_skipped_and_kept():
    arm, teensy, *_ = make()
    teensy.inject = ["DB: joint A speed 70 > 60, clipping.", "WN: something"]
    arm.get_joints()
    assert arm.messages[-2:] == ["DB: joint A speed 70 > 60, clipping.",
                                 "WN: something"]


def test_firmware_error_raises():
    arm, teensy, *_ = make()
    teensy.inject = ["ER: panic, missing joint C"]
    with pytest.raises(HardwareError, match="panic"):
        arm.get_joints()


def test_silence_raises_instead_of_hanging():
    class Mute(FakeTeensy):
        def readline(self):
            return b""
    clock = FakeClock()
    arm = HwBackend(transport=Mute(clock), clock=clock, sleep=clock.sleep)
    with pytest.raises(HardwareError, match="timed out"):
        arm.connect()


def test_estop_stops_the_stream():
    arm, teensy, *_ = make()
    teensy.estop = True
    with pytest.raises(EStopError):
        arm.move_joints(Q_READY)


def test_out_of_limit_waypoint_refused_before_motion():
    arm, teensy, *_ = make()
    bad = Q_READY.copy()
    bad[1] = 95.0  # J2 max is 90
    n_before = len(teensy.sent)
    with pytest.raises(ValueError, match="J2"):
        arm.follow(smooth(Q_READY, bad, 20.0))
    assert not any(s.startswith(("MT", "MV")) for s in teensy.sent[n_before:])


# --- trajectory feasibility ---------------------------------------------------

def test_too_fast_trajectory_refused_with_the_right_slowdown():
    arm, teensy, *_ = make()
    arm.move_joints(Q_READY, speed=50)
    fast = smooth(Q_READY, Q_READY + [30, 0, 0, 0, 0, 0], 0.8)
    n_before = len(teensy.sent)
    with pytest.raises(TrajectoryLimitError) as info:
        arm.follow(fast)
    assert not any(s.startswith(("MT", "MV")) for s in teensy.sent[n_before:])

    k = info.value.slowdown
    v, a = trajectory_demand(fast.q, fast.dt * k)
    assert v <= FW_MAX_SPEED_DEG_S * LIMIT_MARGIN + 1e-9
    assert a <= FW_MAX_ACCEL_DEG_S2 * LIMIT_MARGIN + 1e-9
    # and it is the smallest such factor, not merely a safe one
    v, a = trajectory_demand(fast.q, fast.dt * k * 0.98)
    assert v > FW_MAX_SPEED_DEG_S * LIMIT_MARGIN or a > FW_MAX_ACCEL_DEG_S2 * LIMIT_MARGIN


def test_required_slowdown_is_one_for_a_slow_path():
    t = smooth(Q_READY, Q_READY + [5, 0, 0, 0, 0, 0], 5.0)
    assert required_slowdown(t.q, t.dt) == 1.0


# --- motion ---------------------------------------------------------------------

def test_move_joints_arrives_and_honours_speed():
    arm, _, _, clock = make()
    t0 = clock()
    out = arm.move_joints(Q_READY, speed=100)
    fast = clock() - t0
    assert np.allclose(out, Q_READY, atol=0.05)

    t0 = clock()
    arm.move_joints(np.zeros(6), speed=25)
    slow = clock() - t0
    # A 40 deg move at these limits is acceleration-bound, where duration goes
    # as 1/sqrt(speed): a quarter of the speed is twice the time, less the
    # fixed settle both moves share.
    assert slow > 1.7 * fast


def test_follow_tracks_a_cartesian_line():
    """The reason follow() streams velocity with feedback instead of a single
    MT: the tool has to stay on the planned line, not just arrive. Plan a real
    Cartesian line, slow it to fit, and check the measured joints stay with
    the plan throughout."""
    arm, *_ = make()
    arm.move_joints(Q_READY, speed=50)
    from motion.kinematics import fk
    start = fk(Q_READY)
    goal = start.copy()
    goal[:3, 3] += [0.0, 0.08, -0.04]
    traj = plan_line(Q_READY, goal)
    k = required_slowdown(traj.q, traj.dt)
    traj = Trajectory(traj.q, traj.dt * k)

    out = arm.follow(traj)
    assert np.allclose(out, traj.q[-1], atol=0.05)

    t, cmd, meas = arm.last_follow
    assert len(t) == len(traj.q) - 1
    worst = np.abs(cmd - meas).max()
    # 0.5 deg of joint error is ~3 mm at the tool at full reach; the fake has
    # no latency, so a real arm will need margin beyond this
    assert worst < 0.5, f"tracking error {worst:.3f} deg"


def test_follow_keeps_schedule():
    arm, _, _, clock = make()
    arm.move_joints(Q_READY, speed=50)
    traj = smooth(Q_READY, Q_READY + [10, -5, 5, 0, 0, 10], 4.0)
    t0 = clock()
    arm.follow(traj)
    # duration plus a short settle, not a multiple of it
    assert traj.duration <= clock() - t0 < traj.duration + 1.0


def test_exception_mid_stream_zeroes_velocity():
    arm, teensy, *_ = make()
    arm.move_joints(Q_READY, speed=50)
    traj = smooth(Q_READY, Q_READY + [20, 0, 0, 0, 0, 0], 4.0)
    calls = {"n": 0}
    real = arm._read_fw

    def flaky():
        calls["n"] += 1
        if calls["n"] == 50:
            raise HardwareError("cable pulled")
        return real()
    arm._read_fw = flaky
    with pytest.raises(HardwareError, match="cable"):
        arm.follow(traj)
    assert teensy.sent[-1] == "MVA0.0000B0.0000C0.0000D0.0000E0.0000F0.0000\n"


def test_stall_is_reported_not_swallowed():
    arm, teensy, *_ = make()
    arm.move_joints(Q_READY, speed=50)
    stuck = FakeTeensy._advance

    def jam(self):
        stuck(self)
        self.p[2] = min(self.p[2], arm.to_fw(Q_READY)[2] + 3.0)
    teensy._advance = jam.__get__(teensy)
    with pytest.raises(HardwareError, match="J3"):
        arm.move_joints(Q_READY + [0, 0, 10, 0, 0, 0], speed=50)


# --- gripper and lifecycle ------------------------------------------------------

def test_gripper_maps_fraction_to_servo_angle():
    arm, _, nano, _ = make()
    arm.set_gripper(1.0)
    assert nano.angle == 35
    arm.set_gripper(0.0)
    assert nano.angle == 0
    arm.set_gripper(0.5)
    assert nano.angle == 18


def test_gripper_without_port_raises():
    arm, *_ = make(gripper=False)
    with pytest.raises(HardwareError, match="gripper"):
        arm.set_gripper(1.0)


def test_disconnect_twice_is_safe():
    arm, teensy, *_ = make()
    arm.disconnect()
    arm.disconnect()
    assert teensy.closed and not arm.connected

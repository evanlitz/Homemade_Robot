"""MuJoCo backend. Same interface as the hardware, no robot required."""
import time

import numpy as np

try:
    import mujoco
    import mujoco.viewer
except ImportError:
    mujoco = None

from backends.base import Backend


# How far ahead of the current waypoint to place the setpoint when following a
# trajectory. The position servo lags a moving setpoint in proportion to joint
# speed -- ~0.037 deg per deg/s -- which costs 9.2 mm of wrist error on a
# full-speed chess move, well over the +/-5 mm budget. Leading by this much
# brings it to 0.9 mm at no cost in duration.
#
# MEASURED, not derived. kv/kp = 400/8000 predicts 50 ms, and 50 ms is 2.7x
# worse than this value: the model's armature and joint damping mean the servo
# is not a pure first-order lag. Re-measure by sweeping this against wrist
# error if the gains, armature, damping, or dt ever change.
SETPOINT_LEAD_S = 0.040

# Per-jaw travel of the SG1 gripper, metres. From the prismatic limits in
# ar_gripper_macro.xacro: each jaw runs 0 to 0.014, mimicked, so the opening
# changes by 28 mm between fully closed and fully open.
JAW_TRAVEL_M = 0.014


class SimBackend(Backend):
    def __init__(self, model_path="models/ar4.xml", render=True, realtime=True,
                 setpoint_lead_s=SETPOINT_LEAD_S):
        if mujoco is None:
            raise RuntimeError("mujoco is not installed")
        self.model_path = model_path
        self.render = render
        self.realtime = realtime
        self.setpoint_lead_s = float(setpoint_lead_s)
        self.m = self.d = self.viewer = None
        self._gripper = 0.0

    def connect(self):
        self.m = mujoco.MjModel.from_xml_path(self.model_path)
        self.d = mujoco.MjData(self.m)
        mujoco.mj_forward(self.m, self.d)
        if self.render:
            self.viewer = mujoco.viewer.launch_passive(self.m, self.d)

    def disconnect(self):
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None

    def get_joints(self):
        return list(np.rad2deg(self.d.qpos[:6]))

    def move_joints(self, angles_deg, speed=25):
        target = np.deg2rad(np.asarray(angles_deg, dtype=float))
        start = self.d.qpos[:6].copy()

        # speed is a percentage; map it to a traverse duration
        span = float(np.max(np.abs(target - start)))
        duration = max(0.15, np.rad2deg(span) / (180.0 * max(speed, 1) / 100.0))
        steps = max(1, int(duration / self.m.opt.timestep))

        for i in range(steps):
            frac = (i + 1) / steps
            # smoothstep gives a soft accel and decel
            eased = frac * frac * (3.0 - 2.0 * frac)
            self.d.ctrl[:6] = start + (target - start) * eased
            mujoco.mj_step(self.m, self.d)
            if self.viewer is not None:
                self.viewer.sync()
                if self.realtime:
                    time.sleep(self.m.opt.timestep)

        self._settle(target)
        return self.get_joints()

    def _settle(self, target_rad, max_steps=6000, vel_tol=1e-7):
        """Hold the setpoint until the arm stops moving, so both move_joints
        and follow can honour "blocks until the motion is complete".

        Settles on VELOCITY, not on reaching the setpoint. These are P+D
        position actuators with no integral term, so gravity leaves a
        steady-state offset of 3e-4 to 8e-4 rad -- several times larger than
        any useful position tolerance. Waiting on position never converges: it
        silently burned the full 6000-step budget on every single move.

        Measured on the real move_joints path (which ramps the setpoint first),
        this returns in ~410 steps instead of 6000. vel_tol is deliberately an
        order below the 1e-6 that test_follow_ends_at_rest asserts.
        """
        for i in range(max_steps):
            self.d.ctrl[:6] = target_rad
            mujoco.mj_step(self.m, self.d)
            # the guard on i stops it returning before the arm has started
            if i > 20 and np.max(np.abs(self.d.qvel[:6])) < vel_tol:
                break
        if self.viewer is not None:
            self.viewer.sync()

    def follow(self, trajectory):
        q = np.deg2rad(np.asarray(trajectory.q, dtype=float))
        n = len(q)
        if n == 0:
            return self.get_joints()
        if n == 1:
            return self.move_joints(np.rad2deg(q[0]))

        dt = float(trajectory.dt)
        steps = max(1, int(round(dt / self.m.opt.timestep)))
        lead = self.setpoint_lead_s / dt

        for i in range(n):
            # Interpolate rather than rounding to a whole waypoint: the lead is
            # a duration, and dt is the caller's choice, so it is generally
            # fractional.
            f = min(i + lead, n - 1.0)
            lo = int(np.floor(f))
            hi = min(lo + 1, n - 1)
            frac = f - lo
            self.d.ctrl[:6] = q[lo] * (1.0 - frac) + q[hi] * frac

            for _ in range(steps):
                mujoco.mj_step(self.m, self.d)
            if self.viewer is not None:
                self.viewer.sync()
                if self.realtime:
                    time.sleep(dt)

        self._settle(q[-1])
        return self.get_joints()

    def set_gripper(self, open_frac, max_steps=4000):
        """0.0 fully closed, 1.0 fully open. Blocks until the jaws stop.

        Settles on jaw velocity rather than on reaching the setpoint, because
        closing onto an object is a stall, not a failure: the jaws come to rest
        short of the target and that is the grasp. The real gripper detects the
        same condition with the ACS712 current sensor.
        """
        self._gripper = float(np.clip(open_frac, 0.0, 1.0))
        if self.d is None:
            return self._gripper

        self.d.ctrl[6:8] = self._gripper * JAW_TRAVEL_M
        for i in range(max_steps):
            mujoco.mj_step(self.m, self.d)
            if self.viewer is not None:
                self.viewer.sync()
                if self.realtime:
                    time.sleep(self.m.opt.timestep)
            # the guard on i stops it returning on the first step, when the
            # jaws have not started moving yet and velocity is still zero
            if i > 20 and np.abs(self.d.qvel[6:8]).max() < 1e-5:
                break
        return self._gripper
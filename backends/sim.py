"""MuJoCo backend. Same interface as the hardware, no robot required."""
import time

import numpy as np

try:
    import mujoco
    import mujoco.viewer
except ImportError:
    mujoco = None

from backends.base import Backend


class SimBackend(Backend):
    def __init__(self, model_path="models/ar4.xml", render=True, realtime=True):
        if mujoco is None:
            raise RuntimeError("mujoco is not installed")
        self.model_path = model_path
        self.render = render
        self.realtime = realtime
        self.m = self.d = self.viewer = None
        self._gripper = 1.0

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

        # let the controller settle onto the target
        for _ in range(6000):
            self.d.ctrl[:6] = target
            mujoco.mj_step(self.m, self.d)
            if np.max(np.abs(self.d.qpos[:6] - target)) < 1e-4:
                break
        if self.viewer is not None:
            self.viewer.sync()
        return self.get_joints()

    def set_gripper(self, open_frac):
        self._gripper = float(np.clip(open_frac, 0.0, 1.0))
        return self._gripper
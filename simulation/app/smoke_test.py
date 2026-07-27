"""Toolchain proof. Builds a two-link MJCF, steps it 100 times, prints angles.

Not a model of anything. Run: python -m app.smoke_test
"""

from __future__ import annotations

import mujoco
import numpy as np

# Two-link pendulum in the xz plane, both hinges about +y. Released horizontal
# and dropped under gravity, so the angles must move.
MJCF = """
<mujoco model="smoke_test_two_link">
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <worldbody>
    <body name="link1" pos="0 0 1">
      <joint name="j1" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0  0.3 0 0" size="0.02" density="1000"/>
      <body name="link2" pos="0.3 0 0">
        <joint name="j2" type="hinge" axis="0 1 0"/>
        <geom type="capsule" fromto="0 0 0  0.25 0 0" size="0.015" density="1000"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""

N_STEPS = 100


def main() -> int:
    model = mujoco.MjModel.from_xml_string(MJCF)
    data = mujoco.MjData(model)

    print(f"mujoco {mujoco.__version__}")
    print(f"nq={model.nq} nv={model.nv} nbody={model.nbody} timestep={model.opt.timestep}")
    print(f"initial theta (deg): {np.degrees(data.qpos)}")

    for _ in range(N_STEPS):
        mujoco.mj_step(model, data)

    theta = data.qpos.copy()
    print(f"stepped {N_STEPS} steps, t={data.time:.4f} s")
    print(f"final theta (rad): {theta}")
    print(f"final theta (deg): {np.degrees(theta)}")

    moved = np.abs(theta).max() > 1e-3
    finite = np.all(np.isfinite(theta))
    print(f"OK: moved={moved} finite={finite}")
    return 0 if (moved and finite) else 1


if __name__ == "__main__":
    raise SystemExit(main())

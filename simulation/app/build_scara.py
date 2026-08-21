"""Regenerate models/scara.xml and verify it. Candidate #8, D18.

The applicable subset of `app.build_palletizer`'s six checks. Two of those six
do not exist here and their absence is the point:

  - no loop residual: an open chain has no equality constraints
  - no tool-plate levelling: there is no levelling mechanism to check

What remains is what an open chain can still get wrong: joint count, spurious
contacts, FK against an independent oracle, and dynamic stability under gravity.

Run: python -m app.build_scara
"""

from __future__ import annotations

import math
from pathlib import Path

import mujoco
import numpy as np

from backends.sim_backend.scara_mjcf import (absolute_from_qpos, actuated_forces,
                                             build_mjcf, qpos_from_absolute)
from config import load_architecture
from motion.kinematics import forward_kinematics

OUT = Path("models/scara.xml")

# (q0, q1 absolute, lift) -- spread across the joint box, including a folded
# pose and a near-extended one.
POSES = [(0.0, 0.0, 0.10), (0.5, -0.4, 0.00), (-1.2, 0.9, 0.25),
         (2.0, 2.6, 0.05), (0.3, 2.9, 0.20)]


def oracle_tcp(cfg, q0: float, q1: float, d: float) -> np.ndarray:
    """Independent analytic FK, written from the geometry rather than reused
    from motion/. A check against the same code it is checking proves nothing."""
    l1, l2 = cfg.g("upper_arm_m"), cfg.g("forearm_m")
    return np.array([
        l1 * math.cos(q0) + l2 * math.cos(q1),
        l1 * math.sin(q0) + l2 * math.sin(q1),
        cfg.g("lift_home_m") + d - cfg.g("tool_drop_m"),
    ])


def main() -> int:
    cfg = load_architecture("scara")
    xml = build_mjcf(cfg)
    OUT.parent.mkdir(exist_ok=True)
    OUT.write_text(xml, encoding="utf-8")
    print(f"wrote {OUT}  ({len(xml.splitlines())} lines)")

    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    fails: list[str] = []

    # 1. DOF. Open chain: one per joint, no constraint rows to subtract.
    print(f"\n--- 1. degrees of freedom ---")
    print(f"  njnt={model.njnt}  nv={model.nv}  neq={model.neq}")
    if model.nv != 3:
        fails.append(f"nv={model.nv}, expected 3")
    if model.neq != 0:
        fails.append(f"neq={model.neq}, expected 0 for an open chain")

    # 2/3. Contacts and FK against the oracle.
    print(f"\n--- 2/3. contacts, and TCP against an independent oracle ---")
    print(f"{'q0':>7} {'q1':>7} {'lift':>6} {'ncon':>5} {'TCP err mm':>12} "
          f"{'round-trip':>11}")
    tcp = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tcp")
    for q0, q1, d in POSES:
        data.qpos[:] = qpos_from_absolute(model, q0, q1, d)
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)
        err = np.linalg.norm(data.site_xpos[tcp] - oracle_tcp(cfg, q0, q1, d))
        # motion/ FK must agree with the model too, not just with the oracle.
        trip = np.linalg.norm(
            np.array(absolute_from_qpos(model, data.qpos)) - np.array([q0, q1, d]))
        print(f"{math.degrees(q0):7.1f} {math.degrees(q1):7.1f} {d:6.3f} "
              f"{data.ncon:5d} {err * 1000:12.3e} {trip:11.2e}")
        if data.ncon:
            fails.append(f"ncon={data.ncon} at ({q0:.2f},{q1:.2f},{d:.2f})")
        if err > 1e-9:
            fails.append(f"TCP off by {err * 1000:.4f} mm at ({q0:.2f},{q1:.2f})")
        if trip > 1e-12:
            fails.append(f"qpos round-trip off by {trip:.2e}")
        motion_fk = forward_kinematics(cfg, np.array([q0, q1, d]))
        if np.linalg.norm(motion_fk - data.site_xpos[tcp]) > 1e-9:
            fails.append(f"motion/ FK disagrees with the model at ({q0:.2f},{q1:.2f})")

    # 4. Dynamic: hold each pose under gravity and confirm it stays finite.
    print(f"\n--- 4. dynamic hold under gravity ---")
    print(f"{'q0':>7} {'q1':>7} {'lift':>6} {'drift mm':>10} {'finite':>7} "
          f"{'tau0':>8} {'tau1':>8} {'lift N':>8}")
    for q0, q1, d in POSES:
        mujoco.mj_resetData(model, data)
        data.qpos[:] = qpos_from_absolute(model, q0, q1, d)
        for i in range(model.nu):
            data.ctrl[i] = data.qpos[model.jnt_qposadr[model.actuator_trnid[i, 0]]]
        # mj_forward BEFORE reading the site: site_xpos is only refreshed by a
        # forward pass, so capturing `start` straight after writing qpos reads
        # the PREVIOUS pose's position and reports a drift of hundreds of mm
        # that never happened.
        mujoco.mj_forward(model, data)
        start = data.site_xpos[tcp].copy()
        for _ in range(500):
            mujoco.mj_step(model, data)
        drift = float(np.linalg.norm(data.site_xpos[tcp] - start))
        finite = bool(np.all(np.isfinite(data.qpos)) and np.all(np.isfinite(data.qvel)))

        data.qvel[:] = 0.0
        data.qacc[:] = 0.0
        mujoco.mj_inverse(model, data)
        f = actuated_forces(model, data)
        print(f"{math.degrees(q0):7.1f} {math.degrees(q1):7.1f} {d:6.3f} "
              f"{drift * 1000:10.3f} {str(finite):>7} {f[0]:8.3f} {f[1]:8.3f} "
              f"{f[2]:8.1f}")
        if not finite:
            fails.append(f"non-finite state at ({q0:.2f},{q1:.2f},{d:.2f})")

    # 5. The structural claim this candidate is being modelled to test.
    print(f"\n--- 5. gravity torque on the revolute joints ---")
    print("  Both revolute axes are vertical, so gravity should contribute")
    print("  EXACTLY zero torque to them at every pose. This is the property")
    print("  that distinguishes candidate #8 and it is asserted, not assumed.")
    worst = 0.0
    for q0, q1, d in POSES:
        data.qpos[:] = qpos_from_absolute(model, q0, q1, d)
        data.qvel[:] = 0.0
        data.qacc[:] = 0.0
        mujoco.mj_inverse(model, data)
        worst = max(worst, float(np.abs(actuated_forces(model, data)[:2]).max()))
    print(f"  worst revolute gravity torque over {len(POSES)} poses: "
          f"{worst:.3e} N.m")
    if worst > 1e-9:
        fails.append(f"gravity torque {worst:.3e} N.m on a vertical axis")

    print(f"\nFAILURES: {'none' if not fails else ''}")
    for f in fails:
        print(f"  {f}")
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(main())

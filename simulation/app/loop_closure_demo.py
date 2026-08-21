"""Compare the two MJCF ways of closing a parallelogram loop.

connect  -- a real cut joint, restored by a positional equality constraint.
joint    -- an algebraic coupling between joint angles, no tie rod.

Sweeps the driven crank, and at each pose reports:
  loop residual        distance between the two cut sites (connect only --
                       jointeq has no pin, so its "residual" is the gap the
                       missing tie rod would have to span)
  coupler world angle  must stay constant; this is the palletizer property
  tool offset          tool position minus crank tip; constant iff the
                       parallelogram is actually holding
  static hold torque   generalised force at j_crank to hold the pose, from
                       inverse dynamics. This is the number the substitution
                       is allowed to change or not.

Run: python -m app.loop_closure_demo
"""

from __future__ import annotations

import math
from pathlib import Path

import mujoco
import numpy as np

MODELS = Path(__file__).resolve().parent.parent / "models" / "examples"
SWEEP_DEG = [-40.0, -20.0, 0.0, 20.0, 40.0]


def settle(model: mujoco.MjModel, data: mujoco.MjData, target: float, seconds: float = 1.5) -> None:
    data.ctrl[0] = target
    for _ in range(int(seconds / model.opt.timestep)):
        mujoco.mj_step(model, data)


def site_xpos(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> np.ndarray:
    return data.site_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)].copy()


def coupler_world_angle(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "coupler")
    r = data.xmat[bid].reshape(3, 3)
    return math.degrees(math.atan2(r[0, 2], r[2, 2]))


def equality_forces(data: mujoco.MjData) -> np.ndarray:
    """Equality rows only. efc also holds contact and limit rows; a stray
    contact between adjacent loop bodies lands in the same array and is easy
    to mistake for a tie-rod load. Filter, always."""
    mask = data.efc_type[: data.nefc] == mujoco.mjtConstraint.mjCNSTR_EQUALITY
    return data.efc_force[: data.nefc][mask]


def analytic_hold_torque(model: mujoco.MjModel, q: float) -> float:
    """Hand-derived static torque at j_crank, for cross-checking the solver.

    Virtual work on the single mechanism DOF. CoM heights, with s = sin(q):
      crank   0.60 + 0.25 s     rocker  0.80 + 0.25 s
      coupler 0.80 + 0.50 s - 0.175   (hangs off the rocker tip)
    dPE/dq = g cos(q) * (0.25 m_crank + 0.25 m_rocker + 0.50 m_coupler)
    """
    mass = {n: model.body_mass[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n)]
            for n in ("crank", "rocker", "coupler")}
    return -9.81 * math.cos(q) * (0.25 * mass["crank"] + 0.25 * mass["rocker"]
                                  + 0.50 * mass["coupler"])


def hold_torque(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    data.qvel[:] = 0.0
    data.qacc[:] = 0.0
    mujoco.mj_inverse(model, data)
    return data.qfrc_inverse.copy()


def run(path: Path) -> None:
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)

    screen_only = model.eq_type[0] != mujoco.mjtEq.mjEQ_CONNECT
    nominal = sum(_eq_rows(model, i) for i in range(model.neq))
    print(f"\n=== {path.name} ==="
          + ("   [SCREEN-ONLY -- kinematics valid, forces are NOT. DECISIONS.md D7]"
             if screen_only else ""))
    print(f"tree DOF={model.nv}  equality rows={nominal} (nominal; connect's y row is "
          f"redundant in a planar loop, so the true rank is 2 and the mechanism is 1 DOF)")
    f_eq_col = "|f_eq| N.m*" if screen_only else "|f_eq| N"
    print(f"{'crank':>7} {'residual_mm':>12} {'coupler_deg':>12} "
          f"{'tool_dx_mm':>11} {'tool_dz_mm':>11} {'tau_crank':>10} {f_eq_col:>12} {'tau_analytic':>12}")

    for deg in SWEEP_DEG:
        mujoco.mj_resetData(model, data)
        settle(model, data, math.radians(deg))
        mujoco.mj_forward(model, data)

        a = site_xpos(model, data, "cut_a")
        b = site_xpos(model, data, "cut_b")
        tool = site_xpos(model, data, "tool")
        residual_mm = float(np.linalg.norm(a - b)) * 1000.0
        offset = (tool - a) * 1000.0
        tau = hold_torque(model, data)
        f_eq = float(np.linalg.norm(equality_forces(data)))

        # F1: no intentional collision pairs in either model, so any contact is spurious.
        assert data.ncon == 0, f"{data.ncon} unexpected contacts -- see DECISIONS.md F1"

        print(f"{deg:7.1f} {residual_mm:12.4f} {coupler_world_angle(model, data):12.4f} "
              f"{offset[0]:11.3f} {offset[2]:11.3f} {tau[0]:10.3f} {f_eq:12.3f} "
              f"{analytic_hold_torque(model, math.radians(deg)):12.3f}")

    if screen_only:
        print("  * generalised torque, NOT a rod force. No tie rod exists in this model.")


def _eq_rows(model: mujoco.MjModel, i: int) -> int:
    return {mujoco.mjtEq.mjEQ_CONNECT: 3, mujoco.mjtEq.mjEQ_JOINT: 1,
            mujoco.mjtEq.mjEQ_WELD: 6}[mujoco.mjtEq(model.eq_type[i])]


if __name__ == "__main__":
    run(MODELS / "parallelogram_connect.xml")
    run(MODELS / "parallelogram_jointeq.xml")

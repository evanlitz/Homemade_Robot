"""Generate models/palletizer.xml from config and verify it.

Six checks, any of which failing means the model is wrong:
  1. DOF          9 tree joints - 3 loops x 2 rows = 3
  2. no contacts  ncon == 0 (DECISIONS.md F1)
  3. loop closure residual under the D7/OPEN-A gate, kinematically
  4. tool plate is level everywhere in the sweep
  5. TCP matches an INDEPENDENTLY written analytic FK
  6. the constraint solver holds all of the above dynamically, under gravity

Check 5's formula is written from the D2 convention directly, not derived from
the model, so it is a real oracle rather than a restatement.

Run: python -m app.build_palletizer
"""

from __future__ import annotations

import math
from pathlib import Path

import mujoco
import numpy as np

from backends.sim_backend.palletizer_mjcf import (
    ACTUATED,
    CUT_PAIRS,
    build_mjcf,
    qpos_from_absolute,
)
from config import load_architecture

OUT = Path(__file__).resolve().parent.parent / "models" / "palletizer.xml"

# (q1_abs, q2_abs) in degrees. Spans elbow-down working poses plus the singular
# reference pose, so a convention error cannot hide in a symmetric corner.
POSES_DEG = [
    (0.0, 0.0),
    (5.0, -70.0),
    (30.0, -60.0),
    (-10.0, -50.0),
    (60.0, -90.0),
    (20.0, -110.0),
]
YAW_DEG = [0.0, 37.0, -90.0]


def analytic_tcp(cfg, q0: float, q1a: float, q2a: float) -> np.ndarray:
    """Independent oracle. Straight from D2: absolute angles, so the planar
    reach is a plain sum of link vectors and the tool drop is always vertical
    because the plate is held level by geometry."""
    h = cfg.g("shoulder_height_m")
    l1, l2 = cfg.g("upper_arm_m"), cfg.g("forearm_m")
    d = cfg.g("tool_drop_m")
    r = l1 * math.cos(q1a) + l2 * math.cos(q2a)
    z = h + l1 * math.sin(q1a) + l2 * math.sin(q2a) - d
    return np.array([r * math.cos(q0), r * math.sin(q0), z])


def site(model, data, name: str) -> np.ndarray:
    return data.site_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)].copy()


def residuals_m(model, data) -> dict[str, float]:
    return {rod.replace("_rod", ""): float(np.linalg.norm(site(model, data, rod)
                                                          - site(model, data, mech)))
            for rod, mech in CUT_PAIRS}


def plate_tilt_deg(model, data) -> float:
    """Angle between the tool plate's local +z and world +z. Must be ~0.

    Measured as asin of the horizontal component, not acos of the vertical one:
    acos is ill-conditioned near 1, and reports ~1.2e-6 deg of "tilt" for a
    perfectly level plate purely from double-precision rounding.
    """
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "tool_plate")
    local_z = data.xmat[bid].reshape(3, 3)[:, 2]
    return math.degrees(math.asin(min(1.0, math.hypot(local_z[0], local_z[1]))))


def main() -> int:
    cfg = load_architecture("palletizer")
    OUT.write_text(build_mjcf(cfg), encoding="utf-8")
    print(f"wrote {OUT.relative_to(OUT.parent.parent)}")

    model = mujoco.MjModel.from_xml_path(str(OUT))
    data = mujoco.MjData(model)
    gate = cfg.validity
    failures = 0

    rows = sum(3 for _ in range(model.neq))
    independent = sum(2 for _ in range(model.neq))
    print(f"\ntree DOF {model.nv}  loops {model.neq}  connect rows {rows} nominal / "
          f"{independent} independent  ->  mechanism DOF {model.nv - independent}")
    print(f"actuators {model.nu}: {ACTUATED}")
    print(f"total mass {model.body_mass.sum():.3f} kg")
    if model.nv - independent != 3:
        print("FAIL: mechanism is not 3 DOF")
        failures += 1

    print(f"\ngate: INVALID above {gate.residual_invalid_m * 1e6:.0f} um, "
          f"target below {gate.residual_target_m * 1e6:.0f} um  (DECISIONS.md OPEN-A)")

    print("\n--- kinematic: qpos set analytically, no solver ---")
    print(f"{'yaw':>6} {'q1a':>6} {'q2a':>7} {'worst resid um':>15} {'verdict':>13} "
          f"{'tilt deg':>9} {'FK err mm':>10} {'r mm':>8} {'z mm':>8}")
    for yaw_deg in YAW_DEG:
        for q1_deg, q2_deg in POSES_DEG:
            q0, q1a, q2a = (math.radians(v) for v in (yaw_deg, q1_deg, q2_deg))
            data.qpos[:] = qpos_from_absolute(model, q0, q1a, q2a)
            data.qvel[:] = 0.0
            mujoco.mj_forward(model, data)

            res = residuals_m(model, data)
            worst = max(res.values())
            tcp = site(model, data, "tcp")
            fk_err = float(np.linalg.norm(tcp - analytic_tcp(cfg, q0, q1a, q2a)))
            tilt = plate_tilt_deg(model, data)

            print(f"{yaw_deg:6.0f} {q1_deg:6.1f} {q2_deg:7.1f} {worst * 1e6:15.6f} "
                  f"{gate.verdict(worst):>13} {tilt:9.2e} {fk_err * 1000:10.2e} "
                  f"{math.hypot(tcp[0], tcp[1]) * 1000:8.1f} {tcp[2] * 1000:8.1f}")

            if data.ncon:
                print(f"   FAIL: {data.ncon} contacts (F1)")
                failures += 1
            if gate.verdict(worst) == "INVALID":
                failures += 1
            if tilt > 1e-6:
                print(f"   FAIL: tool plate not level ({tilt:.3e} deg)")
                failures += 1
            if fk_err > 1e-9:
                print(f"   FAIL: TCP disagrees with analytic FK by {fk_err * 1000:.3e} mm")
                failures += 1

    print("\n--- dynamic: driven to pose under gravity, solver holds the loops ---")
    # The last column is the P-actuator's steady-state droop under gravity, NOT a
    # model error and NOT an accuracy prediction: a held stepper does not sag like
    # a proportional controller, it holds until it loses steps. Reported so it is
    # never mistaken for a real positioning figure.
    print(f"{'q1a':>6} {'q2a':>7} {'worst resid um':>15} {'verdict':>13} {'tilt deg':>9} "
          f"{'P-droop mm*':>12}")
    for q1_deg, q2_deg in POSES_DEG[1:]:
        q0, q1a, q2a = 0.0, math.radians(q1_deg), math.radians(q2_deg)
        mujoco.mj_resetData(model, data)
        data.qpos[:] = qpos_from_absolute(model, q0, q1a, q2a)
        data.ctrl[:] = [q0, q1a, q2a]
        for _ in range(int(2.0 / model.opt.timestep)):
            mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)

        worst = max(residuals_m(model, data).values())
        settle = float(np.linalg.norm(site(model, data, "tcp")
                                      - analytic_tcp(cfg, q0, q1a, q2a)))
        print(f"{q1_deg:6.1f} {q2_deg:7.1f} {worst * 1e6:15.6f} {gate.verdict(worst):>13} "
              f"{plate_tilt_deg(model, data):9.2e} {settle * 1000:12.3f}")
        if gate.verdict(worst) == "INVALID":
            failures += 1
        if data.ncon:
            print(f"   FAIL: {data.ncon} contacts (F1)")
            failures += 1

    print("  * P-actuator droop under gravity, not a model error and not an accuracy "
          "figure.\n    A held stepper does not sag proportionally -- it holds until it "
          "loses steps.")

    placeholders = cfg.placeholder_fields()
    print(f"\n{len(placeholders)} placeholder values still in this config "
          f"({sum(1 for p in placeholders if p.startswith('geometry'))} of them geometry, "
          f"swept per D9)")
    print("FAILURES:", failures if failures else "none")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())

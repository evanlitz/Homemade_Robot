# Running this project

Every command below is run **from the `simulation/` directory**, not the repository root:

```bash
cd simulation
```

All scripts use `python -m`, which puts the current directory on `sys.path` — that is why
no `PYTHONPATH` is needed, and why running a script by file path (`python app/sweep.py`)
fails on `import config`.

---

## 1. Setup — once

MuJoCo has no wheel for Python 3.14. **Use Python 3.12.** If `python --version` reports
3.13 or newer, install 3.12 and use it explicitly.

### Windows (PowerShell)

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

Prefix every later command with `.\.venv\Scripts\python.exe`, or activate the venv once
with `.\.venv\Scripts\Activate.ps1` and then just use `python`.

### macOS / Linux

```bash
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

### Verify the toolchain before anything else

```bash
python -m app.smoke_test
```

Builds a trivial two-link MJCF, steps it 100 times, prints the final joint angles.
Exits non-zero if MuJoCo is broken. Expected tail:

```
final theta (deg): [ 44.574 -46.432 ]
OK: moved=True finite=True
```

---

## 2. Every command

Measured runtimes, not estimates. Everything is sub-second except the sweep.

| Command | Time | Exit code | What it does |
|---|---|---|---|
| `python -m app.smoke_test` | 0.2 s | 1 if MuJoCo fails | MuJoCo toolchain proof. Two-link MJCF, 100 steps. |
| `python -m app.show_config` | 0.1 s | always 0 | Prints a loaded architecture and every remaining placeholder. |
| `python -m app.show_config palletizer` | 0.1 s | always 0 | Same, for a named config in `config/`. |
| `python -m app.loop_closure_demo` | 0.2 s | always 0 | Compares `<connect>` vs `<joint>` loop closure on a reference parallelogram. |
| `python -m app.build_palletizer` | 0.3 s | 1 if any check fails | Regenerates `models/palletizer.xml` and runs six verification checks. |
| `python -m app.belt_check` | 0.1 s | **always 0** — see note | Belt tension budget, driver-current cap, standstill dissipation. |
| `python -m app.sweep` | 1.2 s | 1 if nothing survives | Geometry + board-placement sweep with hard rejection gates. Prints the trade curve. |
| `python -m pytest tests/ -q` | 0.3 s | standard pytest | FK/IK property tests. Samples joint space, not Cartesian space. |

**Do not wire `app.belt_check` into CI expecting a non-zero exit.** It currently prints
`GATE FAILS` and exits **0**, because the failure is OPEN-B — a deliberate, recorded
disagreement between the torque cap and the belt spec, awaiting a component decision. It
is a report, not a test. `app.build_palletizer` and `app.sweep` *are* tests and do exit
non-zero on failure.

### Run everything

```bash
python -m pytest tests/ -q && \
python -m app.smoke_test && \
python -m app.build_palletizer && \
python -m app.belt_check && \
python -m app.sweep
```

PowerShell has no `&&`; chain with `;` and check `$?`, or run them one at a time.

---

## 3. What each script produces

### `app.smoke_test`
Toolchain proof only. Models nothing real. Non-zero exit if the model fails to move or
produces non-finite state.

### `app.show_config [name]`
Loads `config/<name>.yaml` (default `placeholder_arm`) and prints links, joints,
per-joint resolution and torque ceiling, then **every value still tagged
`!placeholder`**. Placeholders are marked at the value in YAML (`length_m: !placeholder
0.35`) and the loader returns a `float` subclass, so they work in arithmetic but stay
findable via `cfg.placeholder_fields()`.

### `app.loop_closure_demo`
Two models, identical bodies, differing only in the equality block. Shows that actuator
torque agrees exactly between them (forced — equal configuration manifolds imply equal
generalised force) while the internal forces do not. `parallelogram_jointeq.xml` output
is labelled `SCREEN-ONLY`: kinematics valid, forces not.

### `app.build_palletizer`
Writes `models/palletizer.xml`, then verifies:

1. mechanism is 3 DOF (9 tree joints − 3 loops × 2 independent rows)
2. `ncon == 0` — no spurious self-contacts
3. loop residual inside the validity gate
4. tool plate level everywhere
5. TCP matches an independently written analytic FK oracle
6. the solver holds all of the above dynamically, under gravity

Prints `FAILURES: none` and exits 0 when the model is good. Kinematic residual should be
`0.000000 um`; dynamic residual `0.007–0.147 um`.

The `P-droop mm` column is the position actuator's steady-state sag under gravity. It is
**not** a model error and **not** an accuracy prediction — a held stepper does not sag
proportionally, it holds until it loses steps.

### `app.belt_check`
Per-stage belt tension against the manufacturer allowable, the maximum joint torque each
stage can carry, the driver current that produces the configured torque cap, and the
standstill dissipation that implies. Currently reports `GATE FAILS` — see OPEN-B in
[DECISIONS.md](DECISIONS.md); the config is deliberately left inconsistent because
resolving it is a component decision.

### `app.sweep`
Sweeps link lengths and board placement jointly, evaluating coverage, worst-square
radius, resolution, peak joint torque (gravity + inertial), belt tension and gripper
clearance. Geometries violating a gate are **rejected, not ranked**. Prints a rejection
census, surviving geometries, and the trade curve.

Exits 1 if nothing survives all gates.

### `pytest`
Samples **joint space**, computes `p = FK(θ)`, asserts `IK(p)` recovers `θ`. Sampling
Cartesian space instead produces false failures on unreachable points where IK correctly
has no solution. Also checks the Jacobian against finite differences, and asserts the D2
property that planar Jacobian columns have norm exactly `L₁` and `L₂` — which fails under
the parent-relative convention, so it is a live check that D2 is in force.

---

## 4. Changing things

### Change geometry, limits or gates
Edit `config/palletizer.yaml`, then re-run `app.build_palletizer` and `app.sweep`.
Nothing is hardcoded in the model — `models/palletizer.xml` is **generated**, so editing
it directly is pointless; it is overwritten on the next build.

### Change sweep ranges
The `sweep:` block in `config/palletizer.yaml`. Keep it coarse until the trade curve has
been reviewed (D9).

### Add an architecture
1. `config/<name>.yaml` — same schema.
2. A kinematics class in `motion/architectures.py`, registered in `REGISTRY`.
3. An MJCF generator in `backends/sim_backend/`.

`motion/` must stay free of simulator and hardware imports. That is the point of the
layer split: it is the code that survives from simulation to hardware.

---

## 5. Troubleshooting

**`ModuleNotFoundError: No module named 'config'`** — you ran a script by path. Use
`python -m app.<name>` from the repo root.

**`No matching distribution found for mujoco`** — Python is newer than 3.12. Rebuild the
venv with `py -3.12` / `python3.12`.

**Model diverges, NaN in `qacc`** — check `armature` is present on the actuated joints.
It is reflected rotor inertia, `J_rotor × N²` = 0.169 kg·m² at 1:25, which is comparable
to the whole arm's inertia. Without it the model diverges on every integrator. Do **not**
respond by loosening `solref`/`solimp` — that produces a stable, healthy-looking run with
a 16 mm loop residual, 3270× over the validity gate. See F6 in
[DECISIONS.md](DECISIONS.md).

**Loop residual large, or huge forces appearing** — check `data.ncon == 0`. A closed loop
puts the cut bodies in permanent coincident contact by construction; with collision on,
MuJoCo generates contacts of hundreds of newtons at the cut point. Also never take a norm
over `data.efc_force` — filter by `data.efc_type == mjCNSTR_EQUALITY` first. See F1.

**Sweep rejects everything** — read the rejection census it prints. `coverage` means the
board placement is out of reach; `resolution` means the worst square is too far out for
the reduction ratio.

---

## 6. Repository

This directory is the `simulation/` component of
[Homemade_Robot](https://github.com/evanlitz/Homemade_Robot). The repository root is one
level up and holds the top-level `README.md` and `.gitignore`.

```
Homemade_Robot/
  README.md
  .gitignore
  simulation/        <- you are here
```

Normal workflow, run from the repository root:

```bash
git add -A
git commit -m "your message"
git push
```

`.gitignore` at the repository root excludes `.venv/`, `__pycache__/`, `.pytest_cache/`
and `MUJOCO_LOG.TXT` anywhere in the tree.

`models/palletizer.xml` **is** committed even though it is generated. It is the readable
artifact carrying the angle-convention header, and committing it makes geometry changes
diffable across commits — regenerate it with `python -m app.build_palletizer` after any
config change so the committed copy never drifts from the config that produced it.

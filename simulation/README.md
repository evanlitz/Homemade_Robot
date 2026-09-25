# Simulation

The simulation and motion-control component of
[Homemade_Robot](https://github.com/evanlitz/Homemade_Robot).

All commands in [RUNNING.md](RUNNING.md) are run from this directory, not the repository
root.

---


A simulation harness for choosing a robot arm architecture with numbers rather than
intuition, which then becomes the control stack for the real machine. Sim first, hardware
second, same motion code both times.

**Target task:** move chess pieces on a tournament board (57 mm squares, ~457 mm board).
~600 mm reach, ±5 mm at the tool point, trivial payload.

**Constraint that shapes everything:** six NEMA 34 steppers (8.4 N·m, 3.8 kg each) are
already owned, and every motor must be grounded — a 3.8 kg motor at the elbow spends its
own torque budget carrying itself. That rules out most serial arms and pushes the
candidate list towards closed kinematic chains, which **URDF cannot represent**. The
pipeline is MuJoCo MJCF with equality constraints throughout.

- **[RUNNING.md](RUNNING.md)** — setup and every command.
- **[DECISIONS.md](DECISIONS.md)** — every design decision with its reasoning, and every
  measured finding with the measurement that produced it. Append-only. Read this before
  changing anything; several entries record traps that produce plausible wrong answers.
- **[CLAUDE.MD](CLAUDE.MD)** — project brief and working agreement.

## The comparison — decided: SCARA+Z (D20)

Two candidates, same sweep box, same gates, same assumptions including the unvalidated
ones. `python -m app.sweep palletizer` against `python -m app.sweep scara`. Full detail
and caveats in F15; the decision and what it costs in D20.

|  | palletizer (#6) | SCARA+Z (#8) |
|---|---|---|
| surviving geometries | 7 | **35** (46 before F17's joint limits) |
| best worst-square resolution | 0.890 mm (17.8% of target) | **0.490 mm (9.8%)** |
| peak revolute joint torque | 18.8–24.7 N·m | **1.9–4.8 N·m** |
| of which gravity | 14.6–19.2 N·m | **exactly 0** |
| worst belt tension vs allowable | 36–47% | **4–9%** |
| Z axis motor torque | n/a | 1.03 N·m at 862 rpm, 32 mm lead ball screw (F18) — the OPEN-D-critical joint |

These share an unvalidated mass model and are a *relative* comparison, not absolute
numbers. Both will be superseded by bench data.

## Status

Phase 1. Architecture chosen: **SCARA+Z (D20)**. The palletizer is shelved, not deleted.
Next is the OPEN-D pull-out test on the single-joint rig — see [BENCH.md](BENCH.md).

| Area | State |
|---|---|
| MuJoCo toolchain | working, verified |
| Config + placeholder tracking | working |
| Palletizer model (candidate #6) | built, 6 verification checks pass |
| SCARA+Z model (candidate #8) | built, 4 verification checks pass |
| `motion/` FK, IK, Jacobian, limits | implemented, property-tested, both candidates |
| `motion/` trajectory, workspace | **stubs** — signatures and conventions only |
| `backends/hw_backend` | **empty** — single-joint rig parts on order |
| Geometry + placement sweep | working, gated, runs either candidate |
| Candidates #1, #5, #7 | **descoped**, not deferred — see D18 |
| Palletizer (#6) | **shelved** by D20 — still builds, sweeps and tests |
| OPEN-D bench target + verdict | `python -m app.pullout scara [curve.csv]` — see BENCH.md |

31 values in `config/palletizer.yaml` are still tagged `!placeholder`. They are marked at
the value in YAML and enumerated by `python -m app.show_config palletizer`. Nothing
silently invents a link length, ratio, or limit.

## The shelved candidate: palletizer (#6)

Kept because the comparison should stay reproducible, and because its loop machinery is
what any future closed-chain candidate would reuse. Candidate #6, parallelogram palletizer. 3 DOF: base yaw plus two grounded planar drives
on the rotating base. The tool is held level by geometry through **two chained levelling
parallelograms** — one is not enough, because the only constant-angle reference available
is the rotating base, so the elbow yoke must be levelled off the base before the tool
plate can be levelled off the yoke.

Three closed loops, three cut joints, three `<connect>` equality constraints. 9 tree
joints − 3 loops × 2 independent rows = 3 DOF.

## Layout

```
app/        demos, verification scripts, the sweep
motion/     FK, IK, limits, workspace, trajectories
            PURE PYTHON. No simulator imports. No hardware imports.
backends/   sim_backend (MuJoCo, MJCF generation)  |  hw_backend (step/dir, later)
models/     MJCF. palletizer.xml is GENERATED; examples/ are hand-written references
config/     link lengths, ratios, limits, gates — all tunable without touching code
tests/      property tests
```

The motion layer being backend-free is the point of the structure. It is the code that
survives from simulation to hardware.

## Conventions

- **SI internally.** Metres, radians, newton-metres. Conversion at display boundaries only.
- **Angles are ABSOLUTE**, measured from horizontal — not parent-relative. In these
  coordinates the palletizer's defining invariant reads `θ_tool = 0`, which cannot be
  forgotten and whose violation is visible on inspection. FK collapses to a plain sum of
  link vectors. See D2; the earlier mixed convention (θ1 absolute, θ2 parent-relative) is
  dead and recorded as such.
- **Tool pose is `(x, y, z)`.** No orientation. A 3-DOF palletizer cannot control tool
  yaw independently — it is `atan2(y, x)`.
- **Microstepping is smoothness, not resolution.** Incremental torque per microstep falls
  off as the sine of the microstep angle, so under load the rotor settles where the load
  balances. Resolution figures use full steps only.

## What the simulation will and will not tell you

**Will:** reachability, workspace coverage, singularities, self-collision, trajectory
smoothness, kinematic torque demand, cycle time.

**Will not:** belt backlash, part flex, real positioning accuracy under load. Those need a
physical single-joint test rig. A clean sim result does not settle a hardware question.

*(Motor standstill heat was on that list. It is now largely designed out rather than
deferred — see D10.)*

## Findings worth knowing before you touch the model

Full detail in [DECISIONS.md](DECISIONS.md). These are the ones that produce plausible
wrong answers rather than obvious failures:

- **F1 — closed loops generate spurious self-contacts at the cut.** The cut bodies are
  coincident *by construction*, so with collision enabled MuJoCo produced contacts of up
  to 1624 N on a mechanism weighing 7.8 N. Actuator torque was unaffected, so the headline
  number looked perfect while the internal loads were wrong by two orders of magnitude.
- **F2 — equality constraints are soft, and MuJoCo's defaults cost 7.8% of the accuracy
  budget** as pure modelling artefact.
- **F6 — reflected rotor inertia is a first-order term.** `J_rotor × N²` at 1:25 is
  0.169 kg·m², *larger than the link it drives*. Without it the model diverges on every
  integrator. The instinctive fix — softening the constraints — produces a stable,
  healthy-looking run with a 16 mm loop residual, 3270× over the validity gate.
- **F7 — real belt data, not estimates.** HTD-5M is rated 454 N per 25.4 mm. An earlier
  estimate here was optimistic by ~2×, which changed a component decision.
- **F8 — measured torque demand is 13.7–25.0 N·m**, well under the 30 N·m that had been
  assumed. Estimates were replaced by measurements as soon as the model could produce them.
- **F10 — GT3 and HTD do not interchange at 5 mm pitch.** They share the pitch, and so the
  pitch diameter and the whole packaging envelope, but not the tooth profile. The
  8M/14M drop-in upgrade is real and does *not* carry down to 5M. Same blank, different
  groove.
- **F11 — a sweep range can hide a missing constraint.** The old `board_radius_m` lower
  bound was acting as a base-clearance gate by accident. Extending the box downward, as the
  pinned optimum demanded, produced a "best" geometry with the board sitting on the base.
  Resolution is now the binding gate at 89% of budget, where D5 had recorded it as free.
- **F13 — the packaging envelope binds before the physics does.** Yaw reduction has a real
  inertia-matching optimum at `N* = √(J_eff/J_rotor) ≈ 85`, confirmed numerically. It is
  irrelevant: motor torque never passes 14% of the cap at any ratio, while the output
  pulley outgrows the base column above 1:25. Resolution then turns around at 1:40 because
  a bigger column pushes the board outward faster than a finer step pulls travel down.
- **F14 — a quoted figure was a comparison against the wrong baseline.** The "+30%" cited
  against SDP/SI's +57% is Gates' own GT3-vs-**GT2** number, not GT3-vs-HTD. Also found
  while checking: SDP/SI's table requires an extra derate below 1 inch of belt width that
  the model does not apply, and 25 mm is the widest standard 5MGT belt, so D12's 10% margin
  is a ceiling rather than a step.

## Method notes

- **Gates reject, they do not rank.** Coverage rewards exactly the direction that breaks
  the tension and resolution budgets — longer links, board pushed outward — so a sweep
  that reports violations post hoc returns an optimum that is invalid. See D11.
- **Geometry and board placement are swept, not chosen.** Fixing them by fiat would make
  the comparison between architectures depend on an arbitrary choice. See D8, D9.
- **Property tests sample joint space, not Cartesian space.** For a grid of joint angles
  within limits, compute `p = FK(θ)` and assert `IK(p)` recovers `θ`. Sampling `(x, y)`
  instead produces false failures on unreachable points where IK correctly has no solution.

## Open items

- **OPEN-D — BLOCKING.** Motor torque is modelled as speed-independent. It is not. Narrowed
  by D20 to a bench target: ≥ 0.43 N·m at 532 rpm on the revolute joints and **≥ 2.06 N·m
  at 862 rpm on Z** (2x margin, `python -m app.pullout scara`). Procedure in BENCH.md.
- **OPEN-E** — the yaw stack does not fit the 200 mm column above 1:25. Growing it to
  244 mm buys yaw 1:40 and takes the binding gate from 89% of budget to 56%. Blocked on
  OPEN-D, which the higher motor speed makes worse.
- **OPEN-SAFETY-1** — with SCARA it moves to the Z axis: the 32 mm ball screw is
  backdrivable, so a power cut drops the carriage. Needs a fail-safe brake (F18, D20).
  Does not block the sim; blocks powered hardware.

*(OPEN-B closed by D12, GT3-5M final stage. OPEN-C closed by D13, 200 mm column.)*

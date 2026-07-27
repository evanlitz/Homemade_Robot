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

## Status

Phase 1, in progress. One architecture modelled so far.

| Area | State |
|---|---|
| MuJoCo toolchain | working, verified |
| Config + placeholder tracking | working |
| Palletizer model (candidate #6) | built, 6 verification checks pass |
| `motion/` FK, IK, Jacobian, limits | implemented, property-tested |
| `motion/` trajectory, workspace | **stubs** — signatures and conventions only |
| `backends/hw_backend` | **empty** — no hardware bought |
| Geometry + placement sweep | working, gated |
| Other four candidates | not started (deliberately — see D1) |

29 values in `config/palletizer.yaml` are still tagged `!placeholder`. They are marked at
the value in YAML and enumerated by `python -m app.show_config palletizer`. Nothing
silently invents a link length, ratio, or limit.

## Architecture under evaluation

Candidate #6, parallelogram palletizer. 3 DOF: base yaw plus two grounded planar drives
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

- **OPEN-B** — the configured torque cap (48 N·m) and the configured belt (HTD-5M, 25 mm,
  33.7 N·m) disagree. Left deliberately inconsistent and loud; resolving it is a component
  decision.
- **OPEN-SAFETY-1** — belt reduction is backdrivable and there is no brake, so cutting
  motor power drops the arm. Does not block the sim; blocks powered hardware.
- The coarse sweep does not bracket the optimum — the best survivor sits at the minimum of
  two of four swept axes, so the ranges need extending before any refinement.

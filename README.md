# Homemade Robot

A robot arm built from scratch — designed, simulated, machined and controlled.

The immediate target task is moving chess pieces on a tournament board: 57 mm squares,
~457 mm board, ~600 mm reach, ±5 mm at the tool point.

## Repository layout

| Directory | Contents |
|---|---|
| [`simulation/`](simulation/) | MuJoCo simulation harness, motion layer (FK/IK/limits), architecture comparison. Python. |

## Where to start

- [`simulation/README.md`](simulation/README.md) — what the simulation does and its current state
- [`simulation/RUNNING.md`](simulation/RUNNING.md) — setup and every command
- [`simulation/DECISIONS.md`](simulation/DECISIONS.md) — every design decision with its reasoning,
  and every measured finding with the measurement that produced it

## Approach

The architecture is chosen with numbers, not intuition. Candidate arm designs are
modelled and compared on identical terms — workspace coverage, peak joint torque, and
tool resolution per full motor step — before anything is machined.

Six NEMA 34 steppers (8.4 N·m, 3.8 kg each) are already owned, and every motor must be
grounded, since a 3.8 kg motor at the elbow spends its own torque budget carrying itself.
That constraint pushes the candidate list towards closed kinematic chains, which URDF
cannot represent — so the pipeline is MuJoCo MJCF with equality constraints throughout.

Nothing else is bought yet. The simulation informs the purchase.

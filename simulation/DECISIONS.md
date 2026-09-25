# Decisions and findings

Append-only. Decisions are Evan's; findings are measured facts with the measurement
that produced them. Nothing here gets edited after the fact — if a decision changes,
add a new entry that supersedes the old one and mark the old one SUPERSEDED.

Numbering follows the decision list raised on 2026-07-27. Gaps are decisions still open.

---

## D1 — Model the parallelogram palletizer (#6) first, alone. SETTLED 2026-07-27

Build candidate #6 and nothing else until it is complete.

**Reasoning.** The alternative was to start with an open-chain candidate (#1 belt-serial
or #8 SCARA+Z), which would reach an end-to-end pipeline faster because neither needs
equality constraints or a passive-joint mapping. Rejected: three of the five candidates
are closed loops, so the `qpos` ↔ actuated-θ mapping in `backends/sim_backend` has to
exist regardless. Designing it against a case that does not need it means retrofitting
it later against three that do. Starting with the hardest loop case forces the backend
to be built correctly the first time.

**Consequence.** The open-chain candidates are cheap once the loop machinery exists.
The reverse is not true.

---

## D6 — Encoders mount on the joint output, after reduction. SETTLED 2026-07-27

Closed-loop control, with encoders on the joint output side, not the motor.

**Reasoning.** The 34HS1456 is single-shaft with no rear shaft, so rear-mounted encoder
kits are not an option — motor-side measurement would need a shaft-end or magnetic
solution competing with the drive pulley for the one shaft that exists. Joint-side
measurement is also strictly better information: it captures belt stretch, reduction
backlash and lost steps, all of which are invisible to a motor-side encoder. Cost is a
per-joint mounting design.

**Consequence for the sim.** Commanded and achieved position are separate quantities
and must be reported separately from the start, in both backends. A single "position"
field anywhere in the reporting layer is a bug. This is not deferrable — retrofitting
the distinction after the reporting layer exists means auditing every call site.

---

## D7 — Internal loads are required. Use `<connect>`. SETTLED 2026-07-27

Closed-loop models use `<equality><connect>`, never `<equality><joint>`.

**Reasoning.** Phase-1's stated deliverable is peak *joint* torque, and for that alone
the cheap `<joint>` angle-coupling would be sufficient — it is provably equivalent for
actuator torque (equal configuration manifolds imply equal generalised force; measured
agreement was 0.004 N·m across the sweep, see F3). But Evan is machining the aluminium
links and buying the bearings, so tie-rod tension and passive-pivot reactions are the
numbers that size real parts. `<joint>` equality has no tie rod, so those forces do not
exist in it at all. `<connect>` produces them as physical newtons.

**Validity gate.** The equality residual — the distance between the two cut sites — is
reported alongside every result. Any result whose residual is a meaningful fraction of
the ±5 mm accuracy budget is **invalid, not approximate**, and is discarded rather than
caveated. The threshold value is an open sub-decision, see OPEN-A.

**`parallelogram_jointeq.xml` is retained as a fast kinematic screen only.** Every
output derived from it is labelled SCREEN-ONLY wherever it appears. It may be used to
answer reachability and workspace questions. It may never be used to answer a force,
torque, or part-sizing question.

---

## D8 — Board placement is a swept parameter, not a constant. SETTLED 2026-07-27

Board position and yaw relative to the arm base go in config as swept variables.

**Reasoning.** Fixing the board pose by fiat would make workspace coverage — one of the
three comparison numbers — depend on an arbitrary choice, which makes the comparison
between architectures unfalsifiable. A candidate could lose only because its board was
badly placed. Sweeping the placement removes the arbitrariness and converts it into a
result.

**Consequence for the reported number.** Coverage becomes *best achievable coverage over
placements*, reported together with the placement that achieved it. The deliverable is a
**coverage map over placement**, not a scalar. A single coverage number, with no map and
no stated placement, is not a valid phase-1 output.

**Note.** This makes coverage a property of the (architecture, placement) pair. Two
architectures should be compared at each of their own optima, not at a shared placement.

---

## D2 — Angles are ABSOLUTE, measured from horizontal. SETTLED 2026-07-27

Every joint angle in `motion/` is the link's own angle from horizontal in the arm's
vertical plane, right-handed, world-referenced. Not parent-relative.

**Reasoning.** The palletizer's defining invariant is that the tool stays level. In
absolute coordinates that reads `θ_tool = 0` — a statement that cannot be forgotten and
whose violation is visible on inspection. In parent-relative coordinates the same
invariant reads `θ_tool = −θ_forearm`, a correction term that has to be remembered at
every site and whose sign error survives review. FK also collapses to a plain sum of
link vectors, `r = Σ Lᵢ cos θᵢ`, `z = h + Σ Lᵢ sin θᵢ`.

**Cost, and why it was acceptable.** A translation layer between `motion` and MuJoCo's
`qpos`. For a serial sub-chain that map is a constant lower-triangular matrix of ones —
`θ_abs = A · θ_rel` — invertible, architecture-independent in form, written once.

**Consequences already applied.**

- `motion/conventions.py` no longer marks the convention unratified.
- All planar hinges in generated MJCF use `axis="0 -1 0"`, not `0 1 0`. With `+y` a
  positive hinge rotation moves `+z` toward `+x`, so absolute angle measured as
  `atan2(z, x)` *decreases* — the cumulative-sum property fails and every angle picks
  up a sign flip. `-y` makes absolute angle the plain cumulative sum. This is a
  correctness requirement of D2, not a style choice.
- The absolute → `qpos` map lives in `backends/sim_backend/palletizer_mjcf.py`, which
  is where D2's translation layer belongs. `motion/` never sees `qpos`.

### D2-SUPERSEDED — mixed convention (θ1 absolute, θ2 parent-relative). DEAD

An earlier convention recorded elsewhere by Evan used **θ1 absolute, θ2 parent-relative**.
That convention is superseded by D2 and must not be reintroduced.

It is specifically dangerous because it is *almost* right: θ1 agrees with D2, so any
single-joint check passes and the disagreement only appears in two-link expressions —
where `θ2_abs = θ1 + θ2_mixed` differs from `θ2_abs = θ2_D2` by exactly θ1. At the home
pose θ1 ≈ 0 and the two agree, so the error is invisible near home and grows with
shoulder angle. Any document or note using it is stale.

## D3 — Tool pose is 3 elements: (x, y, z). SETTLED 2026-07-27

No orientation in the pose vector.

**Reasoning.** The palletizer has 3 DOF and holds the tool level by geometry, so tool
yaw is not independent — it is `atan2(y, x)`, determined by where the arm was sent.
Carrying a yaw field would mean a field holding derived data describing a motor that
does not exist, which is the failure mode where someone commands a yaw that silently
does nothing. Chess pieces have cylindrical bases, so neither a parallel-jaw gripper nor
an electromagnet needs approach yaw.

### D3-REQ — gripper/neighbour clearance must be TESTED, not assumed

D3 rests on yaw being unnecessary. That is a hypothesis, and it is to be tested rather
than protected. With yaw pinned to `atan2(y, x)` the gripper always approaches
**radially**, which may foul neighbouring pieces — worst near board edges where the
approach is most oblique relative to the rank/file grid.

Required: a gripper-vs-neighbour clearance check against a **densely populated** board,
not an empty one. Squares are 57 mm; pieces are ~30–35 mm across, leaving ~11–13 mm of
clearance per side before contact.

If it fails, that is evidence for a 4th motor and D3 gets revisited. **Evidence, not
precaution** — the 4th motor is not to be added pre-emptively.

## D4 — Two-stage belt reduction, 1:25. SETTLED 2026-07-27

**Reasoning.** Evan machines his own pulleys, so belt cost is materials and time rather
than purchase. Both actuated joints are grounded on the rotating base, so large pulleys
cost packaging volume and not moving mass — this is the architecture where belts are
least penalised. At 1:25 the resolution is 0.754 mm/full step at 600 mm, 15% of the ±5 mm
budget, leaving room for backlash and flex to consume some.

**Three stages rejected deliberately.** Three stages reach 1:100 and 0.19 mm/step — a
better number — but compounded belt backlash is precisely what F-series findings say the
sim *cannot* predict. The criterion applied was fewest unmodellable failure modes, not
best number. Recorded because the reasoning is not recoverable from the number alone.

**Packaging envelope** — see F5 for the full arithmetic, and F5 for a real problem the
1:25 ratio creates at the belt.

## D5 — ±5 mm must hold over the whole 457 mm board, scored on the worst square. SETTLED 2026-07-27

**Reasoning.** Captures use the whole board, and "used squares" is not well defined for a
general game.

**Conditional attached to this decision, per Evan's request to be told if it stops
holding.** The claim "this is free given D4" is true *only while the farthest reachable
board square sits at ≈600 mm*. Resolution is linear in radius: at 1:25 the joint moves
`0.031416 / 25 = 1.2566e-3` rad per full step, so tool error is `1.2566 mm per metre of
radius`.

```
worst-square radius     tool mm / full step     % of 5 mm budget
      600 mm                  0.754                  15%
      700 mm                  0.880                  18%
      800 mm                  1.005                  20%
      900 mm                  1.131                  23%
```

Since D8 sweeps board placement and D9 sweeps link lengths, the worst-square radius is a
**result, not a constant** — and a placement chosen to maximise coverage will tend to push
the board outward. **The trigger to re-open D4: any swept optimum whose worst reachable
board square exceeds ~800 mm radius**, at which point the demanding reading of D5 costs
20% of budget and 1:25 stops being comfortable. The sweep must report worst-square radius
alongside coverage so this is checkable rather than assumed.

## D9 — Link lengths are swept, jointly with board placement. SETTLED 2026-07-27

Palletizer link lengths are swept parameters, not chosen constants — same reasoning as
D8. Picking them by hand reintroduces exactly the arbitrariness D8 rejected, and they
move coverage, torque and resolution together rather than independently.

**Joint sweep with board placement**, not sequential. Sweeping lengths at a fixed board
pose and then placement at fixed lengths finds a corner of the space, not an optimum.

**Coarse first.** The deliverable is the *shape* of the coverage/torque/resolution
surface and the trade curve between them — not a fine grid. No fine sweep until the trade
curve has been reviewed.

## OPEN-A — RESOLVED 2026-07-27

Residual validity thresholds, settled:

- **Invalid above 50 µm** (1% of the ±5 mm budget). Results are discarded, not caveated.
- **Target below 5 µm** (0.1%).

Now plumbed as `validity.residual_invalid_m` / `validity.residual_target_m` in config.
Per F2 the current solver settings hold the residual at 0.043 µm, ~116× inside target, so
the gate fires only when something is genuinely wrong.

---

# FINDINGS

## F1 — Closed loops generate spurious self-contacts at the cut. 2026-07-27

**Symptom.** The first `parallelogram_connect.xml` had collision enabled (MuJoCo's
default). A closed loop places adjacent bodies in permanent contact — the coupler's
lower end is *coincident* with the crank tip **by construction**, since that is what the
`connect` constraint enforces. MuJoCo therefore generated a deeply-interpenetrating
contact exactly at the cut point and pushed the loop apart.

**Measured.** 3 spurious contacts on a mechanism weighing 7.82 N total:

```
row  0-2   mjCNSTR_EQUALITY          488.7, 0.0, -3.5  N   <- the real constraint
row  3-6   mjCNSTR_CONTACT_PYRAMIDAL      525.5 N  x4
row  7-10  mjCNSTR_CONTACT_PYRAMIDAL     1624.0 N  x4      <- 208x the mechanism weight
row 11-14  mjCNSTR_CONTACT_PYRAMIDAL      122.2 N  x4
```

Loop residual was 6.0 µm with the contacts present, 0.043 µm after the fix — a 140×
error in the number that D7 uses as its validity gate.

**Why it is dangerous.** Three compounding reasons:

1. It is silent. The sim runs, converges, and produces plausible-looking output.
2. `data.efc_force` mixes constraint types in one flat array. Taking `norm(efc_force)`
   returns contacts and equalities summed together. The first version of
   `loop_closure_demo.py` did exactly this and reported 3457 N as a "tie-rod load"
   on an 8 N mechanism.
3. The actuator torque was *unaffected* (2.4686 N·m, matching hand calculation to four
   digits) because the spurious contacts happened to produce no net generalised force on
   the driven DOF. So the headline phase-1 number looked perfect while the internal
   loads — the numbers D7 exists to produce — were wrong by two orders of magnitude.

**Fix, now standing practice.**

- Kinematic/inertial geoms in every closed-loop model carry `contype="0" conaffinity="0"`.
- Collision geometry, when needed, goes in a separate set with explicit masks that
  exclude adjacent links.
- Never take a norm over `data.efc_force`. Filter by `data.efc_type == mjCNSTR_EQUALITY`
  first. See `equality_forces()` in `app/loop_closure_demo.py`.
- Assert `data.ncon == 0` in any model that has no intentional collision pairs.

## F2 — Equality constraints are soft; defaults burn 8% of the accuracy budget. 2026-07-27

MuJoCo equality constraints are regularised springs, not exact. The residual is set by
`solref` (time constant) and `solimp` (impedance), and **MuJoCo's defaults are not good
enough for this project**.

Max loop residual over a ±40° crank sweep, contacts off, `parallelogram_connect.xml`:

```
solref timeconst (solimp tight)        solimp dmax (solref 0.02)
  0.001    0.000043 mm                   0.9      0.391557 mm
  0.002    0.000043 mm   <- in use       0.99     0.043538 mm
  0.005    0.000269 mm                   0.999    0.004284 mm
  0.01     0.001074 mm
  0.02     0.004284 mm                 MuJoCo defaults, no override
  0.05     0.024060 mm                   0.391557 mm  = 7.8% of the 5 mm budget
  0.1      0.055797 mm
  0.2     13.575639 mm   <- loses tracking entirely
```

**Reading.** Residual scales roughly as the *square* of the solref time constant
(0.005 → 0.01 → 0.02 gives 0.27 → 1.07 → 4.28 µm, i.e. ×4 per doubling), then falls off
a cliff between 0.1 and 0.2 where the constraint stops tracking at all. `solimp` dmax
buys roughly a factor of 9 per decade of `(1 - dmax)`.

**Consequence.** Left at defaults, the sim reports a 0.39 mm loop gap that is pure
modelling artefact — 7.8% of the ±5 mm accuracy target, silently consumed before any
real error source is modelled. The settings in `parallelogram_connect.xml` reduce this
to 0.043 µm. These are not tuning preferences; they are a correctness requirement, and
they belong in every closed-loop model.

**Caveat not yet investigated.** Whether tighter `solref` costs stability at the larger
timesteps a full trajectory run will want. Measured at `timestep=0.001` only.

## F3 — `<joint>` equality reproduces actuator torque exactly, internal forces not at all. 2026-07-27

Direct comparison, identical bodies and masses, only the equality block differing:

```
                          tau_crank    |f_eq|          tau_analytic (hand-derived)
connect   crank=  0.0      -2.469      3.495 N            -2.469
connect   crank= 40.0      -1.887      3.495 N            -1.891
jointeq   crank=  0.0      -2.469      1.747 N.m          -2.469
jointeq   crank= 40.0      -1.887      1.336 N.m          -1.891
```

Both reproduce the parallelogram property (coupler world angle constant, tool offset
pinned at −150.000 mm) and both match hand-derived virtual work to 0.004 N·m.

**Torque agreement is not evidence the substitution is safe** — it is forced by the fact
that equal configuration manifolds imply equal generalised force. The `|f_eq|` column is
where they part: `connect` reports 3.495 N of real tie-rod tension, cross-checked against
crank moment balance about pivot A as `(2.469 − 0.25 × 2.88) / 0.5 = 3.498 N`, agreeing
to 0.1%. `jointeq`'s 1.747 is an abstract generalised torque — different quantity,
different units, and there is no rod for it to be a load in.

This finding is the evidence base for D7.

## F4 — `<connect>` is rank-deficient on a planar loop. 2026-07-27

`<connect>` writes 3 scalar equations (x, y, z coincidence). A planar loop has only 2
independent ones; the out-of-plane row is redundant. MuJoCo's regularised solver
tolerates this and the demo converges, but it is the reason the residual is not
identically zero.

Not currently a problem. Recorded because if a redundant row ever does cause trouble,
the fix is to constrain the mechanism to its plane by construction — **not** to loosen
`solref`/`solimp`, which F2 shows would directly consume accuracy budget.

## F5 — 1:25 belt stack: packaging envelope, and the belt cannot pass stall torque. 2026-07-27

Requested under D4. Baseline HTD-5M (5 mm pitch), pitch diameter `d = N·p/π`.

**Ratio split.** An even 1:5 × 1:5 minimises the largest pulley. 1:4 × 1:6.25 needs a
125T at 199 mm; 1:5 × 1:5 needs a 100T at 159 mm. Use the even split.

```
                    teeth    pitch dia
motor pulley         20T      31.83 mm
stage-1 driven      100T     159.15 mm
stage-2 driver       20T      31.83 mm   (on intermediate shaft, coaxial with above)
stage-2 driven      100T     159.15 mm   (joint output)
```

**Envelope, in-line 3-shaft layout.** Minimum centre distance is
`(31.83 + 159.15)/2 = 95.5 mm` (pulleys touching); with flanges and belt clearance take
**130 mm** working centres for both stages.

```
length   15.9 + 130 + 130 + 79.6            =  355.5 mm
width    largest pitch dia + flanges         =  ~165 mm
depth    2 belt planes, 15 mm belt + flanges =  ~55 mm
```

Plus the NEMA 34 body itself: 86 × 86 × 116 mm, 3.8 kg, on the motor shaft axis.
Folding the layout (triangulating shaft 3 back over shaft 1 instead of in-line) trades
length for width — roughly 225 × 270 mm — and is worth trying in CAD if 355 mm is the
binding dimension.

**The real problem: the belt cannot carry what the reduction produces.**

Effective belt tension is `F = τ / r` at the driven pulley (`r = 79.58 mm` for the 100T).

```
                                      joint torque    effective belt tension
motor at full holding torque (8.4 N.m)
  stage 1, at intermediate shaft         39.9 N.m            501 N
  stage 2, at joint output              189.0 N.m           2375 N
expected operating load (~30 N.m est.)
  stage 2, at joint output                30.0 N.m            377 N
```

An HTD-5M belt at 15 mm width carries roughly 400–700 N working tension, 25 mm roughly
700–1200 N — **approximate, confirm against a specific belt datasheet before committing.**
Note also that `τ/r` is the *difference* between tight and slack side; peak tight-side
tension is higher once pretension is added, so these figures are optimistic.

So at 1:25 the reduction multiplies motor stall torque to ~2375 N of belt tension, several
times what any practical belt width carries. **The belt is the weakest link, not the
motor.** Three ways to respond, and this is Evan's call:

1. Limit drive current so motor torque never reaches the level that strips the belt.
2. Size stage 2 for stall — impractical, needs ~50 mm+ belt and pulleys to match.
3. Accept the belt as a deliberate mechanical fuse. Defensible for a machine of this
   class, but it should be a decision rather than an accident.

**Connection to phase 1.** Peak joint torque is already a phase-1 deliverable. Divide it
by 79.58 mm and it becomes the number that sizes belt width directly — so the sweep should
report belt tension alongside torque rather than leaving the conversion to be done by hand
later.

## F6 — Reflected rotor inertia is not optional. Omitting it diverges the sim. 2026-07-27

**Symptom.** The first palletizer build was kinematically exact — loop residual 0.000000 µm,
TCP matching analytic FK to 1e-13 mm — and diverged within 0.02–0.18 s the moment it was
simulated dynamically, with NaN in `qacc`.

**Wrong diagnosis, recorded because it was convincing.** A single-pose integrator sweep
said `implicitfast` and `implicit` diverged while `Euler` and `RK4` were fine, which reads
as an integrator problem. It is not. Re-running across four poses, **every** integrator
diverges — Euler and RK4 survived only the one pose they were tested at. A one-sample
diagnostic on a configuration-dependent failure is worthless.

**Actual cause.** Missing rotor inertia. A stepper's rotor has inertia, and through a
reduction it appears at the joint multiplied by the *square* of the ratio:

```
J_reflected = J_rotor * N^2 = 2.7e-4 * 25^2 = 0.169 kg.m^2
```

For scale: the upper arm alone (0.40 m aluminium capsule, r = 20 mm, 1.45 kg) contributes
roughly `(1/3)mL^2 = 0.077 kg.m^2` about the shoulder. **The reflected rotor inertia is
larger than the link it drives** and comparable to the whole arm's inertia. It is a
first-order term, not a correction.

Adding it to the three actuated joints as MJCF `armature` fixes the divergence completely,
and all four integrators then agree to the digit:

```
                                       worst residual   worst q1 drift
actuated armature = 0.169, implicitfast    0.0767 um       0.164 deg
actuated armature = 0.169, implicit        0.0767 um       0.164 deg
actuated armature = 0.169, Euler           0.0767 um       0.164 deg
actuated armature = 0.169, RK4             0.0767 um       0.164 deg
no armature, any integrator                DIVERGED
```

Armature goes on the **actuated joints only**. Passive loop joints carry no motor; putting
armature there would be a numerical fudge dressed as physics.

**The trap this sets.** The instinctive response to a diverging constrained model is to
soften the constraints. Measured, that "works" and destroys the result:

```
solref=0.01   stable, residual 16336 um    -> INVALID by OPEN-A, 3270x over the gate
solref=0.02   stable, residual  1422 um    -> INVALID by OPEN-A,  284x over the gate
```

Both look stable and both silently blow the entire ±5 mm accuracy budget several times
over. F2 predicted this; the OPEN-A gate is what catches it. **Divergence is a physics
question first and a solver question second.**

**Timestep bound.** With armature present, stable at 0.0005 / 0.001 / 0.002 s
(residual 0.077 / 0.077 / 0.307 µm); diverges at 0.004 s. Timestep must stay ≤ 0.002.

**CLOSED 2026-07-27.** `J_rotor = 2.7e-4 kg.m^2` confirmed from Longs 34HS1456 vendor
datasheets (2700 g·cm² = 2.7e-4 kg·m²). The placeholder was exactly right; it is now a
real value in config and no longer tagged `!placeholder`.

**Also corrected in the same pass:** phase resistance is **0.55 Ω**, superseding a 0.46 Ω
figure recorded elsewhere. 0.55 is the self-consistent value — standstill dissipation
`2 × 5.6² × 0.55 = 34.50 W` matches the stated 34.5 W per motor, where 0.46 Ω would give
28.85 W. The 0.46 figure is dead.

## D10 — Belt is NOT a fuse. Cap torque by driver current. SETTLED 2026-07-27

The "accept the belt as a mechanical fuse" option offered in F5 is **rejected**.

**Reasoning.** This is a vertical-plane arm and the shoulder belt holds it up against
gravity. A belt failure is an uncontrolled drop, not a graceful current limit. A fuse is
only a fuse if failing is safe, and here it is not.

> **FIGURE CORRECTED 2026-07-28.** This entry originally said "a 22 kg mechanism". That
> was whole-machine mass and it overstated the hazard. What actually falls is the moving
> link mass — **4.2 kg** in the model — driven by **19.2 N.m** of gravity torque at the
> shoulder. The grounded column, the rotating turret and two of the three motors do not
> fall. The argument is unchanged; the number was wrong in the alarming direction, and an
> overstated hazard gets discounted. Same correction applied to OPEN-SAFETY-1.

**Instead:** cap joint torque by driver current, sized to the actual requirement plus
margin, NOT to motor capability. Configured as `validity.joint_torque_cap_nm`.

At the requested 48 N.m cap, computed and now a config value rather than a note:

```
motor torque required        48 / (25 x 0.90)      = 2.1333 N.m
DRIVER CURRENT SETTING       5.6 x 2.1333 / 8.4    = 1.422 A/phase  (25.4% of rated)
standstill dissipation       2 x 1.422^2 x 0.55    = 2.22 W per motor, 6.67 W for three
```

Down from 34.50 W per motor / 103.5 W for three at rated current. **Capping the current
for the belt also removes the standstill-heat problem** — which CLAUDE.md lists as one of
the things simulation cannot tell us. It is now largely designed out rather than deferred
to a test rig. Torque is assumed linear in current, which is good at 25% of rated but is
an approximation and wants a bench check.

**Unresolved, see OPEN-B:** the configured belt cannot carry a 48 N.m cap. F7.

## D11 — Gates REJECT, they do not rank. SETTLED 2026-07-27

Belt tension and worst-square resolution are gated quantities, exactly like the loop
residual. The D8/D9 sweep discards geometries that violate a budget; it does not rank
them and leave the violation to be noticed.

**Reasoning.** Coverage rewards precisely the direction that breaks both budgets — longer
links and the board pushed outward. A sweep that reports violations post hoc will hand
back an optimum that is invalid, and the invalidity will be in a column rather than in
the verdict. Implemented as `ValiditySpec.rejections()`; the sweep prints survivors and a
rejection-reason census.

---

## OPEN-B — RESOLVED 2026-07-28 by D12. Original entry below.

Resolved by changing the final stage to GT3-5M rather than by moving the cap. See D12.

## OPEN-B — the torque cap and the belt spec disagree

D10 sets a 48 N.m cap. F7 measures the configured belt (HTD-5M, 25 mm, final stage) at
33.7 N.m. The config is deliberately left inconsistent and loud rather than silently
reconciled, because reconciling it is a component decision. Three ways out, F7 has the
numbers, and F8 changes which one looks sensible.

## OPEN-SAFETY-1 — a power cut drops this arm. Not to be solved now.

Belt reduction is backdrivable and there is no brake. Cutting motor power — e-stop, power
failure, a driver fault, a tripped breaker — releases the holding torque and the arm falls
under gravity.

> **FIGURE CORRECTED 2026-07-28.** This entry originally said "~22 kg of aluminium" falls.
> **It does not.** 22 kg is roughly the whole machine including the grounded column, the
> rotating turret, and the two base-mounted motors — none of which fall. The falling mass
> is the moving links only: **4.2 kg** in the model, held by **19.2 N.m** of gravity torque
> at the shoulder (measured, `alpha = 0` row of the sweep's acceleration sensitivity).
> A 4.2 kg arm swinging through ~0.7 m under 19.2 N.m is still a real hazard — enough to
> break a hand or destroy the gripper and whatever is under it — but it is not a 22 kg
> drop, and stating it as one invites the whole entry to be discounted. The three fixes
> below are unchanged.

Three families of fix, none chosen: a counterbalance (spring or gas strut at the
shoulder), a fail-safe brake (spring-applied, electrically released), or a
non-backdrivable final stage (worm or high-ratio cycloidal — but D4 rejected those
reductions for other reasons, so this would reopen D4).

**Does not block the sim.** It blocks powered hardware. Recorded here specifically so it
is not discovered after the machine is built. It also interacts with D4: any fix that
changes the final stage changes the resolution and torque numbers the sweep produces.

---

## F7 — Real HTD-5M belt data, and the 25 mm plan does not work. 2026-07-27

**Source.** SDP/SI *Handbook of Timing Belts, Pulleys, Chains and Sprockets*, Technical
Section: Table 3 "Allowable Working Tension of Different Belt Constructions", Section 9
(design guidelines), Section 13.3 (idler/mesh rules).
`https://sdp-si.com/D820/PDFS/Technical-Section.pdf`

```
Allowable Working Tension per 1 inch (25.4 mm) of belt width, neoprene / fibreglass cord
    HTD  3M    285 N        GT3  3M    507 N
    HTD  5M    454 N        GT3  5M    712 N
    HTD  8M    792 N        GT3  8M   1690 N
```

Two rules that come with it:

- **Teeth in mesh** (13.3): at least 6 on a load-carrying pulley. Below that, subtract
  20% of the rating per missing tooth, floor of 2. Our stages sit at **6.74** teeth in
  mesh at 130 mm centres, so no derating — but that is close. At 100 mm centres it drops
  to 5.6 and costs 20%. **Centre distance must stay >= ~110 mm**, which is a tighter
  constraint than the 95.5 mm geometric minimum in F5.
- **Design factor** (Section 9.1): these ratings already include a 1/15-of-ultimate
  design factor, so they are working values and should not be derated again.
- Section 9.3: pulley diameter must not be smaller than belt width. The 20T pulley is
  31.83 mm, so belt width above ~30 mm is not available without changing the ratio.

**My earlier 400-700 N figure for 15 mm was wrong — optimistic by roughly 2x.** The real
figure is 454 x 15/25.4 = **268 N**.

**The consequence: widening the final stage to 25 mm does not reach the 48 N.m cap.**

```
stage   profile  width   TIM   tension@cap  allowable   used   max joint torque
stage1  HTD-5M    15mm   6.74     134.0 N     268.1 N    50%       96.0 N.m
stage2  HTD-5M    25mm   6.74     635.8 N     446.9 N   142%       33.7 N.m   <-- binding
```

A 25 mm HTD-5M final stage supports **33.7 N.m**, not 45-50. The cap needs 42% more belt
capacity than the belt has. Options:

1. **GT3-5M, 25 mm** — same pitch, same pulleys, same packaging envelope, 712 N/inch
   instead of 454. Gives **52.9 N.m**, supporting the 48 N.m cap with 10% margin. Evan
   machines his own pulleys, so this is a tooth-profile change on parts he is cutting
   anyway. Cheapest fix by a wide margin.
2. **Wider HTD-5M** — blocked. 30 mm gives 42.7 N.m and is already at the
   pulley-diameter-vs-belt-width limit; 40 mm would need a larger small pulley, changing
   the ratio.
3. **Lower the cap to 33.7 N.m** — costs nothing in hardware, and F8 says it costs
   nothing in capability either.

## F8 — Measured torque demand is well below the estimate. 2026-07-27

The ~30 N.m "expected demand" used in F5 and D10 was my estimate, not a measurement.
The sweep now measures it properly: gravity hold plus the inertial term at the configured
acceleration limit, from `mj_inverse` on the full closed-loop model with real link masses.

```
peak joint torque over all full-coverage geometries:   13.7 - 25.0 N.m
corresponding stage-2 belt tension:                      182 - 331 N
against the HTD-5M 25 mm allowable of 447 N:            41% - 74% used
```

**The estimate was high by roughly 20-55%.** The belt is not actually overloaded by the
machine — it is overloaded by the *cap*. At real demand, the configured HTD-5M 25 mm
final stage has margin everywhere in the surviving design space.

This reframes OPEN-B. A cap set at 33.7 N.m (what the current belt carries) still sits
**35% above the worst measured demand of 25.0 N.m**, so option 3 in F7 costs no real
capability. Option 1 buys headroom for loads not yet modelled rather than loads measured.

**Caveat.** The inertial term uses the configured `max_accel_rad_s2`, itself a placeholder
at 4.0 rad/s^2. Torque demand is linear in that number, so raising the acceleration limit
raises peak torque proportionally. This is not a settled figure.

## F9 — D3-REQ clearance: the radial-approach hypothesis survives, but tightly. 2026-07-27

D3 rests on a radial-only gripper approach never fouling a neighbouring piece. Tested
against a fully populated board across every surviving geometry and placement:

```
minimum gripper-to-neighbour clearance:   +3.5 mm to +4.6 mm
```

**Positive everywhere — D3 holds, and no fourth motor is indicated.** That is the evidence
Evan asked for rather than the precaution he declined.

But 3.5 mm is thin, and it is computed from **placeholder** gripper dimensions
(44 mm jaw span, 6 mm jaws, 50 mm body). Clearance moves roughly 1:1 with jaw span and
body width, so a gripper 8 mm wider than the placeholder puts it negative and reopens D3.
The check is wired into the sweep and will re-run automatically once real gripper
geometry exists. It is not a settled result; it is a conditional pass on invented numbers.

---

## D12 — Final belt stage is GT3-5M, not HTD-5M. SETTLED 2026-07-28

Stage 2 changes profile from HTD-5M to GT3-5M at the same 5 mm pitch, same 20T/100T
pulleys, same 25 mm width, same 130 mm centres.

**Reasoning (Evan).** 712 N vs 454 N per 25.4 mm removes the binding constraint outright
rather than negotiating with it. The torque cap then gets set by what the machine needs,
not by what the belt survives — which is the whole point of D10. Option 3 in F7 (drop the
cap to 33.7 N.m) would have worked on today's measured demand, but it would have made the
belt the thing that decides the cap, and F8's demand figure is itself conditional on an
acceleration limit that was still a placeholder at the time.

```
stage   profile  width   TIM   tension@cap  allowable   used   max joint torque
stage1  HTD-5M    15mm   6.74     134.0 N     268.1 N    50%       96.0 N.m
stage2  GT3-5M    25mm   6.74     635.8 N     700.8 N    91%       52.9 N.m   <-- binding
```

**GATE PASSES: 52.9 N.m against a 48 N.m cap, 10% margin.** OPEN-B is closed.

Two things this decision carries with it, neither of them free:

- **10% is thin, and stage 2 runs at 91% of allowable at the cap.** It clears the gate and
  the gate is a hard one, but there is no room to raise the cap later without changing the
  belt again. Widening stage 2 to 30 mm would give 63.5 N.m — F7 notes 30 mm is already at
  the SDP/SI pulley-diameter-vs-belt-width limit for a 31.83 mm pulley, so that is the last
  available step at this ratio. Not taken; recorded so the ceiling is known.
- **The stack now mixes profiles** — HTD-5M on stage 1, GT3-5M on stage 2. Two belt part
  numbers and two groove toolpaths instead of one of each. Stage 1 at 96.0 N.m has enormous
  margin, so switching it too buys nothing structural; it would only buy a single-profile
  bill of materials. Left mixed deliberately.

## F10 — "same pulleys" was wrong. GT3 and HTD do not interchange at 5 mm pitch. 2026-07-28

F7 option 1 said GT3-5M was "same pitch, same pulleys, same packaging envelope". **The
pulley claim was false and Evan caught it.** Checked against sources before D12 was
recorded:

- HTD uses a **fully rounded, symmetrical curvilinear** tooth with relatively large
  tooth-to-groove clearance. GT3 uses a **modified curvilinear** tooth with an optimised
  radius and depth and a deliberately tighter fit in the groove.
- **At 3M and 5M pitch the two are NOT interchangeable.** Gates rates a 5MGT belt on an
  HTD-5M sprocket as *not recommended*; the 3M combination is *not rated at all*.
- The interchange that people remember is real but is **only at 8M and 14M**, where 8MGT
  and 14MGT belts are approved drop-in replacements on existing HTD sprockets. Carrying
  that memory down to 5M is precisely the mistake F7 made.

Sources: <https://texasbelting.com/pages/htd-vs-gt-timing-belts>,
<https://aimsindustrial.com.au/blogs/product-guides/synchronous-timing-belt-guide>.

**What is actually shared: the pitch, and therefore the pitch diameter `d = N*p/pi` and the
whole packaging envelope from F5.** The pulley *blank* is unchanged — same 31.83 mm and
159.15 mm pitch diameters, same 130 mm centres, same belt width. Only the groove profile
differs. Evan machines his own pulleys, so D12 costs a toolpath, not a part. On a bought
pulley it would have cost the part.

**Contradiction in the secondary sources, flagged not resolved.** One source says GT3
gives "up to 30% higher power ratings" than HTD at equal pitch and width; the other says
"approximately twice the power capacity". The SDP/SI tension table used by the model gives
454 -> 712 N, i.e. **+57%**, sitting between them. The model uses the SDP/SI figure because
it is an allowable *tension* and the calculation is a tension calculation — the two power
claims are not directly comparable to it and are not used. If the real number is nearer
+30%, the allowable falls to about 580 N and stage 2 carries roughly 43 N.m, which would
**fail** the 48 N.m cap. D12's 10% margin is therefore not robust to the source being
optimistic. Worth a Gates datasheet check before pulleys are cut.

`app.belt_check` no longer prints "same pulleys"; it prints "same pitch diameter and
envelope, DIFFERENT pulley groove".

## F11 — Extending the sweep box exposed a constraint that reach had been enforcing by accident. 2026-07-28

The first coarse sweep put its best survivor at the **minimum** of both `upper_arm_m` and
`board_radius_m`. That is a box placed wrong, not a grid too coarse, so the ranges were
extended downward rather than subdivided — spacing held at ~0.08 m on the link axes and
~0.11 m on radius, so it is still a coarse run.

```
                 was                    now
upper_arm_m      0.30 - 0.55, 4 steps   0.15 - 0.55, 6 steps
board_radius_m   0.30 - 0.70, 5 steps   0.15 - 0.70, 6 steps
                 240 evaluations        432 evaluations
```

**Result: 34 survive, and the best of them are unbuildable.** With the board pulled in
close, both resolution and torque improve monotonically, and nothing in the gate set stops
the board being placed on top of the machine. The top rows of the trade curve sit at
`board_radius_m = 0.150` with a **board-to-yaw-axis distance of 0.000 m** — the base column
runs from the table to the shoulder, so those placements have the board and the arm in the
same space.

The old 0.30 m lower bound had been hiding this. It was doing the job of a base-clearance
constraint by accident, and removing it revealed that **no such gate exists**.

Added to the sweep, both ungated and reported:

- `board_edge_distance()` — distance from the yaw axis to the board *outline*, not to the
  nearest square centre, and not filtered by reachability. Zero means the axis is inside
  the board.
- a `BASE-COLUMN CLEARANCE` table showing what each candidate column radius costs.
- a `BOX CHECK` block that names any axis whose survivors touch an edge, so this failure
  reports itself rather than waiting to be noticed.

```
 column radius  survivors  best res mm  best torque
         0.000         34        0.524          8.4
         0.100         16        0.758         15.0
         0.150          8        0.890         18.8
         0.200          7        0.890         18.8
         0.250          5        0.890         18.8
```

**The choice between a 150 mm and a 250 mm column is free.** Resolution and torque are
identical across that span; only the count of equivalent placements changes. That is a
cheap decision, and it is Evan's — see OPEN-C.

Two further findings fall out of the constrained set:

- **Resolution is a board-placement quantity, not a link-length quantity.** Tool travel per
  full step is `max(r, L1, L2) * step`, and the yaw joint dominates whenever the worst
  square is further out than either link — which it always is here. So every surviving
  geometry at a given placement reports the *same* resolution, and link lengths trade only
  coverage against torque. Checks by hand: 1.8 deg / 25 = 1.2566e-3 rad, x 0.708 m worst
  radius = **0.890 mm**, matching the sweep exactly.
- **The resolution gate is now the binding one, and it is nearly hard against the limit.**
  0.890 mm against a 1.000 mm budget, i.e. 89% used. D5 recorded that taking the demanding
  reading over the whole board "costs nothing" at 1:25. Once base clearance is enforced,
  **that stops being true** — Evan asked to be told if it did.

## OPEN-C — base-column radius. Needs a number.

The sweep has no gate preventing the board from being placed where the machine is. F11 has
the cost table; every value from 0.150 to 0.250 m costs the same in resolution and torque.
Not chosen here, and deliberately left ungated rather than gated on a guess. `BASE_GAP_M`
in `app.accel_budget` uses 0.15 only to keep unbuildable placements from setting the
acceleration spec — it is a filter for that one report, not a decision.

## F12 — Acceleration derived from cycle time, replacing the placeholder. 2026-07-28

`max_accel_rad_s2` was a placeholder at 4.0 and peak torque is affine in it, so it sized
the belts, the pulleys and the driver current from an invented number. `app.accel_budget`
derives it from a cycle-time requirement instead.

**Move model.** One chess move, rest to rest, five motion segments and two dwells:
descend over source, [close], ascend, traverse, descend over destination, [open], ascend.
Each segment is a coordinated joint-space move at a common acceleration limit with a
triangular velocity profile, so a segment whose limiting joint travels `d` takes
`2*sqrt(d/a)`. Summing and solving:

```
a = ( 2 * SUM_segments sqrt(d_seg) / T_motion )^2        T_motion = T - 2*dwell
```

A triangular profile is the fastest rest-to-rest move under an acceleration limit alone,
so **these are the lowest accelerations that meet the cycle time, not estimates.** Any real
profile — trapezoidal, S-curve — needs more.

Scored on the **worst legal move** (a rook crossing the board), over the 8 geometries that
clear a 150 mm base column. Yaw is wrapped, so the arm always takes the short way round.

```
cycle T   dwell   motion |  alpha worst  alpha median | peak joint vel   motor rpm
    2 s   0.50 s   1.50 s|   21.86         14.33      |   5.20 rad/s      1243
    4 s   0.50 s   3.50 s|    4.01          2.63      |   2.23 rad/s       533
    8 s   0.50 s   7.50 s|    0.87          0.57      |   1.04 rad/s       249

cycle T     alpha  peak torque   vs 48 N.m cap  stage2 tension  vs allowable
    2 s     21.86       49.2 N.m         102%          651 N          93%
    4 s      4.01       24.7 N.m          51%          327 N          47%
    8 s      0.87       20.4 N.m          42%          270 N          38%
```

Three things worth reading off this:

- **The 4.0 rad/s^2 placeholder was almost exactly a 4 s cycle.** A lucky guess, and it is
  now a derived consequence of a spec rather than a guess.
- **A 2 s cycle breaks the torque cap** — 49.2 N.m against 48, and stage 2 at 93% of
  allowable. It does not merely cost more; it invalidates D10 and D12 together.
- **The velocity limit binds before the acceleration limit does.** `max_vel_rad_s` is a
  placeholder at 1.5 rad/s and every cycle time above exceeds it, 2 s by 3.5x. Where it is
  exceeded the profile clips to a trapezoid and the true acceleration is *higher* than
  tabulated, so those rows are optimistic.

**Two placeholders this rests on, both new and both stated in config, not code:**
`gripper.travel_height_m` = 0.120 m (clears a ~95 mm tournament king by 25 mm) and
`gripper.actuation_s` = 0.250 s. The dwell is not a rounding term: at a 2 s cycle two
dwells are a quarter of the whole budget.

**Modelling gap this exposes, not yet addressed.** The sim treats motor torque as
independent of speed. It is not — a NEMA 34 of this class loses most of its torque by
1200 rpm. The 2 s row asks for 1243 rpm. This is mitigated but not removed by the profile
shape: on a triangular profile peak velocity occurs mid-segment where the acceleration
term is zero, so the high-torque instant and the high-speed instant are not the same
instant. The demanding case is gravity hold at speed, not peak torque at speed. Resolving
it properly needs a torque-speed curve for the 34HS1456 at the intended bus voltage, which
the project does not have. Same class of omission as F6.

`max_accel_rad_s2` stays a placeholder in config until a cycle time is chosen.

---

## D13 — Base column radius is 200 mm, and it is now a hard gate. SETTLED 2026-07-28

`validity.base_clearance_m = 0.200`. The board outline must clear the yaw axis by at least
this much, enforced by `ValiditySpec.rejections()` alongside torque, tension and resolution.
Closes OPEN-C.

**Reasoning (Evan).** F11 showed 150–250 mm costs the same in resolution and torque, so the
choice was free on those axes and was made on packaging instead: the yaw reduction stack has
to live inside the column, and packaging room is worth more than extra equivalent
placements. **Conditional on the stack fitting — see F13, where it does not, at any ratio
above 1:25.**

Effect on the sweep: 34 survivors become 7. `base` is now the second-largest rejection
reason after `coverage` (240 of 432 geometries).

## D14 — Cycle time is 4 s. Acceleration and velocity are derived from it. SETTLED 2026-07-28

```
max_accel_rad_s2 = 4.01     max_vel_rad_s = 2.23
```

Both promoted from `!placeholder` to real values in `config/palletizer.yaml`, with the
derivation named in the comment. F12 has the method.

**Reasoning (Evan).** 2 s is not a cost, it is a contradiction — it needs 49.2 N.m against a
48 N.m cap, breaking D10 and D12 together, and asks for 1243 motor rpm these motors cannot
deliver. 8 s is safe but chess does not need it.

**`max_vel_rad_s` = 2.23 rad/s is a REQUIREMENT, not a capability.** It is the peak the
triangular profile produces on the longest single segment at 4.01 rad/s^2 — the speed the
cycle time demands, derived rather than assumed. It replaces a 1.5 rad/s placeholder that
every row of F12 already violated, which is exactly the failure mode of leaving a limit
invented: it was quietly wrong in the unsafe direction and nothing checked it.

**Flagged as asked: 2.23 rad/s is 533 motor rpm at 1:25, and that is untrustworthy.** See
OPEN-D. The number is the honest output of the cycle-time spec; whether the motor can hold
torque there is a question this model cannot answer.

## D15 — Reduction is per joint, not uniform. SETTLED 2026-07-28

`q0_yaw` gets its own `drive:` block in config instead of sharing the planar anchor. The
schema already supported it — `build_mjcf` reads `reflected_inertia_kgm2` per joint, and
`tool_m_per_full_step` reads `joint_rad_per_full_step` per joint — so this is a config
change, not a code change.

**Reasoning (Evan).** F11 established that tool travel per full step is
`||J_column|| * step_joint`, that the planar columns have norm L1 and L2 while the yaw
column has norm r, and that every surviving placement puts the worst square further out
than either link. So the yaw joint alone sets the binding gate and raising the planar
ratios buys nothing against it. A uniform ratio was never a decision, only an inherited
default.

## F13 — Yaw ratio: the column, not the inertia term, is what stops it. 2026-07-28

`app.yaw_ratio` sweeps yaw reduction against the full placement grid, rebuilding the MJCF
per ratio so reflected rotor inertia `J_rotor * N^2` enters the torque properly (F6).

**The expected optimum is real and was found.** Motor torque is

```
tau_motor(N) = tau_joint / (N*eta) = (J_eff*alpha)/(N*eta) + (J_rotor*alpha*N)/eta
```

falling as 1/N and rising as N, minimised at the inertia-matching ratio
`N* = sqrt(J_eff/J_rotor)`. Backing `J_eff` out of the measured torque per row gives
1.95–1.98 kg.m^2 and `N* = 85–86`; the numeric minimum sits at 1:75–1:100 at 0.207 N.m.
Analytic and numeric agree to the grid spacing.

**It is not the constraint that binds.** Motor torque never exceeds 14% of the
driver-current cap at any ratio, and belt tension never exceeds 26% of allowable. Two other
things bind first:

```
 ratio   teeth  res mm  places  yaw tau   J_eff    N*  motor tau  vs cap  tension   D_out  fold R    rpm
    25 100/100   0.890       7      5.8   1.285    69      0.259     12%       77     159     188    532
    30 110/110   0.742      19      8.4   1.850    83      0.309     14%      101     175     209    644
    40 126/126   0.556      19      9.6   1.976    86      0.270     13%      101     201     244    845
    50 141/141   0.591      12     10.6   1.977    86      0.237     11%      100     224     276   1058
    60 155/155   0.591      12     11.8   1.967    85      0.218     10%      101     247     306   1279
    75 173/173   0.591       9     14.0   1.976    86      0.208     10%      107     275     345   1593
   100 200/200   0.691       2     18.6   1.946    85      0.207     10%      123     318     403   2129
   125 224/224   0.691       1     24.4   1.824    82      0.216     10%      144     357     454   2671
```

**1. The stack does not fit the column.** The final driven pulley is concentric with the yaw
axis and its pitch diameter is `N_stage * 5/pi`, so the envelope grows with the ratio. At
1:25 the folded envelope is 188 mm against D13's 200 mm — **12 mm of margin, and that is the
whole of it.** Every ratio above 1:25 needs a bigger column. `fold R` is the folded layout
(motor tucked back toward the axis); a collinear layout is 45–115 mm worse again. Which one
is achievable is a CAD question, not a simulation one.

**2. Resolution turns around at 1:40, and it turns around because of the column.** The
column houses the stack *and* sets how far out the board must sit, so they are not
independent — this is computed self-consistently, taking the base clearance as
`max(200 mm, the stack's own envelope)`. Gating on a fixed 200 mm would credit a ratio with
a gain it cannot physically have.

```
1:25   0.890 mm    board at R = 0.480, clearance 251 mm, column needs 188   fits
1:40   0.556 mm    board at R = 0.480, clearance 251 mm, column needs 244   fits behind the board
1:50   0.591 mm    column needs 276 > 251, so the board is pushed to R = 0.590 and resolution WORSENS
```

**1:40 is exactly the largest ratio whose column still fits behind the board placement the
design already wanted.** Above it, the board is pushed outward faster than the finer step
pulls the tool travel down.

**3. There is a floor at 0.524 mm that yaw ratio cannot reach.** Once the yaw term drops
below the planar terms, `max(L1, L2) * 1.2566e-3` takes over. At 1:40 the two are already
co-binding: the best geometry is L1 = 0.390, L2 = 0.417, and `0.417 * 1.2566e-3 = 0.524 mm`
against a yaw term of `0.708 * 7.854e-4 = 0.556 mm`. **Going past 1:40 on yaw alone is
wasted; it would require raising the planar ratios too, which reopens D4.**

What 1:40 would buy, if the column grows to 244 mm:

```
              resolution   vs 1.000 mm budget   surviving placements
 yaw 1:25       0.890 mm          89%                    7
 yaw 1:40       0.556 mm          56%                   19
```

Best geometry at 1:40: L1 = 0.390, L2 = 0.417, board at R = 0.480, yaw 0. The trade curve
moves substantially — the binding gate goes from 89% consumed to 56%.

## F14 — Belt data checked against Gates' own literature. The contradiction dissolves. 2026-07-28

F10 flagged that two secondary sources disagreed (+30% vs ~2x) with SDP/SI's +57%, and that
if the true figure were +30% the D12 cap would fail. Checked against primary documents.

**The SDP/SI figures are confirmed exactly.** Table 3, "Allowable Working Tension Per 1 Inch
of Belt Width", neoprene/fibreglass: HTD 5 mm = **102 lbf / 454 N**, GT3 5 mm =
**160 lbf / 712 N**. Ratio 712/454 = **+56.8%**. The numbers the gate uses are the numbers
in the source.

**The +30% was misapplied.** Gates' own datasheet says GT3 "transmits up to 30% more power
than previous generation belts **(PowerGrip GT2)**". That is GT3 vs GT2, not GT3 vs HTD. The
secondary source quoted it as a GT3-vs-HTD figure. **It does not undermine the +57%, and the
D12 margin is not at risk from it.** Recorded because a wrong number that has already been
argued about tends to come back.

**F10 confirmed from the primary source.** The same Gates datasheet states 5MGT is
"**Used on GT type pulleys**". No inference needed.

**Which source the gate uses:** `drive_train.rating_n_per_inch` in
`config/palletizer.yaml`, from SDP/SI Table 3. Gates does not publish an allowable-tension
figure in its public literature at all — it publishes power-rating tables per pitch, groove
count and rpm and directs designers to DesignFlex Pro. So the SDP/SI tension table is not a
second-best source here; it is the only one in the right units for this calculation.
gates.com returns HTTP 403 to automated fetching, so a Gates 5M rating table could not be
read directly.

**Two flaws found while verifying, neither previously recorded:**

- **The model's width scaling is optimistic below 1 inch and nothing derates it.** SDP/SI's
  own note under Table 3: *"For thinner belt widths, less than 1", the tension must be
  derated since the tension cords on the sides are not complete loops."* The model computes
  `rating * width_mm / 25.4 * mesh_derate` — a pure pro-rata. **Both stages are under 1
  inch**: stage 2 at 25 mm is 0.984", stage 1 at 15 mm is 0.591". The source does not
  quantify the derate, so it is not implemented; it is recorded because D12's margin is 10%
  and this error is in the direction that eats it.
- **25 mm is the widest standard 5MGT belt.** Gates lists 9, 15 and 25 mm. F7 option 2
  contemplated widening to 30 mm as a fallback; for GT3-5M **that fallback does not exist**.
  D12's 10% margin is the ceiling at this pitch and ratio, not a step on the way to more.

Sources: SDP/SI *Handbook of Timing Belts, Pulleys, Chains and Sprockets*, Technical Section
Table 3, page T-15, `https://sdp-si.com/D820/PDFS/Technical-Section.pdf`; Gates
*PowerGrip GT3 2MGT, 3MGT & 5MGT* product datasheet; Gates *PowerGrip GT3 and HTD Belt
Drives* design manual.

## OPEN-D — the 34HS1456 torque-speed curve is BLOCKING, not cosmetic.

Upgraded from the modelling-gap note in F12. The sim treats motor torque as independent of
speed; it is not, and the speeds are no longer hypothetical:

```
D14, 4 s cycle, yaw 1:25       533 rpm
F13, yaw 1:40                  845 rpm
F13, yaw 1:50                 1058 rpm
```

A NEMA 34 of this class and inductance is well down its torque curve by 500 rpm and has lost
most of it by 1200. **Every torque and belt-tension result above is computed as if it were
not.** The partial mitigation from F12 still stands — on a triangular profile the
high-torque instant (segment start) and the high-speed instant (mid-segment) are different
instants, so the demanding case is gravity hold at speed rather than peak torque at speed —
but that is an argument for the error being smaller than it looks, not for it being absent.

**What this blocks:** the cycle-time decision itself. D14 chose 4 s partly because 2 s asked
for rpm the motors cannot deliver; that reasoning applies at 533 rpm too, just less
severely. It also blocks any yaw ratio above 1:25, which multiplies the speed directly.

**What is needed:** a torque-speed curve for the Longs 34HS1456 at the intended bus voltage
(~60 V), from the vendor or from a bench pull-out test. Until then, treat every result here
as an upper bound on capability.

## OPEN-E — column radius versus yaw ratio. Evan asked to be told; this is the telling.

D13 chose 200 mm on the expectation that the yaw stack would fit. **At 1:25 it fits with
12 mm to spare. At every higher ratio it does not.** F13 has the numbers. The choice:

1. **Keep 200 mm, keep yaw 1:25.** Resolution stays at 0.890 mm, 89% of the D5 budget, with
   7 surviving placements. Nothing else changes. The binding gate stays nearly hard.
2. **Grow the column to 244 mm, take yaw 1:40.** Resolution 0.556 mm, 56% of budget, 19
   surviving placements. Board placement does not move — 244 mm still fits behind the
   251 mm clearance the chosen placement already has, so the resolution gain is not clawed
   back. Costs 44 mm of column radius and 845 rpm at the motor, which OPEN-D says is not
   yet trustworthy.
3. **Grow past 244 mm.** Not indicated. Resolution gets worse above 1:40, not better, and
   the 0.524 mm planar floor would need D4 reopened to go further.

Blocked on OPEN-D either way: option 2's case rests on a motor speed the model cannot
currently justify.

---

## D16 — OPEN-E closed. Yaw stays at 1:25. No ratio change. SETTLED 2026-07-28

**Reasoning (Evan).** The case for 1:40 rested on reading 0.890 mm as "89% of budget". It
is 89% of the **D5 gate**, and the D5 gate is a self-imposed 20% sub-allocation of the real
±5 mm target. Actual consumption is **17.8%**. The allocation is doing its job, not
failing.

```
worst-square resolution        0.890 mm
D5 gate (self-imposed)         1.000 mm    ->  89% of the gate
accuracy target (real)         5.000 mm    ->  17.8% of the budget
```

Yaw 1:40 would move 17.8% -> 11.1%, i.e. **6.7 percentage points of a budget that is not
tight**, in exchange for 44 mm of column radius and 845 motor rpm in a regime OPEN-D says
may not exist. Declined.

**Recorded as a standing correction, not just a decision:**

> **Single-full-step resolution is a QUANTISATION figure, not a positioning accuracy.**
> It is the size of the smallest commanded increment, and it is a conservative proxy for
> where a loaded rotor can settle between detents. It is not what the tool point will
> actually miss by — that is dominated by backlash, flex and microstep error under load,
> none of which this model can see. **It is not to be treated as a hard physical
> constraint again.** The `tool_m_per_full_step_max` gate stays in the sweep because a
> budget allocation is a useful discipline, but a geometry sitting at 89% of it is not
> "nearly failing" and must not be described that way.

This supersedes the framing I used when presenting F13 and OPEN-E, which gave a
quantisation figure the authority of an accuracy figure and made the 1:40 trade look
better than it was.

## D17 — Re-derive the torque cap from measured demand. SETTLED 2026-07-28

D10's 48 N.m came from a **~30 N.m estimate that was mine, not a measurement**. F8
superseded it with 13.7–25.0 N.m and the cap was never revisited. Everything downstream —
the driver current, the belt tension budget, OPEN-B, and D12 itself — inherited a number
that had already been invalidated.

The re-derived value and the margin argument are presented for approval before being
written to config; see the proposal recorded with this entry. D10 stands as the *method*
(cap by driver current, sized to requirement not capability); only its number is stale.

## D13-REOPENED — 200 mm was chosen to package a stack D16 just cancelled. 2026-07-28

D13's stated reasoning was: 150–250 mm costs the same in resolution and torque, so choose
on packaging, because the yaw reduction stack has to live in the column. **D16 fixes the
yaw stack at 1:25, whose folded envelope is 188 mm and collinear envelope is 233 mm.** The
packaging argument that decided 200 mm now points somewhere else. Restated for a fresh
choice; see the options presented with this entry.

## D18 — The architecture comparison is live again, scoped to one candidate. SETTLED 2026-07-28

D1 said "do not start the others" and was never revisited. Fourteen of the fifteen
decisions since are detail design on candidate #6. **The palletizer is not chosen, and is
not to be chosen by default.**

**Scope: model candidate #8, SCARA+Z. One architecture, not four.**

**Reasoning (Evan).** #8 is open-chain, so it needs none of the loop machinery, and it
differs from the palletizer most on exactly the two axes that matter — gravity load and
packaging. That makes it the cheapest real test of D1's untested premise that "the
open-chain candidates are cheap once the loop machinery exists."

**Constraints on the work, as given:**

1. Same three comparison numbers, same task, same gates, **identical assumptions**.
2. **Do NOT fix the mass model first.** A relative comparison on shared unvalidated
   assumptions is more robust than any absolute number in this repo, and every absolute
   number here is going to be superseded by bench data.
3. **Resist detail design.** Coverage, peak torque, resolution. Stop there. No belt study,
   no cap, no column, no cycle time for this candidate.
4. **Report actual elapsed effort against D1's premise.** If SCARA costs what the
   palletizer cost, that is the finding, and it decides whether the remaining three
   candidates are affordable.

**Consequence for the definition of done.** CLAUDE.md is updated from five architectures
to two, with the reason stated. The remaining three are explicitly **descoped, not
deferred** — leaving them as an implied to-do nobody has scheduled is how D1 quietly became
a permanent state.

## OPEN-D — BLOCKING. Being answered by measurement, not by modelling. Updated 2026-07-28

Unchanged in substance: the sim treats motor torque as independent of speed, and the 4 s
cycle needs 533 motor rpm at 1:25 where a NEMA 34 of this class is well down its curve.
Every torque and belt-tension figure in this file is an upper bound until this closes.

**Status change: Evan is ordering single-joint rig parts.** This will be answered by a
bench pull-out test on real hardware, not by a datasheet and not by the simulator. Until
that measurement exists:

- no torque or tension result here may be presented as a capability;
- D14's 4 s cycle time is provisional, because the reasoning that rejected 2 s (rpm the
  motors cannot deliver) applies at 533 rpm too, only less severely;
- D16 is *reinforced* by this, not weakened: declining 845 rpm while 533 rpm is still
  unverified is the consistent position.

The rig also answers the three things §5 of the 2026-07-28 assessment listed as
"answerable in sim but not trustworthy": backlash, flex, and real positioning error under
load. Those are the quantities the D5 gate is a proxy for.

---

## F15 — Candidate #8 SCARA+Z, on identical terms. 2026-07-28

Built per D18. Same sweep box, same gates, same shared assumptions including the wrong
ones. `python -m app.sweep scara` against `python -m app.sweep palletizer`.

```
                                 palletizer (#6)      SCARA+Z (#8)
surviving geometries                     7                 46
best worst-square resolution        0.890 mm           0.490 mm
   as % of the 5 mm target             17.8%              9.8%
peak REVOLUTE joint torque      18.8 - 24.7 N.m     1.6 - 3.4 N.m
   of which gravity              14.6 - 19.2 N.m         0.000 N.m
worst-case belt tension            250 - 327 N         21 - 45 N
   against a 701 N allowable         36 - 47%            3 - 6%
board placements reaching all 64       yes                yes
gripper/neighbour clearance         +3.5 to +4.1 mm   +3.5 to +4.2 mm
prismatic lift force                     n/a              800 N
```

**Three structural differences, each asserted by a test rather than assumed:**

1. **Gravity torque on the revolute joints is EXACTLY zero**, to machine precision, at
   every pose. Both axes are vertical. `app.build_scara` check 5 asserts it. The
   palletizer's peak torque is 78% gravity, so this is not a small effect — it is most of
   the load disappearing.
2. **No Jacobian column scales with reach.** The palletizer's yaw column has norm `r`, the
   target's horizontal radius, which exceeds both link lengths at every surviving
   placement — so its resolution is set by how far away the board is. SCARA's columns have
   norm L1, L2 and 1. `test_no_jacobian_column_scales_with_reach` asserts the bound.
   **This is why SCARA reaches 0.490 mm where the palletizer floors at 0.890 mm**, and why
   pushing the board outward costs SCARA resolution nothing.
3. **6.6x more of the swept box survives** (46 vs 7). The palletizer is squeezed between
   the resolution gate and the base-clearance gate; SCARA is limited only by reach and
   clearance, and its `board_radius_m` survivors run out to 0.700 where the palletizer
   collapses to a single value of 0.480.

**Where SCARA is worse, and it is not nothing:**

- **The lift carries everything.** 800 N peak, of which 122 N is static weight and the rest
  is inertial. That figure is dominated by **666 kg of reflected rotor mass** — the
  consequence of copying the 1:25 belt reduction onto a linear axis with a 100 mm lead, per
  D18's identical-assumptions rule. A Z drive would obviously not be specified that way.
  **Flagged, not fixed:** fixing it is detail design and D18 puts that out of scope. Read
  the 800 N as "the shared assumption is wrong here", not as a property of the
  architecture.
- **The whole machine hangs off one vertical axis.** OPEN-SAFETY-1 applies differently: a
  power cut drops the carriage, not the arm. A leadscrew Z would be non-backdrivable and
  would remove the hazard entirely; a belt Z would not.
- **Self-collision is not checked for either candidate** (collision is off in both models,
  for F1's reason in the palletizer and by copying in SCARA). A folded SCARA pose putting
  link 2 through the column is not rejected. Same omission both sides, so the comparison is
  fair; it is still an omission, and SCARA has more folded poses available to it because
  its joint limits are unrestricted.

**Caveat that applies to the whole table.** These numbers share the palletizer's
unvalidated mass model, `efficiency = 0.90`, and D14's acceleration limits, which were
derived from the palletizer's joint displacements and applied unchanged. That is D18's
instruction and it is the right call for a *relative* comparison — but the direction of the
resulting bias is worth stating: reusing the palletizer's acceleration limits **cannot
flatter SCARA**, and using the palletizer's link cross-section on gravity-free joints makes
SCARA's torque advantage, if anything, understated.

### Effort against D1's premise

D1 claimed "the open-chain candidates are cheap once the loop machinery exists" and was
never tested. Measured:

```
                                    palletizer   SCARA
architecture-specific new code
  MJCF generator                       318         216   lines
  config                               172         156
  kinematics class                      76          91
  build/verify script                  177         145
  property tests                       105         119
                                     -----       -----
                                       848         727

shared code that had to be generalised to admit a SECOND architecture:
  368 insertions across schema.py, kinematics.py, architectures.py,
  sweep.py, sim_backend/__init__.py, palletizer_mjcf.py
```

**Verdict: the premise is half right, and the half that is right is the important half.**

- **Not cheaper in code.** 727 + 368 = ~1095 lines against the palletizer's 848. The loop
  machinery was *not* reused, because an open chain needs none of it. D1's stronger claim —
  that building #6 first meant the second candidate would be nearly free — is not supported.
- **Much cheaper in risk, which is what actually cost time.** The palletizer produced six
  substantive corrections (F1 spurious contacts, F2 solref sensitivity, F4 rank deficiency,
  F6 armature divergence *plus a wrong diagnosis first*, an IK boundary bug, a test-
  comparison bug). SCARA produced three, and **all three announced themselves as absurd**:
  0 of 432 geometries surviving, a 791 N.m torque on a gravity-free joint, and 758 mm of
  drift in a model that cannot move that far. **None of the palletizer's did.** F1 produced
  a perfect-looking headline torque with internal loads wrong by two orders of magnitude.
  That asymmetry — loud failures versus plausible ones — is the real cost difference
  between an open chain and a closed loop, and it is not visible in a line count.

**Forecast for a third candidate, if one is ever authorised.** The 368 lines of
generalisation are now paid and would not recur. #1 belt-serial is open-chain and would
cost roughly what SCARA's architecture-specific code cost, ~700 lines. #7 five-bar is a
closed loop and would reuse the `connect`/residual/armature machinery that #8 did not
touch — that is where D1's premise would finally be tested properly. #5 rotary delta is
the hardest and shares least.

## F16 — Positional indexing into `qfrc_inverse` read the wrong joints. 2026-07-28

Found while wiring SCARA into the sweep, and it is the more useful half of what building
a second architecture bought.

**Symptom.** SCARA reported 791–800 N.m of peak torque on joints that check 5 of
`app.build_scara` proves carry exactly zero gravity torque.

**Cause.** `app.sweep` read `data.qfrc_inverse[:3]`. MuJoCo orders that array by DOF index,
which follows the order joints are declared in the tree — **not** the order of `cfg.joints`,
and not the actuated set. For SCARA, DOF 0 is the prismatic lift, so the "torque" being
maxed included the lift's 666 N generalised **force**. A max across mixed units.

**It was latent in the palletizer too.** There, `qfrc_inverse[:3]` picks
`q0_yaw, q1_upper_arm, q2_forearm_rel` — and `q2_forearm_rel` is the **passive** relative
elbow hinge, not the actuated `drive_lever` the motor turns. **Verified after the fix: the
palletizer's survivor set and every torque figure are byte-identical**, because the third
element never set the maximum. So no palletizer conclusion changes, including D17's
24.7 N.m input. It was a real bug that happened not to bite.

**Fix.** Each backend module now exposes `actuated_forces(model, data)`, addressing joints
**by name** and returning forces conjugate to the **absolute** actuated coordinates.

**And the fix exposed a second thing, which is not a bug but is a modelling subtlety
neither candidate can ignore.** MuJoCo returns forces conjugate to the *tree* coordinates.
The palletizer needs no transform because all three actuated bodies are children of the
rotating base, so their coordinates are already absolute — that is a D2 payoff the model
header claims and this is the first thing that depended on it. **SCARA does need one.** Its
elbow's tree coordinate is relative, but both motors sit on the carriage and drive absolute
angles through a belt along link 1. With `theta_rel = A theta_abs`, `A = [[1,0],[-1,1]]`:

```
tau_abs = A^T tau_rel   ->   tau_abs0 = tau_rel0 - tau_rel1 ,  tau_abs1 = tau_rel1
```

Skipping it reports the shoulder torque of a machine whose elbow motor is bolted to the
elbow — the arrangement CLAUDE.md hard constraint 1 forbids. That would be the wrong
machine, not a small error.

**Generalisable lesson: never index a MuJoCo per-DOF array positionally.** `nq`, `nv`, tree
order and the actuated set are four different orderings and they coincide only by accident.

---

## F17 — D19 fairness closure: joint limits and self-collision, applied symmetrically. 2026-07-28

### Limits

One rule, applied to **both** candidates so that neither is scored on how carefully its
placeholders happened to be written:

- any joint the gripper cable crosses gets **±150°** of travel, unless a tighter physical
  constraint already applies;
- an **elbow's limit is RELATIVE to its parent**, because its travel is a property of the
  mechanism, not of where the arm points;
- world-frame constraints stay whatever they physically are.

±150° is also what industrial SCARAs of this size publish for J1 and J2. SCARA has no table
and no gravity limiting its revolute joints, so cable routing and the elbow belt genuinely
*are* the constraint — there is no slip ring in this design.

**Absolute and relative limits are BOTH applied, not one instead of the other.** The first
attempt substituted the relative limit for the absolute one and **raised the palletizer's
survivor count from 7 to 16** by silently discarding its table-clearance limit. Caught
because a tightening that loosens is obviously wrong. `JointSpec` now carries `min_rad`/
`max_rad` (absolute, always) plus optional `relative_to`/`rel_min_rad`/`rel_max_rad`.

**What the relative limit actually excludes**, corrected: not "folds beyond −150°" but the
**60° band around full doubling-back** — relative angles from 150° to 210°, which wrap onto
each other and are the same physical configuration. An earlier note claimed −210° was
physically impossible. It is not; −210° *is* +150°. The impossible region is the band near
180°, and the palletizer's absolute range alone admitted it.

```
                              before D19        after D19
palletizer  yaw               +/-180 deg        +/-150 deg   (no effect: board needs ~34)
            upper arm         [-30, +90]        unchanged    (table clearance)
            forearm           [-120, +30] abs   unchanged, PLUS relative +/-150
SCARA       shoulder          +/-180 deg        +/-150 deg   (cable)
            forearm           +/-180 deg abs    +/-180 abs, PLUS relative +/-150 (cable/belt)
```

**The palletizer's survivor set, torques and resolutions are byte-identical before and
after.** Only SCARA's move. That asymmetry is not a rigged comparison — it is what happens
when one candidate's limits were derived from physics and the other's were invented, and it
is the evidence that the tightening was applied honestly rather than aimed.

### Self-collision

Implemented in `motion/collision.py` as capsule-vs-capsule distance, **outside the
physics**, because collision is disabled on every geom in both models and cannot be turned
back on — F1 shows a closed loop puts the cut bodies in permanent coincident contact and
generates hundreds of newtons of spurious force. Link radii and column radius moved from
the MJCF generators into config (`link_radius_m`, `column_radius_m`), identical in both,
because two copies of a number that sizes both link mass and clearance is how they drift.

**Only two interference classes are checked, and the exclusion is deliberate:**

- anything against the **column** — it sits on the rotation axis, so there is nowhere to
  move it to;
- the **gripper** against a link it is not attached to — it has to descend to the board, so
  it cannot be offset out of the plane either.

Link-against-link is **excluded**. Both models place their links in one plane (the
palletizer's rods at y = 0, SCARA's two links at one carriage height) where a real build
staggers them. Checking those pairs would report collisions a competent CAD layout removes,
and would penalise the palletizer simply for having more rods. That would be a modelling
artefact scoring a design.

Gated at **zero** — interpenetration, not a margin. A build margin would be an invented
number; the actual gap is reported so one can be chosen from evidence later.

```
                       rejections    minimum gap over survivors    worst pair
palletizer                 30            +99 to +212 mm       forearm/column, gripper/upper_arm
SCARA                       0           +140 to +294 mm       link2/column, gripper/link1
```

**SCARA never self-collides anywhere in the swept box** on the checked pairs. The
palletizer loses 30 geometries. Both retain large margins where they survive.

## F18 — SCARA's Z axis, specified and scored. 2026-07-28

F15's 800 N was an artefact of copying the 1:25 belt onto a 100 mm lead — 666 kg of
reflected rotor mass. Replaced with a specified drive and scored by `app.z_drive`.

**The quantity that matters is MOTOR TORQUE, not joint force.** A fine-lead screw shows a
huge generalised force and needs almost no torque to produce it,
`tau = F * lead / (2*pi * eta)`. Quoting newtons beside the revolute joints' newton-metres
is what made the 800 N look damning.

**What picks the lead is SPEED, not resolution.** Every candidate below is far inside the
1 mm/full-step budget; what separates them is motor rpm, and OPEN-D says rpm is the thing
this project cannot cash cheques against.

```
carriage 12.47 kg measured from the model (2 revolute motors + links + gripper)
payload 35 g -- 0.3% of the carriage, and NOT what sizes this axis
100 mm hop in 0.438 s, four per move (half of D14's 3.5 s motion budget)

 lead   reflected   joint N   motor N.m   % of 8.4    rpm   mm/step
  5 mm    426 kg      1040      0.919       10.9%    5486    0.025
 10 mm    107 kg       372      0.657        7.8%    2743    0.050
 20 mm     27 kg       204      0.723        8.6%    1371    0.100
 32 mm     10 kg       171      0.965       11.5%     857    0.160   <-- SPECIFIED
```

**Specified: direct-driven 32 mm lead ball screw (SFU3232 class), no reduction.** It is the
only stock lead that lifts the carriage fast enough for D14's cycle while staying under
~1000 motor rpm, and it is also the closest inertia match (10.4 kg reflected against
12.5 kg real). Resolution 0.160 mm/full step, 6x inside budget.

**SCARA's genuine weakness, and it does not go away with a better lead.** A self-locking
trapezoidal screw needs a lead angle under the friction angle, capping the lead near 5 mm
on a 16 mm screw — which would need 6857 rpm to make the same hop. **No self-locking screw
is fast enough.** So the Z axis is backdrivable and the whole carriage drops on a power
cut. It needs a fail-safe brake. That is OPEN-SAFETY-1 again, on a different axis, and it
is a real cost of this candidate rather than something to design around.

Second real cost: **the Z axis is a dedicated serial axis.** Four vertical hops per move
consume half the motion budget on an axis that does nothing else. The palletizer makes the
same vertical moves with the joints it already has.

## F19 — Revolute torque demand against motor capability. Motor-selection evidence. 2026-07-28

Recorded explicitly at Evan's request rather than left implicit in a comparison table. Full
ranges over every surviving geometry, not the truncated top-20 quoted earlier.

```
                             palletizer (#6)      SCARA+Z (#8)
peak revolute joint torque   18.8 - 24.7 N.m    1.9 - 4.8 N.m
   of which gravity          14.6 - 19.2 N.m    exactly 0.000
available per joint
   8.4 N.m x 25                  210 N.m            210 N.m
   x 0.90 efficiency             189 N.m            189 N.m
worst demand as % of 210          11.8%              2.3%
headroom                          8.5x              43.7x
motor-side torque at worst      1.10 N.m           0.213 N.m
   as % of 8.4 N.m holding        13.1%              2.5%
```

**SCARA's revolute demand is 5.1x lower than the palletizer's and 43x under what the owned
motors deliver.** That is the evidence for motor selection, stated as such:

- **For SCARA the reduction is set by RESOLUTION alone.** 1:25 exists to get the step size
  down; torque never asks for it. For the palletizer the reduction serves both, so its
  1:25 is doing two jobs and SCARA's is doing one.
- **SCARA does not need a NEMA 34 on its revolute joints.** A NEMA 23 at ~3 N.m holding,
  at the same 1:25, would deliver 75 N.m against a 4.8 N.m worst demand — still 15x
  headroom — at identical resolution, because resolution depends on the 1.8° step and the
  ratio, not on torque.
- **The saving compounds through the Z axis.** The carriage is 12.47 kg and is dominated by
  the two 3.8 kg NEMA 34s it carries. Two NEMA 23s at ~1.2 kg would take it to roughly
  6.3 kg, roughly halving the lift's static load and its torque — the one place SCARA is
  actually working hard.
- **The Z axis is the only joint with a real duty**, at 0.965 N.m motor-side (11.5% of
  8.4 N.m holding).

**What this does NOT say.** It does not say to buy different motors — six NEMA 34s are
already owned and free, and using them costs nothing but mass. It says the *machine does
not need them*, which is the same premise flagged in the 2026-07-28 assessment: the owned
motor set the scale of the reduction, the pulleys, the belts and the column, and on this
candidate almost none of that scale is earning its keep. If SCARA is chosen, the sensible
follow-up question is whether the two revolute joints should carry NEMA 23s and free two
NEMA 34s for something else — **not a decision here, and explicitly not detail design.**

---

## D20 — Architecture: SCARA+Z (#8). The palletizer (#6) is shelved. SETTLED 2026-09-25

Closes the comparison D18 reopened. **Decided by Evan on the evidence of F15–F19**, entry
written up by Claude at his request.

**The evidence, as the code produces it today** (`python -m app.sweep <name>`,
`python -m app.pullout <name>`; SCARA's survivor count is 35, not F15's 46, because F17's
joint limits tightened SCARA and left the palletizer byte-identical):

```
                                  palletizer (#6)      SCARA+Z (#8)
surviving geometries                     7                 35
best worst-square resolution        0.890 mm           0.490 mm
peak revolute joint torque          18.8 - 24.7 N.m    1.9 - 4.8 N.m
   of which gravity                 14.6 - 19.2 N.m    exactly 0
worst belt tension vs allowable       36 - 47%           4 - 9%
self-collision rejections                30                 0
revolute motor torque at worst      1.096 N.m          0.213 N.m   (13% / 2.5% of holding)
```

**Reasoning.** SCARA is better on every axis the simulation can see, and not narrowly.
Three properties decide it rather than the margins themselves:

1. **Gravity leaves the revolute joints entirely.** The palletizer's peak torque is 78%
   gravity (F15), and gravity is also what makes OPEN-SAFETY-1 dangerous on it. SCARA moves
   the whole gravity load onto one axis that can be specified for it (F18).
2. **Resolution stops depending on reach.** No SCARA Jacobian column scales with the board
   distance (F15 item 2), so the board can be placed for ergonomics and clearance rather
   than pulled in to save resolution.
3. **It fails loudly.** Every SCARA modelling error so far announced itself as absurd; the
   palletizer's closed loops produced plausible wrong answers (F1, F6). That difference
   continues into detail design, where most of the remaining work is.

**Robust to the shared assumptions.** The comparison rests on an unvalidated mass model and
on D14's accelerations derived on the palletizer, and F15 records why both biases run
against SCARA, not for it. OPEN-D is the one open question that could still bite, and it
bites SCARA less. The revolute joints need 0.213 N·m at 532 rpm, 2.5% of holding torque.
The exposure is concentrated on the Z axis, which a lead or driver change can move and
the palletizer's three loaded joints could not.

**What SCARA costs, accepted with the decision:**

- **The Z axis needs a fail-safe brake.** No self-locking screw is fast enough (F18), so a
  power cut drops the carriage. OPEN-SAFETY-1 now lives on Z. The palletizer needed a
  brake or counterbalance on its shoulder anyway, so this is a relocation, not a new cost.
- **A dedicated serial axis spends half the motion budget on vertical hops** (F18).
- **Z is the OPEN-D-critical joint**: 1.03 N·m at 862 rpm, a quarter of holding torque.

**What carries over, and what does not.** The method decisions carry over unchanged. That
covers the conventions (D2, D3), the gated, swept, worst-square evaluation (D5, D8, D9,
D11), encoders on the output (D6), per-joint reduction (D15), the driver-current cap as a
method (D10), and the 4 s cycle (D14). The palletizer's detail design is **moot**, not
inherited: the loop constraints (D7), the column sized for its yaw stack (D13,
D13-REOPENED), its yaw ratio (D16), and its torque-cap number (D17). The belt decisions
(D4, D12) apply to SCARA's revolute joints as they stand, with F19's note that torque never
asks for them there.

**Now live, not decided here:**

- F19's question: NEMA 23s on the revolute joints (43x headroom on 34s), which would roughly
  halve the carriage the Z axis lifts.
- The Z brake itself: type, holding torque, where it mounts.
- SCARA's own cycle-time derivation. D14's accelerations are the palletizer's and D18 kept
  them for fairness. Detail design is where to re-derive them.

**Nothing is deleted.** The palletizer model, config and tests stay, and the comparison
remains reproducible from `app.sweep`. Shelved means no further detail design, not removed.

**What would reopen this:** a pull-out curve on which the Z axis fails the profile-level
check with no lead, driver or voltage change that fixes it, or a Z brake that cannot be
packaged. Either one is a Z-axis problem, and would reopen the Z drive before it reopened the
architecture.

## OPEN-D — narrowed by D20 to one number per joint. Updated 2026-09-25

With the architecture chosen, OPEN-D no longer needs a whole torque-speed curve
interpreted by judgement. It needs the curve to clear specific operating points, which
`python -m app.pullout scara` derives from config (worst case over all 35 survivors, peak
torque paired with peak speed):

```
joint           motor N.m    motor rpm    x2 margin
q0_shoulder       0.213         532       0.426 N.m
q1_forearm        0.213         532       0.426 N.m
q2_lift           1.031         862       2.062 N.m
```

(`q2_lift` reads 1.031 N·m here against F18's 0.965 because the sweep takes the force from
the full model's inverse dynamics at the configured acceleration, rather than from F18's
hand formula. It is the larger figure, so it is the one to test against.)

**Procedure in BENCH.md; data goes in `bench/` in the format of
`bench/pullout_TEMPLATE.csv`; `python -m app.pullout scara <file>` gives the verdict.**
Pairing peak torque with peak speed makes the check pessimistic, so a PASS on every
row closes OPEN-D outright, and a FAIL calls for a profile-level check rather than a
verdict.

**Open for Evan: the pass margin.** The tool defaults to 2.0 (demand at most half of
pull-out), the conservative end of stepper practice. A stepper that reaches pull-out loses
sync and drops the load, it does not slow down, so the margin is not decoration. It is a
command-line parameter until decided.

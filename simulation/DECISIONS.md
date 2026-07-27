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
gravity. A belt failure is an uncontrolled drop of a 22 kg mechanism, not a graceful
current limit. A fuse is only a fuse if failing is safe, and here it is not.

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

## OPEN-B — the torque cap and the belt spec disagree

D10 sets a 48 N.m cap. F7 measures the configured belt (HTD-5M, 25 mm, final stage) at
33.7 N.m. The config is deliberately left inconsistent and loud rather than silently
reconciled, because reconciling it is a component decision. Three ways out, F7 has the
numbers, and F8 changes which one looks sensible.

## OPEN-SAFETY-1 — a power cut drops this arm. Not to be solved now.

Belt reduction is backdrivable and there is no brake. Cutting motor power — e-stop, power
failure, a driver fault, a tripped breaker — releases the holding torque and the arm falls
under gravity. At the measured 13.7-25.0 N.m of gravity/inertial load and ~22 kg of
aluminium, that is a genuine hazard, not a nuisance.

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

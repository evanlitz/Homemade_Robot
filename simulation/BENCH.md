# Bench: the OPEN-D pull-out test

OPEN-D is the one blocking unknown: the simulation treats motor torque as independent of
speed, and a NEMA 34 is not. This page is the procedure for measuring the real curve on the
single-joint rig and turning it into a verdict. See DECISIONS.md OPEN-D and D20.

## What the measurement has to show

Stated before the test, so the test cannot be fitted to it. From
`python -m app.pullout scara` (worst case over every surviving geometry, peak torque taken
at peak speed, 2x margin):

```
joint           motor torque needed    at motor speed    x2 margin
q0_shoulder        0.213 N.m             532 rpm          0.426 N.m
q1_forearm         0.213 N.m             532 rpm          0.426 N.m
q2_lift            1.031 N.m             862 rpm          2.062 N.m   <-- the one that matters
```

The revolute joints ask for 5% of holding torque; almost any stepper passes that. **The Z
axis is the real question**: a quarter of holding torque at the fastest motor speed in the
design. The numbers regenerate from config, so re-run the command rather than trusting this
table if anything upstream changes.

## Setup

- **One 34HS1456** clamped to the rig, shaft free.
- **The driver and power supply the machine will actually use**, at the intended bus
  voltage (~60 V). Torque at speed is set by bus voltage against winding inductance; a curve
  taken on a bench supply at 24 V answers a different question.
- **Driver current at the value the build will run**, not the motor's rating. D10 caps
  torque by driver current and D17's re-derived cap is still pending; if the setting is not
  decided, take the curve twice, at rated current and at the planned cap. Low-speed torque
  scales with current, high-speed torque mostly does not, so the two curves converge toward
  the right-hand end, and where they meet is itself worth knowing.
- **Microstepping at 16**, the config value. Record it either way.
- **A step source that ramps**, e.g. a Teensy running AccelStepper. Accelerate unloaded to
  the test speed at a gentle rate (≤ 300 rpm/s), then hold that speed.

## Load: prony brake

A drum of known radius `r` on the shaft (the first-stage pulley will do). A cord or strap
wraps it; one end runs to a spring scale or load cell fixed to the bench, the other to a
second scale or an adjustable tensioner. Shaft torque is

```
torque = (F_tight - F_slack) * r
```

With the motor held at speed, increase the brake tension **slowly** until the motor stalls.
A stalled stepper does not slow down, it drops out of sync and buzzes. Record the last
steady reading before the stall. That is the pull-out torque at that speed.

A single hanging weight on a wound string (`m g r`, ramp speed until stall) is simpler but
runs out of string within a second or two at 860 rpm. Use it only for spot checks at low speed.

## Speeds

At least `100, 200, 300, 400, 500, 532, 600, 700, 800, 862, 900, 1000, 1200` rpm, **three
runs at each**. The scorer keeps the worst of repeated readings, so put every run in the file.

- Measure past 862 rpm. The scorer refuses to extrapolate beyond the highest measured speed
  and reports `NOT MEASURED` instead.
- Steppers have mid-band resonance, typically a few hundred rpm. If one speed reads
  noticeably low, add points around it. A dip that lands on 532 or 862 rpm is the finding.

## Recording and scoring

Copy `bench/pullout_TEMPLATE.csv` to `bench/pullout_<motor>_<volts>V.csv`, fill in the
header (voltage measured at the driver under load, current setting, microsteps, drum
radius), one row per reading. Then:

```
python -m app.pullout scara bench/pullout_34HS1456_60V.csv
```

- **Every row PASS** closes OPEN-D for this architecture. Record it in DECISIONS.md with the
  file name.
- **Any FAIL is not yet conclusive.** The check pairs peak torque with peak speed, which
  happen at different instants (F12). The next step is a profile-level check on that joint,
  not a new motor.
- `--margin` changes the pass ratio. 2.0 is the conservative end of stepper practice; the
  margin to adopt is Evan's call and is recorded under OPEN-D.

## While the rig is set up

The same rig answers the three things the simulation cannot see and D5's resolution gate
stands in for: **backlash** (reverse direction under a small load, dial indicator on the
output), **flex** (static load at the output, deflection against load), and **positioning
error under load** (command a sequence of moves, measure where the output actually lands).
They do not block OPEN-D, but they are what the ±5 mm budget will actually be spent on.

## Safety

Guard the drum; a snapped cord at 860 rpm is a whip. Eye protection. Stalling a stepper does
not damage it or a current-limited driver, but a brake drum gets hot quickly, so keep runs
short.

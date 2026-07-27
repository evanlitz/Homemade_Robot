"""Angle and frame conventions assumed by every module in `motion/`.

STATUS: RATIFIED 2026-07-27. See DECISIONS.md D2 and D3. Changing either now
is a rewrite of this package, not an edit.

Units
-----
SI everywhere inside this package: metres, radians, seconds, newton-metres.
Conversion to mm/degrees happens in `app/`, never here.

Joint vector
------------
`theta` is a 1-D array of shape (n_dof,), float64, radians. Its ordering is the
order of `ArchitectureConfig.joints`, base outward. `n_dof` counts *actuated*
degrees of freedom, not MuJoCo `qpos` entries -- for a closed-loop candidate
those differ, and `backends/sim_backend` owns the mapping between them.

Angle reference -- D2, SETTLED
-----------------------------
Every angle is ABSOLUTE: the link's own angle from horizontal in the arm's
vertical plane, right-handed, world-referenced. NOT parent-relative.

FK is therefore a plain sum of link vectors:

    r = sum_i L_i cos(theta_i)      z = h + sum_i L_i sin(theta_i)

and the palletizer's defining invariant is `theta_tool = 0` -- a statement that
cannot be forgotten and whose violation is visible on inspection. In
parent-relative coordinates the same invariant is `theta_tool = -theta_forearm`,
a correction term whose sign error survives review.

The cost is a translation layer to MuJoCo's `qpos`, which for a serial
sub-chain is a constant lower-triangular matrix of ones. It lives in
`backends/sim_backend/`, never here. `motion/` does not know `qpos` exists.

DEAD CONVENTION -- do not reintroduce: theta1 absolute, theta2 parent-relative.
It is dangerous precisely because it is almost right: theta1 agrees with D2, so
single-joint checks pass, and the two definitions of theta2 differ by exactly
theta1 -- which is ~0 near home. The error is invisible near home and grows with
shoulder angle. See DECISIONS.md D2-SUPERSEDED.

Tool pose -- D3, SETTLED
------------------------
`pose` is a 3-element array: (x, y, z) in metres. No orientation.

The palletizer has 3 DOF and holds the tool level by geometry, so tool yaw is
not independent -- it is atan2(y, x), determined by where the arm was sent.
A yaw field would hold derived data describing a motor that does not exist,
which is how someone ends up commanding a yaw that silently does nothing.

D3 rests on a hypothesis that is to be TESTED, not protected: that a radial-only
approach never fouls a neighbouring piece. Squares are 57 mm, pieces ~30-35 mm,
so there is ~11-13 mm of clearance per side. If the clearance check against a
densely populated board fails, that is evidence for a 4th motor and D3 reopens.
See DECISIONS.md D3-REQ.

Frames
------
World frame is right-handed, +Z up, origin at the arm's base mounting face.
The board frame is a rigid transform of the world frame, defined in config,
not here.

IK branches
-----------
IK returns *all* valid solutions, not one. Branch selection (elbow-up vs
elbow-down, delta assembly mode, five-bar working mode) is a policy decision
that belongs to the caller, because the right branch depends on the previous
pose and on self-collision, neither of which IK knows about.
"""

from __future__ import annotations

from typing import TypeAlias

import numpy as np

JointVector: TypeAlias = np.ndarray
Pose: TypeAlias = np.ndarray

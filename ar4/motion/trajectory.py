"""Cartesian straight-line trajectories with a trapezoidal velocity profile.

Chess needs approach / descend / grip / lift / traverse / place. Interpolating
in joint space between two poses does not travel in a straight line, so a
"lift" would arc sideways into the neighbouring piece. Every segment here moves
the tool along a straight line in space, with orientation slerped alongside.

The output is joint waypoints in degrees, uniformly spaced in time -- feed them
to a backend in order. Nothing in this module touches a backend or does I/O.
"""
import numpy as np

from motion.kinematics import JOINT_LIMITS, fk, jacobian, in_limits
from motion.ik import ik

# --- defaults -------------------------------------------------------------
# 250 mm/s and 500 mm/s^2 traverse a 400 mm board move in ~2.1 s, which is
# unhurried for a stepper arm and leaves headroom for the real hardware.
V_MAX_MM_S = 250.0
A_MAX_MM_S2 = 500.0
W_MAX_DEG_S = 90.0
ALPHA_MAX_DEG_S2 = 180.0
DT = 0.02

# J5 near zero is the wrist singularity: J4 and J6 go collinear. Independent of
# MIN_SIGMA below, because the Jacobian's smallest singular value also dips for
# elbow and shoulder conditioning -- at |J5| = 8 deg it can still be 1.8e-3,
# lower than it is 2 deg from the wrist singularity. One threshold cannot cover
# both, so there are two.
MIN_J5_DEG = 4.0
MIN_SIGMA = 8e-4

# A step larger than this between consecutive waypoints means IK jumped to a
# different solution branch. The Cartesian step is small by construction, so a
# large joint step is never a real motion -- it is a discontinuity the arm
# would execute as a lurch.
MAX_JOINT_STEP_DEG = 12.0


class PlanningError(Exception):
    """Raised when a segment cannot be realised.

    Carries the waypoint index and the fraction along the path, so a caller can
    say *where* the move failed rather than only that it did.
    """

    def __init__(self, message, index, u, q=None):
        super().__init__(f"{message} at waypoint {index} (u={u:.3f})")
        self.index = index
        self.u = u
        self.q = q


class Trajectory:
    """Joint waypoints, degrees, uniformly spaced by dt."""

    def __init__(self, q, dt):
        self.q = np.asarray(q, dtype=float)
        self.dt = float(dt)

    def __len__(self):
        return len(self.q)

    @property
    def duration(self):
        return (len(self.q) - 1) * self.dt

    @property
    def times(self):
        return np.arange(len(self.q)) * self.dt

    def joint_speeds(self):
        """Per-waypoint joint velocity, deg/s. (N-1, 6)."""
        return np.diff(self.q, axis=0) / self.dt


# --- rotation helpers -----------------------------------------------------

def rotvec_from_matrix(r):
    """Rotation matrix -> rotation vector, robust near theta = pi.

    The usual off-diagonal formula divides by sin(theta), which vanishes at pi.
    There the symmetric part is used instead: R + I has rank one and its
    largest column is parallel to the axis.
    """
    angle = np.arccos(np.clip((np.trace(r) - 1.0) / 2.0, -1.0, 1.0))
    if angle < 1e-9:
        return np.zeros(3)
    if angle < np.pi - 1e-6:
        axis = np.array([r[2, 1] - r[1, 2],
                         r[0, 2] - r[2, 0],
                         r[1, 0] - r[0, 1]]) / (2.0 * np.sin(angle))
        return axis * angle
    m = r + np.eye(3)
    axis = m[:, int(np.argmax(np.linalg.norm(m, axis=0)))]
    axis = axis / np.linalg.norm(axis)
    # sign is ambiguous at exactly pi; pick the branch the off-diagonal implies
    off = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    if off @ axis < 0:
        axis = -axis
    return axis * angle


def matrix_from_rotvec(v):
    theta = np.linalg.norm(v)
    if theta < 1e-12:
        return np.eye(3)
    k = np.asarray(v, dtype=float) / theta
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(theta) * kx + (1.0 - np.cos(theta)) * (kx @ kx)


def interpolate_pose(a, b, u):
    """Pose a fraction u along the straight line from a to b.

    Position interpolates linearly, orientation by slerp -- the constant-rate
    rotation about the fixed axis taking a's frame to b's.
    """
    out = np.eye(4)
    out[:3, 3] = a[:3, 3] + u * (b[:3, 3] - a[:3, 3])
    delta = rotvec_from_matrix(b[:3, :3] @ a[:3, :3].T)
    out[:3, :3] = matrix_from_rotvec(delta * u) @ a[:3, :3]
    return out


# --- time parameterisation ------------------------------------------------

def trapezoid_duration(length, v_max, a_max):
    """How long a trapezoidal move of `length` takes, and its peak velocity.

    Degenerates to a triangular profile when the move is too short to reach
    v_max, which is the common case for a 20 mm chess lift.
    """
    if length <= 0.0:
        return 0.0, 0.0
    if length <= v_max ** 2 / a_max:          # never reaches cruise
        v_peak = np.sqrt(length * a_max)
        return 2.0 * v_peak / a_max, v_peak
    t_ramp = v_max / a_max
    t_cruise = (length - v_max ** 2 / a_max) / v_max
    return 2.0 * t_ramp + t_cruise, v_max


def trapezoid_profile(length, v_max, a_max, duration=None, dt=DT):
    """Sampled (t, s) for a trapezoidal move, s in [0, length].

    `duration` stretches the profile to a longer time than the limits require,
    used to make translation and rotation finish together.
    """
    natural, _ = trapezoid_duration(length, v_max, a_max)
    total = natural if duration is None else max(natural, duration)
    if total <= 0.0:
        return np.zeros(1), np.zeros(1)

    n = max(1, int(np.ceil(total / dt)))
    t = np.linspace(0.0, total, n + 1)

    # Re-derive the profile for the (possibly stretched) duration. Solving
    # a*(T/2)^2 = L for a triangular fit, or holding the ramp fraction for a
    # trapezoid, both reduce to scaling the natural profile in time.
    scale = natural / total if total > 0 else 1.0
    tn = t * scale
    a = a_max
    _, v_peak = trapezoid_duration(length, v_max, a_max)
    t_ramp = v_peak / a if a > 0 else 0.0
    t_flat = max(0.0, natural - 2.0 * t_ramp)

    s = np.empty_like(tn)
    for i, ti in enumerate(tn):
        if ti <= t_ramp:
            s[i] = 0.5 * a * ti ** 2
        elif ti <= t_ramp + t_flat:
            s[i] = 0.5 * a * t_ramp ** 2 + v_peak * (ti - t_ramp)
        else:
            td = max(0.0, natural - ti)
            s[i] = length - 0.5 * a * td ** 2
    s[0] = 0.0
    s[-1] = length
    return t, np.clip(s, 0.0, length)


# --- validation -----------------------------------------------------------

def check_waypoint(q, q_prev, index, u,
                   min_j5_deg=MIN_J5_DEG, min_sigma=MIN_SIGMA,
                   max_step_deg=MAX_JOINT_STEP_DEG):
    """Reject a waypoint the arm should not be asked to execute.

    Four distinct failures, each with its own message so a caller can tell them
    apart: outside joint limits, through the wrist singularity, badly
    conditioned for any other reason, or discontinuous from the previous
    waypoint because IK changed solution branch.
    """
    if not in_limits(q):
        bad = [i + 1 for i, (lo, hi) in enumerate(JOINT_LIMITS)
               if not lo <= q[i] <= hi]
        raise PlanningError(f"joint limit exceeded on J{bad}", index, u, q)

    if abs(q[4]) < min_j5_deg:
        raise PlanningError(
            f"passes within {abs(q[4]):.2f} deg of the J5=0 wrist singularity",
            index, u, q)

    sigma = np.linalg.svd(jacobian(q), compute_uv=False)[-1]
    if sigma < min_sigma:
        raise PlanningError(
            f"ill-conditioned, smallest singular value {sigma:.2e}", index, u, q)

    if q_prev is not None:
        step = np.max(np.abs(q - q_prev))
        if step > max_step_deg:
            raise PlanningError(
                f"joint step of {step:.1f} deg -- IK changed branch", index, u, q)


# --- planning -------------------------------------------------------------

def plan_line(q_start, target, v_max=V_MAX_MM_S, a_max=A_MAX_MM_S2,
              w_max=W_MAX_DEG_S, alpha_max=ALPHA_MAX_DEG_S2, dt=DT,
              validate=True, tool=False, **checks):
    """Straight line in space from fk(q_start) to `target` (4x4, metres).

    Translation and rotation are given separate limits and the segment runs for
    whichever takes longer, so a move that barely translates but flips the
    wrist is paced by the wrist rather than executed instantly.

    Every waypoint is IK-solved warm-seeded from the previous one. That is what
    keeps the joint path continuous, and it is the call pattern the solver is
    best at -- a cold multistart per waypoint would be both slower and free to
    jump between solution branches mid-path.

    Raises PlanningError with the offending index if any waypoint is
    unreachable, out of limits, singular, or discontinuous.
    """
    q_start = np.asarray(q_start, dtype=float)
    # Interpolate in whichever frame the caller is aiming with. Under a rotating
    # wrist the flange and the fingertip trace different curves, so a line that
    # is straight for one is not straight for the other.
    start = fk(q_start, tool=tool)

    length_mm = np.linalg.norm(target[:3, 3] - start[:3, 3]) * 1000.0
    angle_deg = np.rad2deg(
        np.linalg.norm(rotvec_from_matrix(target[:3, :3] @ start[:3, :3].T)))

    t_lin, _ = trapezoid_duration(length_mm, v_max, a_max)
    t_ang, _ = trapezoid_duration(angle_deg, w_max, alpha_max)
    duration = max(t_lin, t_ang)

    if duration <= 0.0:
        return Trajectory(q_start.reshape(1, 6), dt)

    # Drive both by one normalised u(t), profiled on whichever binds.
    if t_lin >= t_ang:
        _, s = trapezoid_profile(length_mm, v_max, a_max, duration, dt)
        u_of_t = s / length_mm
    else:
        _, s = trapezoid_profile(angle_deg, w_max, alpha_max, duration, dt)
        u_of_t = s / angle_deg

    out = [q_start]
    for i, u in enumerate(u_of_t[1:], start=1):
        pose = interpolate_pose(start, target, u)
        q = ik(pose, out[-1], tool=tool)
        if q is None:
            raise PlanningError("unreachable", i, u)
        if validate:
            check_waypoint(q, out[-1], i, u, **checks)
        out.append(q)

    return Trajectory(np.array(out), dt)


def plan_path(q_start, targets, **kw):
    """Chain straight-line segments through a list of poses.

    The pick-and-place shape chess needs -- approach, descend, lift, traverse,
    place -- is exactly this. Segments are butt-joined: the arm comes to rest at
    each waypoint, which is what you want when the middle of the sequence is
    "close the gripper".
    """
    q = np.asarray(q_start, dtype=float)
    pieces = []
    for target in targets:
        seg = plan_line(q, target, **kw)
        pieces.append(seg.q if not pieces else seg.q[1:])
        q = seg.q[-1]
    dt = kw.get("dt", DT)
    return Trajectory(np.vstack(pieces), dt)

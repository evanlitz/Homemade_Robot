"""Teensy backend. Same interface as the simulator, real robot on the end.

Speaks the serial protocol of the ar4_ros_driver firmware
(annin_ar4_firmware/AR4_teensy, version 2.1.0) and, for the SG1 servo gripper,
the Arduino Nano sketch beside it (AR4_nano, 0.1.0). Every line is ASCII and
ends in "\\n"; joints are lettered A..F.

For an MK5, flash the sketch from Annin-Robotics/ar4_ros_driver, NOT from
ycheng517's upstream: both report 2.1.0 and speak the same protocol, but only
Annin's fork accepts model "mk5" (upstream stops at mk3, and a model it does
not know sends the firmware into its unrecoverable error state). Annin's fork
also adds PK (park) and JM (home selected joints), which this backend does not
need.

    host -> Teensy                          Teensy -> host
    STA<version>B<model>   handshake        STA<ok>B<version>C<ok>D<model>
    JC<type><order x6>     home to limits   JCA..F<encoder counts>, or ER...
    JP                     read position    JPA<deg>B<deg>...F<deg>
    MV A<deg/s>..F<deg/s>  velocity         ES<0|1>   (e-stop state)
    MT A<deg>..F<deg>      position         ES<0|1>
    RE                     reset e-stop     ES<0|1>
    any time: DB... debug, WN... warning, ER... error

    host -> Nano: ST -> "<version>", SV0P<deg> -> "Done"

FIRMWARE FRAME IS NOT THE DH FRAME. After JC homing the firmware measures
each joint from its limit switch; ar4_ros_driver adds a per-joint offset to
get the URDF angle (ar_hardware_interface.cpp, read()). The URDF angle then
matches our DH angle on every joint except J1, whose URDF joint carries an
rpy="pi 0 0" and turns the other way -- the same divergence models/ar4.xml
records and test_sim_model.test_j1_sign_is_not_the_urdf_one guards. So

    dh = SIGN * (fw + OFFSET)          fw = SIGN * dh - OFFSET

The offsets below are the fork's joint_offsets/<model>.yaml. They are what
makes the numbers mean anything, and they have NOT been checked on this arm:
see the bring-up list at the bottom of this file before the first powered
move.
"""
import re
import time

import numpy as np

from backends.base import Backend
from motion.kinematics import JOINT_LIMITS

FIRMWARE_VERSION = "2.1.0"
NANO_VERSION = "0.1.0"
BAUD = 115200

# firmware limit-switch frame -> URDF, degrees; joint_offsets/<model>.yaml.
# The MK5 differs only on J1, whose travel is +/-160 rather than +/-170.
JOINT_OFFSETS_BY_MODEL = {
    "mk3": (170.0, -42.0, -89.0, -180.0, -105.0, -180.0),
    "mk5": (160.0, -42.0, -89.0, -180.0, -105.0, -180.0),
}
DEFAULT_MODEL = "mk5"
JOINT_OFFSETS_DEG = JOINT_OFFSETS_BY_MODEL[DEFAULT_MODEL]
# URDF -> DH. Only J1 differs; see the module docstring.
JOINT_SIGNS = (-1.0, 1.0, 1.0, 1.0, 1.0, 1.0)

# JOINT_MAX_SPEED and JOINT_MAX_ACCEL in AR4_teensy.ino, identical on every
# joint and every model. The firmware silently clips velocity commands to the
# first and ramps every command at the second, so a trajectory asking for more
# is not refused -- the arm just falls behind it and cuts the corner.
FW_MAX_SPEED_DEG_S = 60.0
FW_MAX_ACCEL_DEG_S2 = 30.0
# Fraction of those limits a trajectory may use. The rest is left for the
# position correction in follow(), which has to be able to catch up.
LIMIT_MARGIN = 0.8

# Position correction gain for follow(), 1/s. Velocity sent is the planned
# velocity plus this times the tracking error. NOT MEASURED -- a starting point
# low enough not to ring against a 30 deg/s^2 ramp. Tune from last_follow on
# the real arm, the way SETPOINT_LEAD_S was tuned in the simulator.
TRACKING_GAIN = 4.0

# "At rest on q[-1]" means within this of it once the encoders stop changing.
# Several times the encoder resolution (~0.005 deg on J1); a miss this large is
# a stall or lost steps, which should stop the program rather than be ignored.
ARRIVAL_TOL_DEG = 0.5
STILL_TOL_DEG = 0.01
SETTLE_TIMEOUT_S = 15.0

# SG1 servo angles, gripper_driver.yaml. The servo reports only the angle it
# was last told, so completion is a fixed wait, not a measurement.
SERVO_CLOSED_DEG = 0
SERVO_OPEN_DEG = 35
GRIPPER_WAIT_S = 0.6

# JC drives every joint to its switch and back; the firmware's own timeouts
# add up to ~46 s for a single group.
CALIBRATE_TIMEOUT_S = 180.0

_ST_REPLY = re.compile(r"^STA(\d)B(.*)C(\d)D(.*)$")
# No exponent form: the firmware prints fixed-point (String(x, 6)) or ints, and
# allowing one would read "D0E0" -- joint D is 0, joint E is 0 -- as 0e0.
_VALUE = re.compile(r"([A-F])(-?[0-9]+(?:\.[0-9]*)?)")


class HardwareError(RuntimeError):
    """The firmware reported ER, stopped answering, or the arm did not arrive."""


class EStopError(HardwareError):
    """The e-stop is latched. Release it, then call reset_estop()."""


class TrajectoryLimitError(ValueError):
    """The trajectory asks for more than the firmware will deliver.

    Carries `slowdown`, the factor to stretch dt by for it to fit.
    """

    def __init__(self, message, slowdown):
        super().__init__(message)
        self.slowdown = slowdown


def parse_values(line, header):
    """'JPA1.5B-2...' -> [1.5, -2.0, ...]. Raises if a joint is missing."""
    if not line.startswith(header):
        raise HardwareError(f"expected {header} reply, got {line!r}")
    found = dict(_VALUE.findall(line[len(header):]))
    try:
        return [float(found[c]) for c in "ABCDEF"]
    except KeyError as exc:
        raise HardwareError(f"reply {line!r} is missing joint {exc}") from None


def format_values(header, values):
    return header + "".join(f"{c}{v:.4f}" for c, v in zip("ABCDEF", values)) + "\n"


def trajectory_demand(q, dt):
    """(peak |velocity|, peak |acceleration|) over all joints, deg/s, deg/s^2."""
    q = np.asarray(q, dtype=float)
    if len(q) < 2:
        return 0.0, 0.0
    v = np.diff(q, axis=0) / dt
    a = np.diff(v, axis=0) / dt if len(v) > 1 else np.zeros((1, q.shape[1]))
    return float(np.abs(v).max()), float(np.abs(a).max())


def required_slowdown(q, dt, max_speed=FW_MAX_SPEED_DEG_S,
                      max_accel=FW_MAX_ACCEL_DEG_S2, margin=LIMIT_MARGIN):
    """Smallest factor k >= 1 such that the same waypoints at dt * k fit.

    Stretching time by k divides velocity by k and acceleration by k^2, and
    leaves the path itself untouched, so this is exact.
    """
    v, a = trajectory_demand(q, dt)
    k_v = v / (max_speed * margin)
    k_a = np.sqrt(a / (max_accel * margin))
    return max(1.0, k_v, k_a)


def _open_serial(port, baud):
    import serial  # pyserial; only needed when a real port is opened
    return serial.Serial(port, baudrate=baud, timeout=1.0)


class HwBackend(Backend):
    """AR4 over USB serial.

    `port` is the Teensy (e.g. "COM5" or "/dev/ttyACM0"); `gripper_port` the
    Nano, or None for an arm with no gripper. `transport` and `gripper_transport`
    replace the serial ports with anything that has write(bytes), readline()
    and close(); tests use that. `clock` and `sleep` are injectable for the
    same reason.

    Homing is explicit. The firmware's encoders count from wherever the arm
    was at power-on, so until calibrate() has run the reported angles are
    meaningless and every motion call refuses. Pass assume_calibrated=True only
    when the Teensy has stayed powered since it last homed.
    """

    def __init__(self, port=None, gripper_port=None, model=DEFAULT_MODEL,
                 firmware_version=FIRMWARE_VERSION, offsets_deg=None,
                 signs=JOINT_SIGNS, assume_calibrated=False,
                 transport=None, gripper_transport=None,
                 clock=time.monotonic, sleep=time.sleep,
                 tracking_gain=TRACKING_GAIN):
        self.port = port
        self.gripper_port = gripper_port
        self.model = model
        self.firmware_version = firmware_version
        if offsets_deg is None:
            if model not in JOINT_OFFSETS_BY_MODEL:
                raise ValueError(f"no joint offsets known for {model!r}; pass "
                                 f"offsets_deg from joint_offsets/{model}.yaml")
            offsets_deg = JOINT_OFFSETS_BY_MODEL[model]
        self.offsets = np.asarray(offsets_deg, dtype=float)
        self.signs = np.asarray(signs, dtype=float)
        self.calibrated = bool(assume_calibrated)
        self.tracking_gain = float(tracking_gain)
        self._io = transport
        self._grip_io = gripper_transport
        self._clock = clock
        self._sleep = sleep
        self._gripper = None
        self.connected = False
        # (t, commanded deg, measured deg) per tick of the last follow(), so
        # the tracking gain can be tuned from data rather than guessed.
        self.last_follow = None
        self.messages = []  # DB / WN lines, newest last

    # --- frames ----------------------------------------------------------

    def to_dh(self, fw_deg):
        return self.signs * (np.asarray(fw_deg, dtype=float) + self.offsets)

    def to_fw(self, dh_deg):
        return self.signs * np.asarray(dh_deg, dtype=float) - self.offsets

    # --- wire ------------------------------------------------------------

    def _exchange(self, io, msg):
        """Send one line, return the first reply that is not DB or WN."""
        io.write(msg.encode("ascii"))
        while True:
            raw = io.readline()
            if not raw:
                raise HardwareError(f"no reply to {msg.strip()!r} (timed out)")
            line = raw.decode("ascii", errors="replace").strip()
            if line.startswith(("DB", "WN")):
                self.messages.append(line)
                continue
            if line.startswith("ER"):
                raise HardwareError(f"firmware error after {msg.strip()!r}: {line}")
            return line

    def _command(self, msg):
        """MT / MV / RE: reply is the e-stop state."""
        line = self._exchange(self._io, msg)
        if not line.startswith("ES"):
            raise HardwareError(f"expected ES reply to {msg.strip()!r}, got {line!r}")
        if line[2:].strip() == "1":
            raise EStopError("e-stop is pressed")

    def _read_fw(self):
        return np.array(parse_values(self._exchange(self._io, "JP\n"), "JP"))

    # --- lifecycle -------------------------------------------------------

    def connect(self):
        if self.connected:
            return
        if self._io is None:
            self._io = _open_serial(self.port, BAUD)
        if self._grip_io is None and self.gripper_port is not None:
            self._grip_io = _open_serial(self.gripper_port, BAUD)

        line = self._exchange(self._io,
                              f"STA{self.firmware_version}B{self.model}\n")
        m = _ST_REPLY.match(line)
        if not m:
            raise HardwareError(f"unexpected handshake reply {line!r}")
        if m.group(1) != "1":
            raise HardwareError(f"firmware is {m.group(2)}, this backend speaks "
                                f"{self.firmware_version}")
        if m.group(3) != "1":
            raise HardwareError(f"firmware does not know model {self.model!r}")

        if self._grip_io is not None:
            # opening the port resets the Nano; its bootloader needs ~2 s
            self._sleep(2.0)
            ver = self._exchange(self._grip_io, "ST\n")
            if ver != NANO_VERSION:
                raise HardwareError(f"gripper firmware is {ver!r}, expected "
                                    f"{NANO_VERSION}")
        self.connected = True

    def disconnect(self):
        for io in (self._io, self._grip_io):
            if io is not None:
                try:
                    io.close()
                except Exception:
                    pass
        self._io = self._grip_io = None
        self.connected = False

    def calibrate(self, sequence="0012345", timeout_s=CALIBRATE_TIMEOUT_S):
        """Home every joint against its limit switch. The arm WILL move.

        `sequence` is ar4_ros_driver's calib_sequence: a type digit (0 all at
        once, 1 two groups of three, 2 three pairs, 3 one at a time) then the
        six joint indices in order.
        """
        if len(sequence) != 7 or not sequence.isdigit():
            raise ValueError("calibration sequence is 7 digits, e.g. '0012345'")
        io = self._io
        old = getattr(io, "timeout", None)
        if old is not None:
            io.timeout = timeout_s
        try:
            line = self._exchange(io, f"JC{sequence}\n")
        finally:
            if old is not None:
                io.timeout = old
        parse_values(line, "JC")
        self.calibrated = True

    def reset_estop(self):
        """Clear a latched e-stop once the button is released."""
        self._command("RE\n")

    def _require_ready(self):
        if not self.connected:
            raise HardwareError("not connected")
        if not self.calibrated:
            raise HardwareError("not homed: call calibrate() first, or pass "
                                "assume_calibrated=True if the Teensy has "
                                "stayed powered since it last homed")

    def _check_limits(self, q):
        q = np.atleast_2d(q)
        for j, (lo, hi) in enumerate(JOINT_LIMITS):
            bad = np.flatnonzero((q[:, j] < lo) | (q[:, j] > hi))
            if len(bad):
                i = int(bad[0])
                raise ValueError(f"J{j + 1} = {q[i, j]:.2f} deg at waypoint {i} is "
                                 f"outside [{lo}, {hi}]. The firmware only checks "
                                 f"+/-380, so this is the last line of defence.")

    # --- Backend ---------------------------------------------------------

    def get_joints(self):
        self._require_ready()
        return list(self.to_dh(self._read_fw()))

    def move_joints(self, angles_deg, speed=25):
        """Joint-space move at `speed` percent of the firmware limits.

        Streamed as a smoothstep through follow() rather than sent as one MT,
        because MT always runs at the firmware's full 60 deg/s and would
        ignore `speed`.
        """
        self._require_ready()
        target = np.asarray(angles_deg, dtype=float)
        self._check_limits(target)
        start = np.asarray(self.get_joints())
        span = float(np.max(np.abs(target - start)))
        frac = min(max(speed, 1), 100) / 100.0
        v = FW_MAX_SPEED_DEG_S * LIMIT_MARGIN * frac
        a = FW_MAX_ACCEL_DEG_S2 * LIMIT_MARGIN * frac
        # smoothstep peaks at 1.5 span/T in velocity and 6 span/T^2 in accel
        duration = max(0.15, 1.5 * span / v, np.sqrt(6.0 * span / a))
        dt = 0.02
        n = max(2, int(np.ceil(duration / dt)) + 1)
        s = np.linspace(0.0, 1.0, n)
        s = s * s * (3.0 - 2.0 * s)
        q = start + (target - start) * s[:, None]
        return self._stream(q, duration / (n - 1))

    def follow(self, trajectory):
        """Stream the trajectory as velocity commands with position feedback.

        Refuses, before anything moves, a trajectory outside the joint limits
        or faster than the firmware can track; TrajectoryLimitError.slowdown
        says how much to stretch it.
        """
        self._require_ready()
        q = np.asarray(trajectory.q, dtype=float)
        if len(q) == 0:
            return self.get_joints()
        if len(q) == 1:
            return self.move_joints(q[0])
        return self._stream(q, float(trajectory.dt))

    def _stream(self, q, dt):
        self._check_limits(q)
        k = required_slowdown(q, dt)
        if k > 1.0 + 1e-9:
            v, a = trajectory_demand(q, dt)
            raise TrajectoryLimitError(
                f"trajectory peaks at {v:.1f} deg/s and {a:.1f} deg/s^2; the "
                f"firmware allows {FW_MAX_SPEED_DEG_S:.0f} and "
                f"{FW_MAX_ACCEL_DEG_S2:.0f} ({LIMIT_MARGIN:.0%} usable). "
                f"Stretch dt by {k:.2f}x.", k)

        fw = self.to_fw(q)
        v_ff = np.diff(fw, axis=0) / dt
        cap = FW_MAX_SPEED_DEG_S
        log_t, log_cmd, log_meas = [], [], []

        t0 = self._clock()
        try:
            for i in range(len(q) - 1):
                # hold the schedule: tick i belongs at t0 + i dt
                wait = t0 + i * dt - self._clock()
                if wait > 0:
                    self._sleep(wait)
                meas = self._read_fw()
                err = fw[i] - meas
                vel = np.clip(v_ff[i] + self.tracking_gain * err, -cap, cap)
                self._command(format_values("MV", vel))
                log_t.append(self._clock() - t0)
                log_cmd.append(q[i])
                log_meas.append(self.to_dh(meas))
        except BaseException:
            # whatever went wrong, do not leave six joints running at speed
            try:
                self._command(format_values("MV", np.zeros(6)))
            except Exception:
                pass
            raise
        finally:
            self.last_follow = (np.array(log_t), np.array(log_cmd),
                                np.array(log_meas))

        # MT lands on the final waypoint exactly; MV alone would drift by the
        # tracking error of the last tick
        wait = t0 + (len(q) - 1) * dt - self._clock()
        if wait > 0:
            self._sleep(wait)
        self._command(format_values("MT", fw[-1]))
        return self._settle(q[-1])

    def _settle(self, target_dh):
        """Block until the encoders stop, then insist the arm is on target."""
        deadline = self._clock() + SETTLE_TIMEOUT_S
        prev = self._read_fw()
        while True:
            self._sleep(0.05)
            now = self._read_fw()
            if np.max(np.abs(now - prev)) < STILL_TOL_DEG:
                break
            if self._clock() > deadline:
                raise HardwareError(f"arm still moving {SETTLE_TIMEOUT_S:.0f} s "
                                    f"after the last setpoint")
            prev = now
        reached = self.to_dh(now)
        miss = np.abs(reached - target_dh)
        if miss.max() > ARRIVAL_TOL_DEG:
            j = int(miss.argmax())
            raise HardwareError(f"stopped {miss[j]:.2f} deg short on J{j + 1} "
                                f"(stall or lost steps?)")
        return list(reached)

    def set_gripper(self, open_frac):
        if self._grip_io is None:
            raise HardwareError("no gripper port configured")
        frac = float(np.clip(open_frac, 0.0, 1.0))
        angle = round(SERVO_CLOSED_DEG + frac * (SERVO_OPEN_DEG - SERVO_CLOSED_DEG))
        reply = self._exchange(self._grip_io, f"SV0P{angle}\n")
        if reply != "Done":
            raise HardwareError(f"gripper replied {reply!r} to SV0P{angle}")
        self._sleep(GRIPPER_WAIT_S)
        self._gripper = frac
        return frac


# --- first power-up ------------------------------------------------------
#
# Nothing in this file has driven a real arm yet. tools/bringup.py walks
# through these steps interactively and writes down what to change; the list
# is kept here so the reasoning stays next to the code it checks. In this
# order, e-stop in hand, before trusting it with a trajectory:
#
# 1. Flash AR4_teensy 2.1.0 from Annin-Robotics/ar4_ros_driver (and
#    AR4_nano 0.1.0). connect() and nothing else;
#    a version or model mismatch fails here, before any motion.
# 2. calibrate(). Watch every joint reach its switch and come back.
# 3. get_joints() with the arm at rest after homing. Upstream parks it at
#    roughly zero in the URDF frame, so expect roughly [0]*6 here. A joint
#    reading near +/-2x its offset has the wrong offset sign.
# 4. move_joints() on ONE joint at a time, +10 deg, speed=10. Check the
#    direction against the sim, not against intuition: J1 is deliberately
#    reversed relative to the URDF (JOINT_SIGNS).
# 5. follow() one short plan_line, then plot last_follow's commanded against
#    measured and tune TRACKING_GAIN until the worst tracking error is small
#    against the +/-5 mm budget.

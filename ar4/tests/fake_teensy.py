"""Stand-ins for the Teensy and the Nano, for testing backends/hw.py.

Not a mock that replays canned lines: FakeTeensy integrates the joints the way
AR4_teensy.ino's AccelStepper loop does -- every joint ramps at the firmware's
acceleration toward either a velocity (MV) or a position (MT), capped at its
max speed -- on a clock the backend shares. So a trajectory the firmware could
not track also fails to track here, and tracking error is a real measurement
of the control loop in hw.py, not an echo of it.

What it does NOT model: step quantisation, encoder resolution, serial
latency, load. A clean result here is a clean result against the firmware's
motion rules, nothing more.
"""
import numpy as np

from backends.hw import (DEFAULT_MODEL, FIRMWARE_VERSION, FW_MAX_ACCEL_DEG_S2,
                         FW_MAX_SPEED_DEG_S, JOINT_OFFSETS_BY_MODEL, NANO_VERSION)


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, s):
        self.now += max(0.0, s)


class FakeTeensy:
    SUBSTEP = 0.001

    def __init__(self, clock, fw_deg=None, model=DEFAULT_MODEL,
                 version=FIRMWARE_VERSION):
        self.clock = clock
        self.model_ok = model
        self.version = version
        self.p = np.zeros(6) if fw_deg is None else np.asarray(fw_deg, float).copy()
        self.v = np.zeros(6)
        self.mode = "pos"
        self.target = self.p.copy()
        self.vel_target = np.zeros(6)
        self.estop = False
        self.initialised = False
        self.t = clock()
        self.sent = []       # every line the host wrote, in order
        self.inject = []     # extra lines to emit before the next reply
        self._out = []
        self.closed = False

    # --- physics ---------------------------------------------------------

    def _advance(self):
        a, vmax, h = FW_MAX_ACCEL_DEG_S2, FW_MAX_SPEED_DEG_S, self.SUBSTEP
        while self.t + h <= self.clock():
            self.t += h
            if self.estop:
                self.v[:] = 0.0
                continue
            if self.mode == "vel":
                want = self.vel_target
            else:
                e = self.target - self.p
                # fastest speed from which it can still stop on the target
                want = np.sign(e) * np.minimum(vmax, np.sqrt(2.0 * a * np.abs(e)))
            dv = np.clip(want - self.v, -a * h, a * h)
            self.v = np.clip(self.v + dv, -vmax, vmax)
            self.p += self.v * h
            if self.mode == "pos":
                done = (np.abs(self.target - self.p) < 1e-4) & (np.abs(self.v) < a * h * 2)
                self.p[done] = self.target[done]
                self.v[done] = 0.0

    # --- serial ----------------------------------------------------------

    def write(self, data):
        self._advance()
        line = data.decode("ascii")
        self.sent.append(line)
        assert line.endswith("\n"), f"unterminated line {line!r}"
        body = line.strip()
        cmd = body[:2]
        self._out.extend(self.inject)
        self.inject = []

        if cmd == "ST":
            ver, model = body[3:].split("B", 1)
            ok_v, ok_m = int(ver == self.version), int(model == self.model_ok)
            self.initialised = bool(ok_v and ok_m)
            self._out.append(f"STA{ok_v}B{self.version}C{ok_m}D{model}")
            return
        if not self.initialised:
            self._out.append("ER: Unrecoverable error state entered. Please reset.")
            return
        if cmd == "JP":
            self._out.append("JP" + "".join(f"{c}{x:.6f}" for c, x in zip("ABCDEF", self.p)))
        elif cmd in ("MT", "MV"):
            vals = self._parse(body[2:])
            if cmd == "MT":
                self.mode, self.target = "pos", vals
            else:
                self.mode = "vel"
                self.vel_target = np.clip(vals, -FW_MAX_SPEED_DEG_S, FW_MAX_SPEED_DEG_S)
            self._out.append(f"ES{int(self.estop)}")
        elif cmd == "JC":
            # homed, then parked where the URDF reads ~zero
            self.p = -np.asarray(JOINT_OFFSETS_BY_MODEL[self.model_ok], float)
            self.v[:] = 0.0
            self.mode, self.target = "pos", self.p.copy()
            self._out.append("JCA0B0C0D0E0F0")
        elif cmd == "RE":
            self._out.append(f"ES{int(self.estop)}")
        else:
            self._out.append(f"ER: unknown {cmd}")

    @staticmethod
    def _parse(s):
        out, idx = [], [s.index(c) for c in "ABCDEF"] + [len(s)]
        for k in range(6):
            out.append(float(s[idx[k] + 1:idx[k + 1]]))
        return np.array(out)

    def readline(self):
        self._advance()
        return (self._out.pop(0) + "\r\n").encode("ascii") if self._out else b""

    def close(self):
        self.closed = True


class FakeNano:
    def __init__(self, version=NANO_VERSION):
        self.version = version
        self.sent = []
        self.angle = None
        self._out = []

    def write(self, data):
        line = data.decode("ascii").strip()
        self.sent.append(line)
        if line == "ST":
            self._out.append(self.version)
        elif line.startswith("SV0P"):
            self.angle = int(line[4:])
            self._out.append("Done")

    def readline(self):
        return (self._out.pop(0) + "\r\n").encode("ascii") if self._out else b""

    def close(self):
        pass

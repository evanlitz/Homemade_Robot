from __future__ import annotations

import math
from dataclasses import dataclass, field, fields, is_dataclass
from pathlib import Path
from typing import Any

# SI internally: metres, radians, newton-metres, kilograms, seconds.


@dataclass(frozen=True)
class MotorSpec:
    part: str
    holding_torque_nm: float
    current_a_per_phase: float
    full_step_deg: float
    mass_kg: float
    shaft_diameter_m: float
    rear_shaft: bool
    rotor_inertia_kgm2: float
    phase_resistance_ohm: float

    @property
    def full_steps_per_rev(self) -> float:
        return 360.0 / self.full_step_deg

    def dissipation_w(self, current_a: float) -> float:
        """Standstill dissipation, both phases energised: P = 2 I^2 R."""
        return 2.0 * current_a**2 * self.phase_resistance_ohm

    def current_for_torque_a(self, motor_torque_nm: float) -> float:
        """Phase current producing a given motor-shaft torque.

        ASSUMES torque is linear in current. True below magnetic saturation;
        at the currents this project uses (~25% of rated) it is a good
        approximation, but it is an approximation and wants a bench check.
        """
        return self.current_a_per_phase * motor_torque_nm / self.holding_torque_nm


@dataclass(frozen=True)
class BeltStage:
    """One belt reduction stage. Pitch diameters follow d = N*p/pi."""

    name: str
    pitch_mm: float
    driver_teeth: int
    driven_teeth: int
    width_mm: float
    profile: str
    centre_distance_mm: float

    @property
    def ratio(self) -> float:
        return self.driven_teeth / self.driver_teeth

    def pitch_dia_mm(self, teeth: int) -> float:
        return teeth * self.pitch_mm / math.pi

    def teeth_in_mesh(self) -> float:
        """Teeth engaged on the SMALL pulley.

        TIM = (N_small/360) * (180 - 2*asin((D-d)/(2C)))
        SDP/SI section 13.3 requires >= 6 on a load-carrying pulley.
        """
        big = self.pitch_dia_mm(max(self.driver_teeth, self.driven_teeth))
        small = self.pitch_dia_mm(min(self.driver_teeth, self.driven_teeth))
        half = (big - small) / (2.0 * self.centre_distance_mm)
        wrap = 180.0 - 2.0 * math.degrees(math.asin(min(1.0, half)))
        return min(self.driver_teeth, self.driven_teeth) * wrap / 360.0

    def mesh_derate(self) -> float:
        """SDP/SI 13.3: below 6 teeth in mesh, subtract 20% of the rating per
        missing tooth, floor of 2 teeth."""
        tim = self.teeth_in_mesh()
        if tim >= 6.0:
            return 1.0
        return max(0.0, 1.0 - 0.2 * (6.0 - math.floor(tim)))


@dataclass(frozen=True)
class DriveTrainSpec:
    """The belt stack, and the tension budget it can carry.

    `rating_n_per_inch` is manufacturer allowable working tension per 25.4 mm
    of belt width. Source recorded in DECISIONS.md F7.
    """

    stages: tuple[BeltStage, ...]
    rating_n_per_inch: dict[str, float]

    @property
    def total_ratio(self) -> float:
        r = 1.0
        for s in self.stages:
            r *= s.ratio
        return r

    def allowable_tension_n(self, stage: BeltStage) -> float:
        return (self.rating_n_per_inch[stage.profile] * stage.width_mm / 25.4
                * stage.mesh_derate())

    def stage_tension_n(self, stage: BeltStage, driver_torque_nm: float) -> float:
        """Effective tension from the DRIVER side, which is the conservative
        reading: the driven side is lower by the stage efficiency."""
        return driver_torque_nm / (self.pitch_radius_m(stage, stage.driver_teeth))

    @staticmethod
    def pitch_radius_m(stage: BeltStage, teeth: int) -> float:
        return stage.pitch_dia_mm(teeth) / 2000.0


@dataclass(frozen=True)
class DriveSpec:
    kind: str
    reduction: float
    microsteps: float  # float, not int, so it can carry a Placeholder marker
    efficiency: float
    # Set ONLY for a prismatic joint: metres of linear travel per revolution of
    # the reduction OUTPUT (pulley circumference, or leadscrew lead). Its
    # presence is what makes this drive linear, so every unit-carrying method
    # below branches on it rather than on a separate `kind` string that could
    # disagree with it.
    lead_m_per_rev: float | None = None

    @property
    def is_linear(self) -> bool:
        return self.lead_m_per_rev is not None

    def output_per_full_step(self, motor: MotorSpec) -> float:
        """Joint output per motor full step. RADIANS if rotary, METRES if linear.

        Mixed units in one return value is deliberate: it is exactly what
        `||J_column|| * step` needs, because a prismatic Jacobian column is a
        unit vector and a revolute one carries the moment arm. Callers that
        multiply by a Jacobian column norm get metres either way.
        """
        if self.is_linear:
            return self.lead_m_per_rev / (motor.full_steps_per_rev * self.reduction)
        return math.radians(motor.full_step_deg) / self.reduction

    def joint_rad_per_full_step(self, motor: MotorSpec) -> float:
        if self.is_linear:
            raise ValueError("linear drive: use output_per_full_step, which "
                             "returns metres")
        return math.radians(motor.full_step_deg) / self.reduction

    def joint_torque_limit_nm(self, motor: MotorSpec) -> float:
        return motor.holding_torque_nm * self.reduction * self.efficiency

    def reflected_inertia_kgm2(self, motor: MotorSpec) -> float:
        """Rotor inertia seen at the joint. MuJoCo `armature` units.

        Rotary: J_rotor * N^2, kg.m^2. Scales with the SQUARE of the reduction,
        so at 1:25 it is 625x the rotor's own figure and comparable to the whole
        arm's inertia. Omitting it is not a small error -- DECISIONS.md F6.

        Linear: the same energy argument gives a reflected MASS in kg,
        J_rotor * (2*pi*N/lead)^2, which is what MuJoCo's armature means on a
        slide joint. The `2*pi/lead` term makes this large for any fine lead --
        it is not a bug, and it is the same physics F6 is about.
        """
        if self.is_linear:
            return motor.rotor_inertia_kgm2 * (
                2.0 * math.pi * self.reduction / self.lead_m_per_rev) ** 2
        return motor.rotor_inertia_kgm2 * self.reduction**2

    def motor_torque_for_joint_nm(self, joint_torque_nm: float) -> float:
        """Motor-shaft torque for a given joint-side load.

        Linear: the argument is a FORCE in newtons and the screw relation
        `tau = F * lead / (2*pi * N * eta)` applies. This is the only quantity
        that compares a prismatic axis with a revolute one honestly -- the
        generalised force at a fine-lead screw is large and cheap, so quoting
        newtons next to newton-metres invites exactly the wrong conclusion.
        """
        if self.is_linear:
            return (joint_torque_nm * self.lead_m_per_rev
                    / (2.0 * math.pi * self.reduction * self.efficiency))
        return joint_torque_nm / (self.reduction * self.efficiency)


@dataclass(frozen=True)
class LinkSpec:
    name: str
    length_m: float
    mass_kg: float


@dataclass(frozen=True)
class JointSpec:
    name: str
    min_rad: float
    max_rad: float
    max_vel_rad_s: float
    max_accel_rad_s2: float
    drive: DriveSpec
    # An ADDITIONAL constraint, not a replacement. `min_rad`/`max_rad` are
    # always the absolute, world-referenced limits; when `relative_to` is set,
    # `rel_min_rad`/`rel_max_rad` also apply to the difference from that joint.
    # A pose must satisfy BOTH.
    #
    # Both exist because they are different physical facts. An elbow's travel is
    # a property of the mechanism and does not care where the arm points, so it
    # is relative. A vertical-plane arm's clearance over the table is a property
    # of the world, so it is absolute. Substituting one for the other loosens
    # the model in whichever direction the substitution happens to run -- doing
    # exactly that raised the palletizer's survivor count from 7 to 16 on the
    # first attempt at D19, by silently discarding its table-clearance limit.
    relative_to: str | None = None
    rel_min_rad: float | None = None
    rel_max_rad: float | None = None

    @property
    def range_rad(self) -> tuple[float, float]:
        return (self.min_rad, self.max_rad)


@dataclass(frozen=True)
class TaskSpec:
    board_square_m: float
    board_span_m: float
    reach_requirement_m: float
    accuracy_target_m: float


@dataclass(frozen=True)
class ValiditySpec:
    """Hard gates. A geometry violating any of these is REJECTED by the sweep,
    not ranked and reported. DECISIONS.md OPEN-A, D11.

    Coverage rewards exactly the direction that breaks the tension and
    resolution budgets -- longer links, board pushed outward -- so a sweep that
    ranks violations post hoc hands back an optimum that is invalid.
    """

    residual_invalid_m: float
    residual_target_m: float
    joint_torque_cap_nm: float
    tool_m_per_full_step_max: float
    base_clearance_m: float

    def verdict(self, residual_m: float) -> str:
        if residual_m > self.residual_invalid_m:
            return "INVALID"
        return "ok" if residual_m <= self.residual_target_m else "ABOVE-TARGET"

    def rejections(self, *, residual_m: float | None = None,
                   peak_torque_nm: float | None = None,
                   worst_resolution_m: float | None = None,
                   worst_tension_n: float | None = None,
                   allowable_tension_n: float | None = None,
                   board_gap_m: float | None = None) -> list[str]:
        out: list[str] = []
        if board_gap_m is not None and board_gap_m < self.base_clearance_m:
            out.append(f"base clearance {board_gap_m * 1000:.0f} mm < "
                       f"{self.base_clearance_m * 1000:.0f} mm")
        if residual_m is not None and residual_m > self.residual_invalid_m:
            out.append(f"residual {residual_m * 1e6:.1f} um > "
                       f"{self.residual_invalid_m * 1e6:.0f} um")
        if peak_torque_nm is not None and peak_torque_nm > self.joint_torque_cap_nm:
            out.append(f"torque {peak_torque_nm:.1f} N.m > cap "
                       f"{self.joint_torque_cap_nm:.1f} N.m")
        if worst_resolution_m is not None and worst_resolution_m > self.tool_m_per_full_step_max:
            out.append(f"resolution {worst_resolution_m * 1000:.3f} mm/step > "
                       f"{self.tool_m_per_full_step_max * 1000:.3f} mm/step")
        if (worst_tension_n is not None and allowable_tension_n is not None
                and worst_tension_n > allowable_tension_n):
            out.append(f"belt tension {worst_tension_n:.0f} N > allowable "
                       f"{allowable_tension_n:.0f} N")
        return out


@dataclass(frozen=True)
class SweepAxis:
    """One coarse sweep axis. DECISIONS.md D8, D9 -- swept, not chosen."""

    name: str
    min: float
    max: float
    steps: int

    def values(self) -> list[float]:
        if self.steps == 1:
            return [(self.min + self.max) / 2.0]
        span = (self.max - self.min) / (self.steps - 1)
        return [self.min + i * span for i in range(self.steps)]


@dataclass(frozen=True)
class ArchitectureConfig:
    name: str
    candidate_id: int | None
    closed_loop: bool
    description: str
    motor: MotorSpec
    links: tuple[LinkSpec, ...]
    joints: tuple[JointSpec, ...]
    task: TaskSpec
    # Architecture-specific geometry, keyed by name. Serial arms use `links`;
    # a closed-loop mechanism's geometry is not a list of links, so it goes here.
    geometry: dict[str, float] = field(default_factory=dict)
    validity: ValiditySpec | None = None
    drive_train: DriveTrainSpec | None = None
    sweep: tuple[SweepAxis, ...] = ()
    gripper: dict[str, float] = field(default_factory=dict)
    source: Path | None = field(default=None, compare=False)

    def g(self, key: str) -> float:
        return self.geometry[key]

    @classmethod
    def from_dict(cls, raw: dict[str, Any], source: Path | None = None) -> ArchitectureConfig:
        motor = MotorSpec(**raw["motor"])
        links = tuple(LinkSpec(**d) for d in raw.get("links", []))
        joints = tuple(
            JointSpec(
                name=d["name"],
                min_rad=d["min_rad"],
                max_rad=d["max_rad"],
                max_vel_rad_s=d["max_vel_rad_s"],
                max_accel_rad_s2=d["max_accel_rad_s2"],
                drive=DriveSpec(**d["drive"]),
                relative_to=d.get("relative_to"),
                rel_min_rad=d.get("rel_min_rad"),
                rel_max_rad=d.get("rel_max_rad"),
            )
            for d in raw["joints"]
        )
        return cls(
            name=raw["name"],
            candidate_id=raw.get("candidate_id"),
            closed_loop=raw["closed_loop"],
            description=raw["description"],
            motor=motor,
            links=links,
            joints=joints,
            task=TaskSpec(**raw["task"]),
            geometry=dict(raw.get("geometry", {})),
            validity=ValiditySpec(**raw["validity"]) if "validity" in raw else None,
            drive_train=(
                DriveTrainSpec(
                    stages=tuple(BeltStage(**s) for s in raw["drive_train"]["stages"]),
                    rating_n_per_inch=dict(raw["drive_train"]["rating_n_per_inch"]),
                )
                if "drive_train" in raw else None
            ),
            sweep=tuple(SweepAxis(name=k, **v) for k, v in raw.get("sweep", {}).items()),
            gripper=dict(raw.get("gripper", {})),
            source=source,
        )

    def placeholder_fields(self) -> list[str]:
        from .loader import Placeholder

        found: list[str] = []

        def walk(obj: Any, path: str) -> None:
            if isinstance(obj, Placeholder):
                found.append(f"{path} = {float(obj)}")
            elif is_dataclass(obj):
                name = getattr(obj, "name", None)
                label = f"{path}[{name}]" if name and path else path
                for f in fields(obj):
                    walk(getattr(obj, f.name), f"{label}.{f.name}" if label else f.name)
            elif isinstance(obj, dict):
                for key, value in obj.items():
                    walk(value, f"{path}.{key}" if path else key)
            elif isinstance(obj, (tuple, list)):
                for item in obj:
                    walk(item, path)

        walk(self, "")
        return found

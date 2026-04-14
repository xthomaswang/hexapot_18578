"""
Hexapod robot configuration.

All lengths are millimeters and all angles are degrees.
"""

from __future__ import annotations

from dataclasses import dataclass, field


LEG_COUNT = 6
LEG_ORDER = ("L1", "L2", "L3", "R1", "R2", "R3")


@dataclass(frozen=True)
class GeometryConfig:
    """Rigid-body geometry shared by IK, visualization, and simulation export. length unit:(mm)"""

    coxa_length: float = 254.0
    femur_length: float = 228.6
    tibia_length: float = 594.4
    body_length: float = 812.8
    body_width: float = 381.0

    @property
    def reach(self) -> float:
        return self.coxa_length + self.femur_length + self.tibia_length


@dataclass(frozen=True)
class LegMount:
    """One leg mounting origin on the body frame."""

    x: float
    y: float
    angle_deg: float
    name: str


def _clamp_body_x(x: float, half_body_length: float) -> float:
    return max(-half_body_length, min(half_body_length, x))


@dataclass(frozen=True)
class LayoutConfig:
    """
    Rectangular body layout.

    Default behavior:
    - four outer legs sit near the rectangle corners
    - the two middle legs sit on the left/right side centers

    `middle_left_x_offset` and `middle_right_x_offset` are signed distances
    from the body center along the body X axis:
    - positive = move toward the front
    - negative = move toward the rear
    """

    corner_x_inset: float = 50.0
    middle_left_x_offset: float = 0.0
    middle_right_x_offset: float = 0.0

    def build_leg_mounts(self, geometry: GeometryConfig) -> tuple[LegMount, ...]:
        half_l = geometry.body_length / 2.0
        half_w = geometry.body_width / 2.0
        corner_x = max(0.0, min(half_l, half_l - self.corner_x_inset))
        mid_left_x = _clamp_body_x(self.middle_left_x_offset, half_l)
        mid_right_x = _clamp_body_x(self.middle_right_x_offset, half_l)

        return (
            LegMount(corner_x, half_w, 90.0, "L1"),
            LegMount(mid_left_x, half_w, 90.0, "L2"),
            LegMount(-corner_x, half_w, 90.0, "L3"),
            LegMount(corner_x, -half_w, -90.0, "R1"),
            LegMount(mid_right_x, -half_w, -90.0, "R2"),
            LegMount(-corner_x, -half_w, -90.0, "R3"),
        )


@dataclass(frozen=True)
class ServoConfig:
    """Servo limits and calibration."""

    neutral_deg: float = 90.0
    effective_travel_deg: float = 120.0
    pwm_min_us: int = 1000
    pwm_max_us: int = 2000
    pwm_neutral_us: int = 1520
    servo_map: dict[int, tuple[int, int, int]] = field(
        default_factory=lambda: {
            0: (0, 1, 2),
            1: (3, 4, 5),
            2: (6, 7, 8),
            3: (9, 10, 11),
            4: (12, 13, 14),
            5: (15, 16, 17),
        }
    )
    direction: dict[int, tuple[int, int, int]] = field(
        default_factory=lambda: {
            0: (1, -1, 1),
            1: (1, -1, 1),
            2: (1, -1, 1),
            3: (-1, -1, 1),
            4: (-1, -1, 1),
            5: (-1, -1, 1),
        }
    )
    offset_deg: dict[int, tuple[float, float, float]] = field(
        default_factory=lambda: {
            0: (0.0, 0.0, 0.0),
            1: (0.0, 0.0, 0.0),
            2: (0.0, 0.0, 0.0),
            3: (0.0, 0.0, 0.0),
            4: (0.0, 0.0, 0.0),
            5: (0.0, 0.0, 0.0),
        }
    )
    saved_legs: dict[int, bool] = field(
        default_factory=lambda: {i: False for i in range(LEG_COUNT)}
    )

    @property
    def safe_range_deg(self) -> tuple[float, float]:
        half_travel = self.effective_travel_deg / 2.0
        return (self.neutral_deg - half_travel, self.neutral_deg + half_travel)

    def is_saved(self, leg_index: int) -> bool:
        """Return True when the given leg has a user-saved calibration.

        Saved legs bypass the conservative safe-range clamp in
        ``leg_ik.ik_to_servo_angles`` so the user's stored neutral pose is
        honored verbatim.
        """
        return bool(self.saved_legs.get(leg_index, False))


@dataclass(frozen=True)
class HexapodConfig:
    """Top-level robot configuration."""

    geometry: GeometryConfig = field(default_factory=GeometryConfig)
    layout: LayoutConfig = field(default_factory=LayoutConfig)
    servo: ServoConfig = field(default_factory=ServoConfig)
    default_foot_outward_offset: float = 200.0
    default_foot_z: float = -500.0

    @property
    def default_foot_x(self) -> float:
        return self.geometry.coxa_length + self.default_foot_outward_offset

    @property
    def leg_mounts(self) -> tuple[LegMount, ...]:
        return self.layout.build_leg_mounts(self.geometry)


def build_hexapod_config(
    *,
    corner_x_inset: float = 50.0,
    middle_left_x_offset: float = 0.0,
    middle_right_x_offset: float = 0.0,
    geometry: GeometryConfig | None = None,
    servo: ServoConfig | None = None,
    default_foot_outward_offset: float | None = None,
    default_foot_z: float | None = None,
) -> HexapodConfig:
    """
    Convenience builder for the customizable rectangular layout.

    When geometry is provided but foot-position defaults are not,
    the standing pose is auto-derived so the foot sits at ~25% of the
    2-DOF reach outward and ~60% downward.  This means changing leg
    dimensions is a one-liner — no manual foot-position tuning needed.
    """
    geo = geometry or GeometryConfig()

    # Auto-derive standing pose from leg dimensions
    if default_foot_outward_offset is None:
        if geometry is not None:
            default_foot_outward_offset = (geo.femur_length + geo.tibia_length) * 0.25
        else:
            default_foot_outward_offset = 200.0

    if default_foot_z is None:
        if geometry is not None:
            default_foot_z = -(geo.femur_length + geo.tibia_length) * 0.60
        else:
            default_foot_z = -500.0

    return HexapodConfig(
        geometry=geo,
        layout=LayoutConfig(
            corner_x_inset=corner_x_inset,
            middle_left_x_offset=middle_left_x_offset,
            middle_right_x_offset=middle_right_x_offset,
        ),
        servo=servo or ServoConfig(),
        default_foot_outward_offset=default_foot_outward_offset,
        default_foot_z=default_foot_z,
    )


DEFAULT_CONFIG = HexapodConfig()

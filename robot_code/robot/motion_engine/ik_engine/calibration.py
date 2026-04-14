"""
Calibration profile loading and application.

This module is the single source of truth for how calibration profiles are
read from disk and merged into a ``ServoConfig``. It replaces the scattered
``load_calibration_offsets`` / ``load_calibration_saved_mask`` helpers that
used to live in ``runtime/runtime.py`` — the profile is now parsed exactly
once per ``build_motion_controller`` call.

Layering:
    - ``ik_engine/config.py`` stays pure math (no JSON IO).
    - ``ik_engine/calibration.py`` owns the JSON shape and its projection
      into a calibrated ``ServoConfig``.
    - Callers (runtime, web) should only touch ``CalibrationProfile`` and
      never rehydrate ``load_calibration_offsets`` style helpers.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, Mapping

from .config import LEG_COUNT, ServoConfig


_JOINT_KEYS: tuple[str, str, str] = ("coxa_deg", "femur_deg", "tibia_deg")

PROFILE_SCHEMA_VERSION = 2


@dataclass(frozen=True)
class CalibrationLegProfile:
    """Per-leg calibration entry parsed from a profile file.

    ``direction`` is an optional v2 field that, when set, overrides the
    base ``ServoConfig.direction[leg_index]`` tuple. ``None`` means
    "inherit whatever direction the base ``ServoConfig`` provides".

    NOTE: PLAN v3 §5 proposed a "smallest |offset|" heuristic for
    auto-detecting direction at save time. The math does not work out:
    under ``D = +1`` and ``D = -1`` the offset magnitudes are always
    equal (only the sign differs), so magnitude cannot distinguish the
    two cases. Auto-detect is therefore NOT implemented in this pass.
    The v2 schema is still shipped so that a manually-authored profile
    or a future heuristic can fill in ``direction`` without another
    schema bump.
    """

    leg_index: int
    saved: bool = False
    offsets_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)
    direction: tuple[int, int, int] | None = None


@dataclass(frozen=True)
class CalibrationProfile:
    """Full 6-leg calibration profile parsed from JSON."""

    version: int = 1
    legs: tuple[CalibrationLegProfile, ...] = field(default_factory=tuple)

    @classmethod
    def default(cls) -> "CalibrationProfile":
        return cls(
            version=1,
            legs=tuple(
                CalibrationLegProfile(leg_index=i) for i in range(LEG_COUNT)
            ),
        )

    def leg(self, leg_index: int) -> CalibrationLegProfile:
        for entry in self.legs:
            if entry.leg_index == leg_index:
                return entry
        return CalibrationLegProfile(leg_index=leg_index)

    def is_saved(self, leg_index: int) -> bool:
        return self.leg(leg_index).saved


def _coerce_offsets(raw: Any) -> tuple[float, float, float]:
    if not isinstance(raw, Mapping):
        return (0.0, 0.0, 0.0)
    try:
        return (
            float(raw.get(_JOINT_KEYS[0], 0.0)),
            float(raw.get(_JOINT_KEYS[1], 0.0)),
            float(raw.get(_JOINT_KEYS[2], 0.0)),
        )
    except (TypeError, ValueError):
        return (0.0, 0.0, 0.0)


def _coerce_direction(raw: Any) -> tuple[int, int, int] | None:
    """Parse a v2 direction field into a 3-tuple of ±1, or None."""
    if raw is None:
        return None
    if isinstance(raw, Mapping):
        try:
            values = (
                int(raw.get(_JOINT_KEYS[0], 0)),
                int(raw.get(_JOINT_KEYS[1], 0)),
                int(raw.get(_JOINT_KEYS[2], 0)),
            )
        except (TypeError, ValueError):
            return None
    elif isinstance(raw, (list, tuple)) and len(raw) == 3:
        try:
            values = (int(raw[0]), int(raw[1]), int(raw[2]))
        except (TypeError, ValueError):
            return None
    else:
        return None
    for v in values:
        if v not in (-1, 1):
            return None
    return values


def _parse_leg_payload(raw: Any, leg_index: int) -> CalibrationLegProfile:
    if not isinstance(raw, Mapping):
        return CalibrationLegProfile(leg_index=leg_index)

    saved = bool(raw.get("saved", False))
    offsets = _coerce_offsets(raw.get("offsets_deg")) if saved else (0.0, 0.0, 0.0)
    direction = _coerce_direction(raw.get("direction"))
    return CalibrationLegProfile(
        leg_index=leg_index,
        saved=saved,
        offsets_deg=offsets,
        direction=direction,
    )


def load_profile(path: str | Path | None) -> CalibrationProfile:
    """Read a calibration profile JSON into a ``CalibrationProfile``.

    Missing, unreadable, or malformed files yield the default (all-unsaved,
    zero-offset) profile so that runtime startup is never blocked by a bad
    or absent calibration file.
    """
    if not path:
        return CalibrationProfile.default()

    try:
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
    except (FileNotFoundError, IsADirectoryError, OSError, json.JSONDecodeError):
        return CalibrationProfile.default()

    if not isinstance(payload, Mapping):
        return CalibrationProfile.default()

    try:
        version = int(payload.get("version", 1))
    except (TypeError, ValueError):
        version = 1

    raw_legs = payload.get("legs")
    if not isinstance(raw_legs, list):
        raw_legs = []

    legs: list[CalibrationLegProfile] = []
    for leg_index in range(LEG_COUNT):
        raw = raw_legs[leg_index] if leg_index < len(raw_legs) else None
        legs.append(_parse_leg_payload(raw, leg_index))

    return CalibrationProfile(version=version, legs=tuple(legs))


def apply_to_servo_config(
    profile: CalibrationProfile,
    base: ServoConfig,
) -> ServoConfig:
    """Merge a ``CalibrationProfile`` into a base ``ServoConfig``.

    For each leg:
        - saved legs take their ``offsets_deg`` from the profile and have
          ``saved_legs[leg_index] = True``
        - unsaved legs are forced to zero offsets and ``saved_legs = False``
        - if a saved leg has a v2 ``direction`` override, it replaces
          ``base.direction[leg_index]``; otherwise the base default stands

    ``servo_map`` is not modified.
    """
    offsets: dict[int, tuple[float, float, float]] = dict(base.offset_deg)
    saved: dict[int, bool] = dict(base.saved_legs)
    directions: dict[int, tuple[int, int, int]] = dict(base.direction)

    for leg_index in range(LEG_COUNT):
        leg = profile.leg(leg_index)
        if leg.saved:
            offsets[leg_index] = leg.offsets_deg
            saved[leg_index] = True
            if leg.direction is not None:
                directions[leg_index] = leg.direction
        else:
            offsets[leg_index] = (0.0, 0.0, 0.0)
            saved[leg_index] = False

    return replace(
        base,
        offset_deg=offsets,
        saved_legs=saved,
        direction=directions,
    )


def save_profile(path: str | Path, profile: CalibrationProfile) -> None:
    """Atomically write a ``CalibrationProfile`` to disk in v2 format.

    Uses the standard tmpfile + os.replace pattern so a crash mid-write
    cannot leave a half-written profile visible to the runtime poller.
    """
    import os
    from tempfile import NamedTemporaryFile

    payload: dict[str, Any] = {
        "version": PROFILE_SCHEMA_VERSION,
        "legs": [],
    }
    for leg in profile.legs:
        entry: dict[str, Any] = {
            "leg_index": leg.leg_index,
            "saved": leg.saved,
            "offsets_deg": {
                _JOINT_KEYS[0]: leg.offsets_deg[0],
                _JOINT_KEYS[1]: leg.offsets_deg[1],
                _JOINT_KEYS[2]: leg.offsets_deg[2],
            },
        }
        if leg.direction is not None:
            entry["direction"] = {
                _JOINT_KEYS[0]: leg.direction[0],
                _JOINT_KEYS[1]: leg.direction[1],
                _JOINT_KEYS[2]: leg.direction[2],
            }
        payload["legs"].append(entry)

    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    with NamedTemporaryFile(
        mode="w",
        dir=str(target.parent),
        prefix=target.name + ".",
        suffix=".tmp",
        delete=False,
        encoding="utf-8",
    ) as tmp:
        json.dump(payload, tmp, indent=2)
        tmp_path = tmp.name
    os.replace(tmp_path, target)

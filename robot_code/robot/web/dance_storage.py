"""Persistence for user-authored dance presets.

A "dance" is an ordered list of states. Each state records per-leg (from, to)
servo angles — from is locked to the previous state's to (or calibrated
neutral for state 0), so only the editor writes it. Legs not edited in a
state have from == to (hold).

Dances are stored as JSON under ``dance_engine/storage/auto/<dir>/dance.json``
with a sibling ``index.json`` that drives the automated preset list.

Tag binding rules (V1):
- Tag 1 is hardwired to the runner's stop behaviour; dances cannot bind it.
- Any other tag id is globally unique across all saved dances; rebinding
  requires the old binding to be released first.
- Dances may have tag_id = None (callable from the manual button only).
"""

from __future__ import annotations

import json
import os
import re
import shutil
import threading
import time
from pathlib import Path
from typing import Any, Mapping


# Path shared with the existing manual dance recordings:
#   dance_engine/storage/auto/   — user-authored preset dances (this module)
#   dance_engine/storage/manual/ — manual-mode XY timelines (dance_recording.py)
DEFAULT_DANCE_ROOT = Path(__file__).resolve().parents[1] / "dance_engine" / "storage" / "auto"
INDEX_FILENAME = "index.json"
DANCE_FILENAME = "dance.json"

SCHEMA_VERSION = 1
NUM_LEGS = 6
JOINTS_PER_LEG = 3
NUM_SERVOS = NUM_LEGS * JOINTS_PER_LEG

# Tag 1 is reserved for the runner's built-in stop action (see
# automated_runner.TAG_STOP); no saved dance may bind it.
RESERVED_STOP_TAG = 1

DEFAULT_SEGMENT_MS = 440
MIN_SEGMENT_MS = 60
MAX_SEGMENT_MS = 10_000


class DanceError(RuntimeError):
    """Base error for dance-storage operations."""


class DanceValidationError(DanceError):
    """Payload shape or value is wrong."""


class DancePresetNotFound(DanceError):
    """A lookup for preset_id failed."""


class TagConflict(DanceError):
    """Attempted to bind a tag already owned by another preset, or the reserved stop tag."""


_SLUG_RE = re.compile(r"[^a-z0-9]+")


def slugify(name: str) -> str:
    slug = _SLUG_RE.sub("-", name.lower()).strip("-")
    return slug or "dance"


def _canonical_preset_id(name: str, raw_preset_id: Any) -> str:
    source = name if raw_preset_id is None or raw_preset_id == "" else str(raw_preset_id)
    return slugify(source)


def _coerce_float(value: Any, *, field: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise DanceValidationError(f"{field} must be a number") from exc


def _coerce_int(value: Any, *, field: str) -> int:
    try:
        return int(value)
    except (TypeError, ValueError) as exc:
        raise DanceValidationError(f"{field} must be an integer") from exc


def _validate_angle_triplet(payload: Mapping[str, Any], *, field: str) -> dict[str, float]:
    if not isinstance(payload, Mapping):
        raise DanceValidationError(f"{field} must be an object")
    out: dict[str, float] = {}
    for key in ("coxa_deg", "femur_deg", "tibia_deg"):
        if key not in payload:
            raise DanceValidationError(f"{field} missing {key}")
        v = _coerce_float(payload[key], field=f"{field}.{key}")
        if not 0.0 <= v <= 180.0:
            raise DanceValidationError(f"{field}.{key} must be within 0..180, got {v}")
        out[key] = v
    return out


def _validate_state(raw: Mapping[str, Any], *, state_index: int) -> dict[str, Any]:
    if not isinstance(raw, Mapping):
        raise DanceValidationError(f"state[{state_index}] must be an object")
    legs_raw = raw.get("legs")
    if not isinstance(legs_raw, list) or len(legs_raw) != NUM_LEGS:
        raise DanceValidationError(
            f"state[{state_index}].legs must be a list of {NUM_LEGS} entries"
        )
    legs: list[dict[str, Any]] = []
    for leg_index in range(NUM_LEGS):
        leg_raw = legs_raw[leg_index]
        if not isinstance(leg_raw, Mapping):
            raise DanceValidationError(
                f"state[{state_index}].legs[{leg_index}] must be an object"
            )
        if int(leg_raw.get("leg_index", leg_index)) != leg_index:
            raise DanceValidationError(
                f"state[{state_index}].legs[{leg_index}].leg_index mismatch"
            )
        legs.append(
            {
                "leg_index": leg_index,
                "edited": bool(leg_raw.get("edited", False)),
                "from": _validate_angle_triplet(
                    leg_raw.get("from", {}),
                    field=f"state[{state_index}].legs[{leg_index}].from",
                ),
                "to": _validate_angle_triplet(
                    leg_raw.get("to", {}),
                    field=f"state[{state_index}].legs[{leg_index}].to",
                ),
            }
        )
    duration_ms = _coerce_int(
        raw.get("duration_ms", DEFAULT_SEGMENT_MS),
        field=f"state[{state_index}].duration_ms",
    )
    if not MIN_SEGMENT_MS <= duration_ms <= MAX_SEGMENT_MS:
        raise DanceValidationError(
            f"state[{state_index}].duration_ms must be within {MIN_SEGMENT_MS}..{MAX_SEGMENT_MS}"
        )
    return {
        "index": state_index,
        "duration_ms": duration_ms,
        "legs": legs,
    }


def _validate_dance(raw: Mapping[str, Any]) -> dict[str, Any]:
    if not isinstance(raw, Mapping):
        raise DanceValidationError("dance must be an object")
    name = str(raw.get("name") or "").strip()
    if not name:
        raise DanceValidationError("name is required")
    preset_id = _canonical_preset_id(name, raw.get("preset_id"))
    if not preset_id:
        raise DanceValidationError("preset_id could not be derived from name")
    states_raw = raw.get("states")
    if not isinstance(states_raw, list) or not states_raw:
        raise DanceValidationError("states must be a non-empty list")
    states: list[dict[str, Any]] = []
    for i, s in enumerate(states_raw):
        states.append(_validate_state(s, state_index=i))
    for i in range(1, len(states)):
        for leg_index in range(NUM_LEGS):
            prev_to = states[i - 1]["legs"][leg_index]["to"]
            cur_from = states[i]["legs"][leg_index]["from"]
            for key in ("coxa_deg", "femur_deg", "tibia_deg"):
                if abs(prev_to[key] - cur_from[key]) > 1e-3:
                    raise DanceValidationError(
                        f"state[{i}].legs[{leg_index}].from[{key}] must equal "
                        f"state[{i-1}].legs[{leg_index}].to[{key}]"
                    )
    tag_raw = raw.get("tag_id")
    if tag_raw is None:
        tag_id: int | None = None
    else:
        tag_id = _coerce_int(tag_raw, field="tag_id")
        if tag_id == RESERVED_STOP_TAG:
            raise TagConflict(f"tag {RESERVED_STOP_TAG} is reserved for stop")
        if tag_id <= 0:
            raise DanceValidationError("tag_id must be a positive integer")
    return {
        "schema_version": SCHEMA_VERSION,
        "name": name,
        "preset_id": preset_id,
        "tag_id": tag_id,
        "default_segment_ms": _coerce_int(
            raw.get("default_segment_ms", DEFAULT_SEGMENT_MS),
            field="default_segment_ms",
        ),
        "calibration_snapshot": {
            "angles": [float(v) for v in (raw.get("calibration_snapshot") or {}).get("angles", [])],
        },
        "states": states,
        "created_at": float(raw.get("created_at") or time.time()),
        "updated_at": time.time(),
    }


def _read_json(path: Path) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, OSError, json.JSONDecodeError):
        return None


def _atomic_write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    os.replace(tmp, path)


class DanceStorage:
    """Thread-safe file-backed preset storage.

    The index is the source of truth for "which dances exist and what tag
    each one owns". Dance files live under ``<root>/<dir>/dance.json``.
    """

    def __init__(self, root: Path | str = DEFAULT_DANCE_ROOT) -> None:
        self.root = Path(root)
        self._lock = threading.RLock()

    # ----- index helpers ----------------------------------------------------

    def _index_path(self) -> Path:
        return self.root / INDEX_FILENAME

    def _load_index_locked(self) -> list[dict[str, Any]]:
        raw = _read_json(self._index_path())
        if not isinstance(raw, list):
            return []
        cleaned: list[dict[str, Any]] = []
        for entry in raw:
            if not isinstance(entry, dict):
                continue
            preset_id = str(entry.get("preset_id") or "")
            if not preset_id:
                continue
            cleaned.append({
                "preset_id": preset_id,
                "name": str(entry.get("name") or preset_id),
                "tag_id": entry.get("tag_id") if isinstance(entry.get("tag_id"), int) else None,
                "dir": str(entry.get("dir") or ""),
                "created_at": float(entry.get("created_at") or 0.0),
                "updated_at": float(entry.get("updated_at") or 0.0),
            })
        return cleaned

    def _save_index_locked(self, index: list[dict[str, Any]]) -> None:
        _atomic_write_json(self._index_path(), index)

    def _check_tag_locked(
        self,
        index: list[dict[str, Any]],
        tag_id: int | None,
        *,
        exclude_preset: str | None,
    ) -> None:
        if tag_id is None:
            return
        if tag_id == RESERVED_STOP_TAG:
            raise TagConflict(f"tag {RESERVED_STOP_TAG} is reserved for stop")
        for entry in index:
            if entry["preset_id"] == exclude_preset:
                continue
            if entry["tag_id"] == tag_id:
                raise TagConflict(
                    f"tag {tag_id} already bound to preset '{entry['preset_id']}'"
                )

    # ----- public api -------------------------------------------------------

    def list_presets(self) -> list[dict[str, Any]]:
        with self._lock:
            return self._load_index_locked()

    def get_dance(self, preset_id: str) -> dict[str, Any]:
        with self._lock:
            index = self._load_index_locked()
            for entry in index:
                if entry["preset_id"] == preset_id:
                    path = self.root / entry["dir"] / DANCE_FILENAME
                    raw = _read_json(path)
                    if raw is None:
                        raise DancePresetNotFound(
                            f"dance file missing for preset '{preset_id}' at {path}"
                        )
                    return _validate_dance(raw)
        raise DancePresetNotFound(f"preset '{preset_id}' not in index")

    def lookup_by_tag(self, tag_id: int) -> dict[str, Any] | None:
        with self._lock:
            for entry in self._load_index_locked():
                if entry["tag_id"] == tag_id:
                    return entry
        return None

    def save_dance(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        validated = _validate_dance(payload)
        preset_id = validated["preset_id"]
        with self._lock:
            index = self._load_index_locked()
            existing = next((e for e in index if e["preset_id"] == preset_id), None)
            self._check_tag_locked(
                index, validated["tag_id"], exclude_preset=preset_id,
            )
            if existing is not None:
                dir_name = existing["dir"]
                validated["created_at"] = existing["created_at"]
            else:
                stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(validated["created_at"]))
                dir_name = f"{stamp}_{preset_id}"
                base_dir = self.root / dir_name
                suffix = 2
                while base_dir.exists():
                    dir_name = f"{stamp}_{preset_id}_{suffix}"
                    base_dir = self.root / dir_name
                    suffix += 1
            dance_path = self.root / dir_name / DANCE_FILENAME
            _atomic_write_json(dance_path, validated)
            # Update index entry
            new_entry = {
                "preset_id": preset_id,
                "name": validated["name"],
                "tag_id": validated["tag_id"],
                "dir": dir_name,
                "created_at": validated["created_at"],
                "updated_at": validated["updated_at"],
            }
            if existing is not None:
                index = [e for e in index if e["preset_id"] != preset_id]
            index.append(new_entry)
            index.sort(key=lambda e: e["created_at"])
            self._save_index_locked(index)
            return new_entry

    def delete_preset(self, preset_id: str) -> dict[str, Any]:
        with self._lock:
            index = self._load_index_locked()
            entry = next((e for e in index if e["preset_id"] == preset_id), None)
            if entry is None:
                raise DancePresetNotFound(f"preset '{preset_id}' not found")
            dir_path = self.root / entry["dir"]
            shutil.rmtree(dir_path, ignore_errors=True)
            new_index = [e for e in index if e["preset_id"] != preset_id]
            self._save_index_locked(new_index)
            return entry

    def rebind_tag(self, preset_id: str, tag_id: int | None) -> dict[str, Any]:
        with self._lock:
            index = self._load_index_locked()
            entry = next((e for e in index if e["preset_id"] == preset_id), None)
            if entry is None:
                raise DancePresetNotFound(f"preset '{preset_id}' not found")
            self._check_tag_locked(index, tag_id, exclude_preset=preset_id)
            entry["tag_id"] = tag_id
            entry["updated_at"] = time.time()
            # Persist updated tag into the dance file too so it survives a
            # re-read that might bypass the index.
            dance_path = self.root / entry["dir"] / DANCE_FILENAME
            raw = _read_json(dance_path)
            if isinstance(raw, dict):
                raw["tag_id"] = tag_id
                raw["updated_at"] = entry["updated_at"]
                _atomic_write_json(dance_path, raw)
            self._save_index_locked(index)
            return dict(entry)

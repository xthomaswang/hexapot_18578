"""Manual Dance Encoding: state-by-state, leg-by-leg dance authoring.

The encoder reuses the calibration pipeline for live single-leg previewing:
the user picks a leg, nudges coxa/femur/tibia, and the preview is flushed
to the servos via ``send_calibration_pose`` (same 18-servo frame, clamped to
0..180). Multiple legs can be edited within the same state; playback will
move them in parallel from their ``from`` pose to their ``to`` pose.

Key rules enforced here:
- Each state's ``from`` is locked to the previous state's ``to`` (or the
  calibrated neutral for state 0). The user only writes ``to``.
- Within a state, the user edits one leg at a time. Switching legs keeps
  prior edits. Re-editing the same leg replaces its draft.
- ``save_state`` freezes the current state and advances to a new state whose
  ``from`` == the saved state's ``to``.
- ``save_dance`` validates + persists to ``DanceStorage`` and clears the
  session (caller must shut down the runner first so the encoder can own
  the serial).
"""

from __future__ import annotations

import threading
import time
from typing import Any, Callable, Mapping, Sequence


NUM_LEGS = 6
JOINTS_PER_LEG = 3
NUM_SERVOS = NUM_LEGS * JOINTS_PER_LEG

JOINT_KEYS = ("coxa_deg", "femur_deg", "tibia_deg")


class EncoderError(RuntimeError):
    """Base error for encoder operations."""


class EncoderNotActive(EncoderError):
    """Raised when a command requires an active session."""


class EncoderBusy(EncoderError):
    """Raised when a session is already active."""


def _triplet_from_mapping(payload: Mapping[str, Any]) -> dict[str, float]:
    out: dict[str, float] = {}
    for key in JOINT_KEYS:
        try:
            value = float(payload.get(key))  # type: ignore[arg-type]
        except (TypeError, ValueError) as exc:
            raise EncoderError(f"{key} must be a number") from exc
        if not 0.0 <= value <= 180.0:
            raise EncoderError(f"{key} must be within 0..180, got {value}")
        out[key] = value
    return out


def _triplet_from_frame(frame: Sequence[float], leg_index: int) -> dict[str, float]:
    start = leg_index * JOINTS_PER_LEG
    return {
        "coxa_deg": float(frame[start + 0]),
        "femur_deg": float(frame[start + 1]),
        "tibia_deg": float(frame[start + 2]),
    }


def _compose_full_frame(
    neutral_frame: Sequence[float],
    leg_pose_overrides: Mapping[int, Mapping[str, float]],
) -> list[float]:
    frame = [float(v) for v in neutral_frame]
    for leg_index, triplet in leg_pose_overrides.items():
        start = leg_index * JOINTS_PER_LEG
        for offset, key in enumerate(JOINT_KEYS):
            frame[start + offset] = float(triplet.get(key, frame[start + offset]))
    return frame


class DanceEncoder:
    """Session owner for manual dance authoring. Single-user, web-app-local."""

    def __init__(
        self,
        *,
        calibration_store: Any,
        dance_storage: Any,
        send_frame: Callable[[Sequence[float]], None] | None = None,
    ) -> None:
        self._calibration_store = calibration_store
        self._dance_storage = dance_storage
        self._send_frame = send_frame
        self._lock = threading.RLock()
        self._reset_session_locked()

    # ---- session lifecycle ----------------------------------------------

    def _reset_session_locked(self) -> None:
        self._active = False
        self._name: str = ""
        self._neutral_frame: list[float] | None = None
        self._default_segment_ms: int = 440
        self._saved_states: list[dict[str, Any]] = []
        # Current state being edited:
        self._current_state_from: list[dict[str, float]] | None = None  # 6 triplets
        self._current_state_to: list[dict[str, float]] | None = None
        self._current_state_edited: list[bool] = [False] * NUM_LEGS
        self._current_state_duration_ms: int = 440
        self._current_leg_index: int | None = None
        self._last_sent_frame: list[float] | None = None
        self._started_at: float | None = None

    def _require_active_locked(self) -> None:
        if not self._active:
            raise EncoderNotActive("no dance encoding session; call start first")

    # ---- public API ------------------------------------------------------

    def start(self, *, name: str, default_segment_ms: int = 440) -> dict[str, Any]:
        name = str(name or "").strip()
        if not name:
            raise EncoderError("name is required")
        snapshot = self._calibration_store.snapshot()
        if not snapshot.get("complete"):
            raise EncoderError(
                f"calibration incomplete: {snapshot.get('saved_count', 0)}/6 legs saved"
            )
        legs = snapshot.get("legs") or []
        from process_control import _profile_full_frame

        neutral_frame = _profile_full_frame(legs)
        if len(neutral_frame) != NUM_SERVOS:
            raise EncoderError("calibration snapshot did not produce 18 servos")
        with self._lock:
            if self._active:
                raise EncoderBusy("dance encoder already active; save or abort first")
            self._reset_session_locked()
            self._active = True
            self._name = name
            self._neutral_frame = [float(v) for v in neutral_frame]
            self._default_segment_ms = int(default_segment_ms)
            self._current_state_duration_ms = int(default_segment_ms)
            # state 0: from = neutral, to starts = neutral, nothing edited
            self._current_state_from = [
                _triplet_from_frame(self._neutral_frame, leg) for leg in range(NUM_LEGS)
            ]
            self._current_state_to = [
                _triplet_from_frame(self._neutral_frame, leg) for leg in range(NUM_LEGS)
            ]
            self._current_state_edited = [False] * NUM_LEGS
            self._started_at = time.time()
        return self.snapshot()

    def abort(self) -> dict[str, Any]:
        with self._lock:
            self._reset_session_locked()
        return self.snapshot()

    def select_leg(self, leg_index: int) -> dict[str, Any]:
        with self._lock:
            self._require_active_locked()
            if not 0 <= int(leg_index) < NUM_LEGS:
                raise EncoderError(f"leg_index must be in 0..{NUM_LEGS - 1}")
            self._current_leg_index = int(leg_index)
        return self.snapshot()

    def set_leg_duration_ms(self, duration_ms: int) -> dict[str, Any]:
        from dance_storage import MAX_SEGMENT_MS, MIN_SEGMENT_MS

        with self._lock:
            self._require_active_locked()
            dm = int(duration_ms)
            if not MIN_SEGMENT_MS <= dm <= MAX_SEGMENT_MS:
                raise EncoderError(
                    f"duration_ms must be within {MIN_SEGMENT_MS}..{MAX_SEGMENT_MS}"
                )
            self._current_state_duration_ms = dm
        return self.snapshot()

    def draft_leg(self, leg_index: int, angles: Mapping[str, Any]) -> dict[str, Any]:
        """Update the ``to`` pose for one leg in the current state."""
        with self._lock:
            self._require_active_locked()
            if not 0 <= int(leg_index) < NUM_LEGS:
                raise EncoderError(f"leg_index must be in 0..{NUM_LEGS - 1}")
            triplet = _triplet_from_mapping(angles)
            self._current_state_to[int(leg_index)] = triplet
            self._current_state_edited[int(leg_index)] = True
            self._current_leg_index = int(leg_index)
        return self.snapshot()

    def preview_frame(self) -> list[float]:
        """Build the 18-servo frame to send for live preview.

        The preview shows every previously-edited leg AT ITS NEW ``to`` pose,
        so the user can see cumulative edits in the current state; non-edited
        legs stay at their ``from`` (which equals the previous state's ``to``).
        """
        with self._lock:
            self._require_active_locked()
            overrides: dict[int, Mapping[str, float]] = {}
            for leg_index in range(NUM_LEGS):
                if self._current_state_edited[leg_index]:
                    overrides[leg_index] = self._current_state_to[leg_index]
                else:
                    overrides[leg_index] = self._current_state_from[leg_index]
            base = self._neutral_frame or [0.0] * NUM_SERVOS
            frame = _compose_full_frame(base, overrides)
            self._last_sent_frame = list(frame)
        return frame

    def send_preview(self) -> list[float]:
        """Build preview frame and push to the serial bridge (if wired)."""
        frame = self.preview_frame()
        if self._send_frame is not None:
            self._send_frame(frame)
        return frame

    def save_state(self) -> dict[str, Any]:
        """Freeze the current state, advance to a new empty state."""
        with self._lock:
            self._require_active_locked()
            if not any(self._current_state_edited):
                raise EncoderError(
                    "current state has no edits; edit at least one leg before save_state"
                )
            legs_payload: list[dict[str, Any]] = []
            for leg_index in range(NUM_LEGS):
                legs_payload.append(
                    {
                        "leg_index": leg_index,
                        "edited": bool(self._current_state_edited[leg_index]),
                        "from": dict(self._current_state_from[leg_index]),
                        "to": dict(self._current_state_to[leg_index]),
                    }
                )
            state = {
                "index": len(self._saved_states),
                "duration_ms": int(self._current_state_duration_ms),
                "legs": legs_payload,
            }
            self._saved_states.append(state)
            # Advance: next state's from = this state's to
            next_from = [dict(leg["to"]) for leg in legs_payload]
            self._current_state_from = next_from
            self._current_state_to = [dict(t) for t in next_from]
            self._current_state_edited = [False] * NUM_LEGS
            self._current_state_duration_ms = self._default_segment_ms
            self._current_leg_index = None
        return self.snapshot()

    def save_dance(self, *, tag_id: int | None = None) -> dict[str, Any]:
        """Persist the full dance (needs at least one saved state)."""
        with self._lock:
            self._require_active_locked()
            if not self._saved_states:
                raise EncoderError(
                    "dance has no saved states; call save_state at least once before save_dance"
                )
            payload = {
                "name": self._name,
                "preset_id": None,  # storage will derive from name
                "tag_id": tag_id,
                "default_segment_ms": int(self._default_segment_ms),
                "calibration_snapshot": {
                    "angles": [float(v) for v in (self._neutral_frame or [])],
                },
                "states": [dict(s) for s in self._saved_states],
            }
        entry = self._dance_storage.save_dance(payload)
        with self._lock:
            self._reset_session_locked()
        return entry

    # ---- snapshot --------------------------------------------------------

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "active": self._active,
                "name": self._name,
                "default_segment_ms": self._default_segment_ms,
                "current_state_index": len(self._saved_states) if self._active else None,
                "current_state_duration_ms": self._current_state_duration_ms if self._active else None,
                "current_state_edited": list(self._current_state_edited) if self._active else [],
                "current_state_from": [dict(t) for t in (self._current_state_from or [])] if self._active else [],
                "current_state_to": [dict(t) for t in (self._current_state_to or [])] if self._active else [],
                "current_leg_index": self._current_leg_index,
                "saved_state_count": len(self._saved_states),
                "saved_states": [dict(s) for s in self._saved_states] if self._active else [],
                "neutral_frame": list(self._neutral_frame) if (self._active and self._neutral_frame) else [],
                "started_at": self._started_at,
            }

"""Automated-mode worker: plays user-authored dance presets in a loop.

Runs inside the web process and owns the serial link while active. Must be
stopped (``shutdown``) before any other caller (runtime subprocess,
calibration send, dance encoder) can use the serial port.

A "dance" is a validated dict from ``dance_storage`` (list of states; each
state has 6 legs with ``from`` / ``to`` angle triplets and a ``duration_ms``).
The player interpolates every leg in parallel from its ``from`` to its ``to``
over the state's duration, then chains to the next state. When the last state
finishes, we interpolate from its ``to`` back to state 0's ``from`` so the
loop has no hard jump.
"""

from __future__ import annotations

import sys
import threading
import time
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence


class CalibrationIncomplete(RuntimeError):
    """Raised when fewer than 6 legs are saved in the calibration profile."""


class SerialUnavailable(RuntimeError):
    """Raised when the serial bridge cannot be opened or used."""


class PresetNotLoaded(RuntimeError):
    """Raised when start() is called without a loaded dance."""


# Tag id -> logical command. Tag 1 is the hardwired stop action (not
# overridable by user dances). Other tag ids are bound dynamically by dances
# via the storage layer.
TAG_STOP = 1

DEBOUNCE_FRAMES = 3

TICK_SECONDS = 0.1  # 10 Hz camera polling when idle
INTERP_STEP_SECONDS = 0.02  # one servo write every 20 ms during a segment

# Used for the neutral-in / neutral-out transitions on start/stop, and as the
# fallback duration when a dance state lacks a duration_ms.
DEFAULT_NEUTRAL_TRANSITION_MS = 440

NUM_LEGS = 6
JOINTS_PER_LEG = 3
NUM_SERVOS = NUM_LEGS * JOINTS_PER_LEG

# If the camera /detections endpoint keeps failing while a preset is active,
# we've lost the only path that can receive a stop tag. Auto-stop after this
# many consecutive failed polls so the robot returns to neutral instead of
# dancing blind. At TICK_SECONDS = 0.1 s, 30 ~= 3 s grace.
DETECTIONS_FAILURE_STOP_THRESHOLD = 30


ROBOT_ROOT = Path(__file__).resolve().parents[1]
MOTION_ENGINE_ROOT = ROBOT_ROOT / "motion_engine"


def _motion_engine_import_ready() -> None:
    motion_engine_root = str(MOTION_ENGINE_ROOT)
    if motion_engine_root not in sys.path:
        sys.path.insert(0, motion_engine_root)


def _default_format_frame(frame: Sequence[float]) -> str:
    _motion_engine_import_ready()
    from runtime.protocol import format_full_dir_command  # type: ignore

    return format_full_dir_command("N", list(frame))


def _default_serial_factory(port: str, baudrate: int) -> Any:
    _motion_engine_import_ready()
    from runtime.runtime import SerialBridge  # type: ignore

    return SerialBridge(port=port, baudrate=baudrate)


def state_frame(state: Mapping[str, Any], *, which: str) -> list[float]:
    """Return the 18-servo frame for a state's ``from`` or ``to`` column."""
    if which not in ("from", "to"):
        raise ValueError("which must be 'from' or 'to'")
    from process_control import _angles_triplet

    legs = state.get("legs") or []
    frame: list[float] = []
    for leg_index in range(NUM_LEGS):
        leg = legs[leg_index] if leg_index < len(legs) else {}
        frame.extend(_angles_triplet(leg.get(which) or {}))
    return frame


def _profile_neutral_frame(legs: Sequence[Mapping[str, Any]]) -> list[float]:
    from process_control import _profile_full_frame

    frame = _profile_full_frame(legs)
    if len(frame) != NUM_SERVOS:
        raise RuntimeError(
            f"expected {NUM_SERVOS} angles in calibration profile, got {len(frame)}"
        )
    return [float(v) for v in frame]


def _frames_close(
    left: Sequence[float],
    right: Sequence[float],
    *,
    tolerance: float = 1e-6,
) -> bool:
    if len(left) != len(right):
        return False
    return all(abs(float(a) - float(b)) <= tolerance for a, b in zip(left, right))


def dominant_tag_id(
    detections: Sequence[Mapping[str, Any]],
    *,
    allowed: set[int] | None = None,
) -> int | None:
    """Pick the lowest tag id the runner knows about (lowest == highest priority)."""
    ids: set[int] = set()
    for item in detections or []:
        if not isinstance(item, Mapping):
            continue
        raw = item.get("tag_id")
        if raw is None:
            continue
        try:
            tag_id = int(raw)
        except (TypeError, ValueError):
            continue
        if allowed is None or tag_id in allowed:
            ids.add(tag_id)
    if not ids:
        return None
    return min(ids)


class _DebounceState:
    def __init__(self, threshold: int = DEBOUNCE_FRAMES) -> None:
        self.threshold = max(1, int(threshold))
        self.confirmed: int | None = None
        self._pending: int | None = None
        self._count = 0

    def observe(self, tag_id: int | None) -> int | None:
        if tag_id is None:
            self._pending = None
            self._count = 0
            return None
        if tag_id == self.confirmed:
            self._pending = None
            self._count = 0
            return None
        if tag_id == self._pending:
            self._count += 1
        else:
            self._pending = tag_id
            self._count = 1
        if self._count >= self.threshold:
            self.confirmed = tag_id
            self._pending = None
            self._count = 0
            return tag_id
        return None

    def reset(self) -> None:
        self.confirmed = None
        self._pending = None
        self._count = 0


class AutomatedRunner:
    """Serial-owning worker that plays dance presets on AprilTag or button commands.

    The runner has no hardcoded choreography. The caller loads a dance dict
    (via ``load_dance``) from ``DanceStorage`` and then calls ``start`` — the
    player iterates through states forever until ``stop`` or a stop-tag edge.
    """

    def __init__(
        self,
        calibration_store: Any,
        *,
        dance_storage: Any | None = None,
        serial_factory: Callable[[str, int], Any] | None = None,
        detections_fetcher: Callable[[], tuple[Mapping[str, Any], int]] | None = None,
        frame_serializer: Callable[[Sequence[float]], str] | None = None,
        serial_port: str = "/dev/serial0",
        serial_baudrate: int = 115200,
        tick_seconds: float = TICK_SECONDS,
        interp_step_seconds: float = INTERP_STEP_SECONDS,
        detections_failure_stop_threshold: int = DETECTIONS_FAILURE_STOP_THRESHOLD,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._calibration_store = calibration_store
        self._dance_storage = dance_storage
        self._serial_factory = serial_factory or _default_serial_factory
        self._detections_fetcher = detections_fetcher
        self._frame_serializer = frame_serializer or _default_format_frame
        self._serial_port = serial_port
        self._serial_baudrate = serial_baudrate
        self._tick_seconds = float(tick_seconds)
        self._interp_step_seconds = max(0.001, float(interp_step_seconds))
        self._detections_failure_stop_threshold = max(1, int(detections_failure_stop_threshold))
        self._clock = clock

        self._state_lock = threading.Lock()
        self._serial_lock = threading.RLock()
        self._stop_event = threading.Event()

        self._worker: threading.Thread | None = None
        self._bridge: Any | None = None

        # State exposed via snapshot()
        self._active = False
        self._preset_id: str | None = None
        self._preset_name: str | None = None
        self._source: str | None = None
        self._last_request: dict[str, Any] | None = None
        self._last_stop_reason: str | None = None
        self._last_tag_id: int | None = None
        self._recognized_tag_ids: list[int] = []
        self._error: str | None = None
        self._ready = False
        self._detections_failure_streak = 0

        # Playback state
        self._loaded_dance: dict[str, Any] | None = None
        self._segments: list[dict[str, Any]] = []
        self._segment_index = 0
        self._current_frame: list[float] | None = None
        self._current_segment: dict[str, Any] | None = None

        # Tag routing
        self._debounce = _DebounceState()
        self._tag_to_preset: dict[int, str] = {}

    # ---- serial lifecycle ------------------------------------------------

    def prepare(self) -> dict[str, Any]:
        with self._state_lock:
            already_ready = self._ready
        if already_ready:
            return self.snapshot()
        self._open_bridge()
        self._start_worker()
        with self._state_lock:
            self._ready = True
        self._refresh_tag_map()
        return self.snapshot()

    def _open_bridge(self) -> None:
        if self._bridge is not None:
            return
        try:
            bridge = self._serial_factory(self._serial_port, self._serial_baudrate)
        except Exception as exc:  # pragma: no cover - depends on target env
            with self._state_lock:
                self._error = f"serial open failed: {exc}"
            raise SerialUnavailable(str(exc)) from exc
        self._bridge = bridge
        with self._state_lock:
            self._error = None

    def _close_bridge(self) -> None:
        bridge = self._bridge
        self._bridge = None
        if bridge is None:
            return
        try:
            bridge.close()
        except Exception:
            pass

    def shutdown(self) -> dict[str, Any]:
        self._stop_event.set()
        worker = self._worker
        if worker is not None:
            worker.join(timeout=3.0)
        self._worker = None
        with self._serial_lock:
            self._close_bridge()
        with self._state_lock:
            self._active = False
            self._preset_id = None
            self._preset_name = None
            self._source = None
            self._loaded_dance = None
            self._segments = []
            self._segment_index = 0
            self._current_segment = None
            self._current_frame = None
            self._ready = False
            self._debounce.reset()
            self._recognized_tag_ids = []
            self._detections_failure_streak = 0
            self._tag_to_preset = {}
        return self.snapshot()

    # ---- tag routing helpers --------------------------------------------

    def refresh_tag_map(self) -> dict[int, str]:
        """Rebuild the tag → preset_id map from storage. Call after preset CRUD."""
        return self._refresh_tag_map()

    def _refresh_tag_map(self) -> dict[int, str]:
        if self._dance_storage is None:
            with self._state_lock:
                self._tag_to_preset = {}
            return {}
        mapping: dict[int, str] = {}
        try:
            for entry in self._dance_storage.list_presets():
                tag_id = entry.get("tag_id")
                if isinstance(tag_id, int) and tag_id != TAG_STOP:
                    mapping[tag_id] = entry["preset_id"]
        except Exception as exc:
            with self._state_lock:
                self._error = f"preset index read failed: {exc}"
            return {}
        with self._state_lock:
            self._tag_to_preset = mapping
        return mapping

    # ---- playback schedule ----------------------------------------------

    def _build_segments(self, dance: Mapping[str, Any]) -> list[dict[str, Any]]:
        """Expand a validated dance into a list of playback segments.

        Each segment is ``{from_frame, to_frame, duration_ms, label}``. The
        final segment closes the loop by interpolating from the last state's
        ``to`` back to state 0's ``from`` (default duration if not specified).
        """
        states = dance.get("states") or []
        if not states:
            return []
        default_ms = int(dance.get("default_segment_ms") or DEFAULT_NEUTRAL_TRANSITION_MS)
        segments: list[dict[str, Any]] = []
        for i, state in enumerate(states):
            segments.append(
                {
                    "from_frame": state_frame(state, which="from"),
                    "to_frame": state_frame(state, which="to"),
                    "duration_ms": int(state.get("duration_ms") or default_ms),
                    "label": f"state-{i}",
                }
            )
        first_from = segments[0]["from_frame"]
        last_to = segments[-1]["to_frame"]
        if not _frames_close(last_to, first_from):
            segments.append(
                {
                    "from_frame": last_to,
                    "to_frame": first_from,
                    "duration_ms": default_ms,
                    "label": "loop-back",
                }
            )
        return segments

    # ---- public commands -------------------------------------------------

    def load_dance(self, dance: Mapping[str, Any]) -> None:
        """Stash a validated dance dict. Does not start playback."""
        segments = self._build_segments(dance)
        with self._state_lock:
            self._loaded_dance = dict(dance)
            self._segments = segments

    def start(
        self,
        *,
        source: str,
        tag_id: int | None = None,
    ) -> dict[str, Any]:
        with self._state_lock:
            dance = self._loaded_dance
            segments = list(self._segments) if self._segments else []
        if dance is None or not segments:
            raise PresetNotLoaded("no dance loaded; call load_dance first")

        neutral = self._load_calibrated_neutral()
        self._ensure_prepared()

        preset_id = str(dance.get("preset_id") or "")
        preset_name = str(dance.get("name") or preset_id)

        with self._serial_lock:
            # Enter neutral before starting so switching presets is clean.
            if self._current_frame is not None:
                self._execute_segment(
                    self._current_frame, neutral, duration_ms=DEFAULT_NEUTRAL_TRANSITION_MS,
                )
            else:
                self._write_frame(neutral)
                self._pause(0.2)
            first_from = list(segments[0]["from_frame"])
            if not _frames_close(neutral, first_from):
                self._execute_segment(
                    neutral, first_from, duration_ms=DEFAULT_NEUTRAL_TRANSITION_MS,
                )
            with self._state_lock:
                self._active = True
                self._preset_id = preset_id
                self._preset_name = preset_name
                self._source = source
                self._last_request = {
                    "preset": preset_id,
                    "source": source,
                    "tag_id": tag_id,
                    "requested_at": time.time(),
                }
                if tag_id is not None:
                    self._last_tag_id = int(tag_id)
                self._last_stop_reason = None
                self._detections_failure_streak = 0
                self._error = None
                self._segments = segments
                self._segment_index = 0
                self._current_frame = first_from
                self._current_segment = None
        return self.snapshot()

    def stop(self, reason: str) -> dict[str, Any]:
        with self._state_lock:
            was_active = self._active
        try:
            neutral = self._load_calibrated_neutral()
        except CalibrationIncomplete:
            neutral = None

        with self._serial_lock:
            if neutral is None:
                with self._state_lock:
                    neutral = list(self._current_frame or [])
            if neutral and (was_active or self._current_frame is not None):
                from_frame = self._current_frame or neutral
                self._execute_segment(
                    from_frame, neutral, duration_ms=DEFAULT_NEUTRAL_TRANSITION_MS,
                )
            with self._state_lock:
                self._active = False
                self._preset_id = None
                self._preset_name = None
                self._source = None
                self._segments = []
                self._segment_index = 0
                self._current_segment = None
                if neutral:
                    self._current_frame = list(neutral)
                self._last_stop_reason = reason
        return self.snapshot()

    # ---- tag ingestion ---------------------------------------------------

    def process_detections(self, payload: Mapping[str, Any] | None) -> dict[str, Any]:
        detections = []
        if isinstance(payload, Mapping):
            raw = payload.get("detections")
            if isinstance(raw, list):
                detections = raw
        with self._state_lock:
            tag_map = dict(self._tag_to_preset)
        allowed = {TAG_STOP} | set(tag_map.keys())
        tag_id = dominant_tag_id(detections, allowed=allowed)
        recognised = sorted({
            int(item["tag_id"])
            for item in detections
            if isinstance(item, Mapping)
            and isinstance(item.get("tag_id"), (int, float))
            and int(item["tag_id"]) in allowed
        })
        with self._state_lock:
            self._recognized_tag_ids = recognised
        confirmed = self._debounce.observe(tag_id)
        if confirmed is None:
            return self.snapshot()
        return self._dispatch_tag(confirmed, tag_map)

    def _dispatch_tag(self, tag_id: int, tag_map: dict[int, str]) -> dict[str, Any]:
        with self._state_lock:
            self._last_tag_id = int(tag_id)
        try:
            if tag_id == TAG_STOP:
                return self.stop(reason=f"tag{tag_id}")
            preset_id = tag_map.get(tag_id)
            if preset_id is None or self._dance_storage is None:
                return self.snapshot()
            dance = self._dance_storage.get_dance(preset_id)
            self.load_dance(dance)
            return self.start(source="apriltag", tag_id=tag_id)
        except Exception as exc:
            with self._state_lock:
                self._error = f"tag {tag_id}: {exc}"
        return self.snapshot()

    # ---- worker loop -----------------------------------------------------

    def _start_worker(self) -> None:
        if self._worker is not None and self._worker.is_alive():
            return
        self._stop_event.clear()
        self._worker = threading.Thread(
            target=self._run,
            name="automated-runner",
            daemon=True,
        )
        self._worker.start()

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                self._poll_detections_once()
            except Exception as exc:
                with self._state_lock:
                    self._error = f"detections: {exc}"
            with self._state_lock:
                active = self._active
            if active:
                try:
                    self._advance_segment_once()
                except Exception as exc:
                    with self._state_lock:
                        self._error = f"segment step: {exc}"
                    self._stop_event.wait(self._tick_seconds)
            else:
                self._stop_event.wait(self._tick_seconds)

    def _advance_segment_once(self) -> None:
        with self._state_lock:
            if not self._segments:
                self._active = False
                return
            segment = self._segments[self._segment_index % len(self._segments)]
            self._segment_index = (self._segment_index + 1) % len(self._segments)
            # _current_frame is set to neutral in start() and to to_frame after
            # each segment, so it's never None on this path.
            from_frame = list(self._current_frame or segment["from_frame"])
            to_frame = list(segment["to_frame"])
            duration_ms = int(segment.get("duration_ms") or DEFAULT_NEUTRAL_TRANSITION_MS)
            self._current_segment = {
                "label": segment.get("label"),
                "duration_ms": duration_ms,
            }
        with self._serial_lock:
            if self._stop_event.is_set():
                return
            self._execute_segment(from_frame, to_frame, duration_ms=duration_ms)
            with self._state_lock:
                self._current_frame = list(to_frame)

    def _poll_detections_once(self) -> None:
        fetcher = self._detections_fetcher
        if fetcher is None:
            return
        failure_reason: str | None = None
        payload: Mapping[str, Any] | None = None
        try:
            payload, status_code = fetcher()
        except Exception as exc:
            failure_reason = f"detections fetch: {exc}"
        else:
            if status_code >= 400:
                failure_reason = f"detections http {status_code}"
            elif not isinstance(payload, Mapping):
                failure_reason = "detections payload invalid"
        if failure_reason is not None:
            self._note_detections_failure(failure_reason)
            return
        self._clear_detections_failure()
        assert isinstance(payload, Mapping)
        self.process_detections(payload)

    def _note_detections_failure(self, reason: str) -> None:
        with self._state_lock:
            self._detections_failure_streak += 1
            streak = self._detections_failure_streak
            active = self._active
            self._error = reason
        if active and streak >= self._detections_failure_stop_threshold:
            self.stop(reason="camera_lost")

    def _clear_detections_failure(self) -> None:
        with self._state_lock:
            if self._detections_failure_streak:
                self._detections_failure_streak = 0
                if self._error and self._error.startswith(("detections ", "detections:")):
                    self._error = None

    # ---- serial helpers --------------------------------------------------

    def _execute_segment(
        self,
        from_frame: Sequence[float],
        to_frame: Sequence[float],
        *,
        duration_ms: int,
    ) -> None:
        if len(from_frame) != len(to_frame):
            raise ValueError("frame length mismatch")
        duration_s = max(0.0, float(duration_ms) / 1000.0)
        steps = max(1, int(round(duration_s / self._interp_step_seconds)))
        for step in range(1, steps + 1):
            if self._stop_event.is_set():
                return
            t = step / steps
            interp = [float(a) + t * (float(b) - float(a)) for a, b in zip(from_frame, to_frame)]
            self._write_frame(interp)
            if step < steps:
                self._pause(self._interp_step_seconds)

    def _write_frame(self, frame: Sequence[float]) -> None:
        if len(frame) != NUM_SERVOS:
            raise ValueError(f"expected {NUM_SERVOS} servos, got {len(frame)}")
        line = self._frame_serializer(frame)
        bridge = self._bridge
        if bridge is None:
            return
        bridge.write_line(line)

    def _pause(self, seconds: float) -> None:
        if seconds <= 0:
            return
        self._stop_event.wait(seconds)

    # ---- state helpers ---------------------------------------------------

    def _ensure_prepared(self) -> None:
        with self._state_lock:
            already = self._ready
        if already:
            return
        self._open_bridge()
        self._start_worker()
        with self._state_lock:
            self._ready = True
        self._refresh_tag_map()

    def _load_calibrated_neutral(self) -> list[float]:
        snapshot = self._calibration_store.snapshot()
        if not snapshot.get("complete"):
            raise CalibrationIncomplete(
                f"only {snapshot.get('saved_count', 0)}/6 legs saved; save all legs first"
            )
        legs = snapshot.get("legs") or []
        return _profile_neutral_frame(legs)

    def snapshot(self) -> dict[str, Any]:
        with self._state_lock:
            tag_commands = {str(TAG_STOP): "stop"}
            for tag_id, preset_id in self._tag_to_preset.items():
                tag_commands[str(tag_id)] = preset_id
            return {
                "implemented": True,
                "active": bool(self._active),
                "preset_id": self._preset_id,
                "preset_name": self._preset_name,
                "source": self._source,
                "last_request": dict(self._last_request) if self._last_request else None,
                "last_stop_reason": self._last_stop_reason,
                "last_tag_id": self._last_tag_id,
                "recognized_tag_ids": list(self._recognized_tag_ids),
                "tag_commands": tag_commands,
                "error": self._error,
                "ready": self._ready,
                "current_segment": dict(self._current_segment) if self._current_segment else None,
            }

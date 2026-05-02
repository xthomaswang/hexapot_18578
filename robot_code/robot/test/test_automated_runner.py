"""Unit tests for the dance-preset player (no serial hardware required)."""

from __future__ import annotations

import sys
import time
import unittest
from pathlib import Path


WEB_ROOT = Path(__file__).resolve().parents[1] / "web"
if str(WEB_ROOT) not in sys.path:
    sys.path.insert(0, str(WEB_ROOT))


from automated_runner import (  # noqa: E402
    AutomatedRunner,
    CalibrationIncomplete,
    DEBOUNCE_FRAMES,
    PresetNotLoaded,
    SerialUnavailable,
    TAG_STOP,
    _DebounceState,
    dominant_tag_id,
    state_frame,
)


NUM_LEGS = 6
JOINTS_PER_LEG = 3
NUM_SERVOS = NUM_LEGS * JOINTS_PER_LEG

NEUTRAL = [90.0 + i * 0.5 for i in range(NUM_SERVOS)]


class FakeSerial:
    def __init__(self) -> None:
        self.lines: list[str] = []
        self.closed = False

    def write_line(self, line: str) -> None:
        self.lines.append(line)

    def close(self) -> None:
        self.closed = True


def _build_neutral_legs():
    legs = []
    for i in range(NUM_LEGS):
        legs.append({
            "leg_index": i,
            "angles_deg": {
                "coxa_deg": NEUTRAL[i * 3 + 0],
                "femur_deg": NEUTRAL[i * 3 + 1],
                "tibia_deg": NEUTRAL[i * 3 + 2],
            },
            "saved": True,
        })
    return legs


class FakeCalibrationStore:
    def __init__(self, complete: bool = True) -> None:
        self._complete = complete
        self._legs = _build_neutral_legs()

    def snapshot(self):
        return {
            "legs": [dict(l, angles_deg=dict(l["angles_deg"])) for l in self._legs],
            "saved_count": NUM_LEGS if self._complete else 0,
            "complete": self._complete,
        }


class FakeDanceStorage:
    def __init__(self, dances: list[dict] | None = None) -> None:
        self.dances = dances or []

    def list_presets(self) -> list[dict]:
        return [
            {
                "preset_id": d["preset_id"],
                "name": d["name"],
                "tag_id": d.get("tag_id"),
                "dir": "",
                "created_at": 0.0,
                "updated_at": 0.0,
            }
            for d in self.dances
        ]

    def get_dance(self, preset_id: str) -> dict:
        for d in self.dances:
            if d["preset_id"] == preset_id:
                return d
        raise KeyError(preset_id)


def _serializer(frame):
    return "N:" + ",".join(f"{v:.3f}" for v in frame) + "\n"


def _parse(line: str) -> list[float]:
    assert line.startswith("N:")
    return [float(v) for v in line[2:].strip().split(",")]


def _build_dance(preset_id: str, *, tag_id: int | None = None) -> dict:
    """Build a minimal 1-state dance that lifts leg 0 (femur -8, tibia +6)."""
    lifted = list(NEUTRAL)
    lifted[1] -= 8.0
    lifted[2] += 6.0

    def triplet_from(frame, leg):
        s = leg * JOINTS_PER_LEG
        return {"coxa_deg": frame[s], "femur_deg": frame[s + 1], "tibia_deg": frame[s + 2]}

    legs_payload = []
    for i in range(NUM_LEGS):
        legs_payload.append({
            "leg_index": i,
            "edited": i == 0,
            "from": triplet_from(NEUTRAL, i),
            "to": triplet_from(lifted if i == 0 else NEUTRAL, i),
        })
    return {
        "preset_id": preset_id,
        "name": preset_id,
        "tag_id": tag_id,
        "default_segment_ms": 440,
        "states": [
            {"index": 0, "duration_ms": 100, "legs": legs_payload},
        ],
    }


def _make_runner(*, dances=None, fetcher=None, threshold=30):
    bridge = FakeSerial()
    runner = AutomatedRunner(
        calibration_store=FakeCalibrationStore(),
        dance_storage=FakeDanceStorage(dances or []),
        serial_factory=lambda port, baud: bridge,
        detections_fetcher=fetcher,
        frame_serializer=_serializer,
        interp_step_seconds=0.01,
        tick_seconds=60.0,
        detections_failure_stop_threshold=threshold,
    )
    return runner, bridge


class FrameMathTests(unittest.TestCase):
    def test_state_frame_from_and_to(self) -> None:
        dance = _build_dance("x")
        state = dance["states"][0]
        self.assertEqual(state_frame(state, which="from"), NEUTRAL)
        to_frame = state_frame(state, which="to")
        self.assertEqual(to_frame[0], NEUTRAL[0])  # coxa unchanged
        self.assertAlmostEqual(to_frame[1] - NEUTRAL[1], -8.0)
        self.assertAlmostEqual(to_frame[2] - NEUTRAL[2], 6.0)
        for i in range(JOINTS_PER_LEG, NUM_SERVOS):
            self.assertEqual(to_frame[i], NEUTRAL[i])


class DebounceTests(unittest.TestCase):
    def test_confirms_after_n_consecutive(self) -> None:
        d = _DebounceState(threshold=DEBOUNCE_FRAMES)
        self.assertIsNone(d.observe(2))
        self.assertIsNone(d.observe(2))
        self.assertEqual(d.observe(2), 2)
        self.assertIsNone(d.observe(2))  # already confirmed

    def test_switch_resets(self) -> None:
        d = _DebounceState(threshold=3)
        d.observe(5)
        d.observe(5)
        self.assertIsNone(d.observe(3))
        self.assertIsNone(d.observe(3))
        self.assertEqual(d.observe(3), 3)


class DominantTagTests(unittest.TestCase):
    def test_minimum_tag_wins(self) -> None:
        self.assertEqual(dominant_tag_id([{"tag_id": 3}, {"tag_id": 1}]), 1)
        self.assertEqual(dominant_tag_id([{"tag_id": 5}, {"tag_id": 2}]), 2)

    def test_filter_by_allowed(self) -> None:
        self.assertEqual(dominant_tag_id([{"tag_id": 9}, {"tag_id": 5}], allowed={5}), 5)
        self.assertIsNone(dominant_tag_id([{"tag_id": 9}], allowed={5}))


class LoadAndPlayTests(unittest.TestCase):
    def test_load_dance_builds_segments_with_loopback(self) -> None:
        dance = _build_dance("x")
        runner, _bridge = _make_runner(dances=[dance])
        runner.prepare()
        try:
            runner.load_dance(dance)
            self.assertEqual(len(runner._segments), 2)  # 1 state + 1 loop-back
            self.assertEqual(runner._segments[0]["label"], "state-0")
            self.assertEqual(runner._segments[1]["label"], "loop-back")
        finally:
            runner.shutdown()

    def test_start_requires_load(self) -> None:
        runner, _bridge = _make_runner()
        runner.prepare()
        try:
            with self.assertRaises(PresetNotLoaded):
                runner.start(source="manual_button")
        finally:
            runner.shutdown()

    def test_start_then_stop_sets_state(self) -> None:
        dance = _build_dance("x")
        runner, bridge = _make_runner(dances=[dance])
        runner.prepare()
        try:
            runner.load_dance(dance)
            runner.start(source="manual_button")
            snap = runner.snapshot()
            self.assertTrue(snap["active"])
            self.assertEqual(snap["preset_id"], "x")
            runner.stop(reason="manual")
            snap = runner.snapshot()
            self.assertFalse(snap["active"])
            self.assertEqual(snap["last_stop_reason"], "manual")
            self.assertTrue(any(l.startswith("N:") for l in bridge.lines))
        finally:
            runner.shutdown()

    def test_start_aligns_to_recorded_first_from_frame(self) -> None:
        dance = _build_dance("x")
        first_from = list(NEUTRAL)
        first_from[1] += 12.0
        first_from[2] -= 9.0
        dance["states"][0]["legs"][0]["from"] = {
            "coxa_deg": first_from[0],
            "femur_deg": first_from[1],
            "tibia_deg": first_from[2],
        }
        runner, bridge = _make_runner(dances=[dance])
        runner.prepare()
        try:
            runner.load_dance(dance)
            runner.start(source="manual_button")
            self.assertEqual(_parse(bridge.lines[-1]), first_from)
            self.assertEqual(runner._current_frame, first_from)
        finally:
            runner.shutdown()


class TagRoutingTests(unittest.TestCase):
    def test_refresh_tag_map_excludes_stop(self) -> None:
        runner, _bridge = _make_runner(dances=[
            _build_dance("wave", tag_id=5),
            _build_dance("wiggle", tag_id=7),
        ])
        runner.prepare()
        try:
            mapping = runner.refresh_tag_map()
            self.assertEqual(mapping, {5: "wave", 7: "wiggle"})
            # Even if a dance somehow claimed tag 1, the runner must ignore it.
        finally:
            runner.shutdown()

    def test_stop_tag_fires_stop(self) -> None:
        dance = _build_dance("wave", tag_id=5)
        runner, _bridge = _make_runner(dances=[dance])
        runner.prepare()
        try:
            runner.load_dance(dance)
            runner.start(source="manual_button")
            for _ in range(DEBOUNCE_FRAMES):
                runner.process_detections({"detections": [{"tag_id": TAG_STOP}]})
            snap = runner.snapshot()
            self.assertFalse(snap["active"])
            self.assertEqual(snap["last_stop_reason"], f"tag{TAG_STOP}")
        finally:
            runner.shutdown()

    def test_tag_triggers_preset_start(self) -> None:
        dance = _build_dance("wave", tag_id=5)
        runner, _bridge = _make_runner(dances=[dance])
        runner.prepare()
        try:
            for _ in range(DEBOUNCE_FRAMES):
                runner.process_detections({"detections": [{"tag_id": 5}]})
            snap = runner.snapshot()
            self.assertTrue(snap["active"])
            self.assertEqual(snap["preset_id"], "wave")
            self.assertEqual(snap["source"], "apriltag")
        finally:
            runner.shutdown()


class FailureStreakTests(unittest.TestCase):
    def test_shutdown_clears_streak(self) -> None:
        runner, _bridge = _make_runner()
        runner._detections_failure_streak = 2
        runner.shutdown()
        self.assertEqual(runner._detections_failure_streak, 0)

    def test_start_clears_streak(self) -> None:
        dance = _build_dance("x")
        runner, _bridge = _make_runner(dances=[dance])
        runner._detections_failure_streak = 2
        runner.prepare()
        try:
            runner.load_dance(dance)
            runner.start(source="manual_button")
            self.assertEqual(runner._detections_failure_streak, 0)
        finally:
            runner.shutdown()


if __name__ == "__main__":
    unittest.main()

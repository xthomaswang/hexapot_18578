"""Tests for DanceStorage file persistence and index invariants."""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path


WEB_ROOT = Path(__file__).resolve().parents[1] / "web"
if str(WEB_ROOT) not in sys.path:
    sys.path.insert(0, str(WEB_ROOT))


from dance_storage import (  # noqa: E402
    DancePresetNotFound,
    DanceStorage,
    DanceValidationError,
    RESERVED_STOP_TAG,
    TagConflict,
)


NUM_LEGS = 6


def _leg(leg_index: int, from_t: dict, to_t: dict, *, edited: bool = False) -> dict:
    return {
        "leg_index": leg_index,
        "edited": edited,
        "from": from_t,
        "to": to_t,
    }


def _triplet(c: float = 90.0, f: float = 90.0, t: float = 90.0) -> dict:
    return {"coxa_deg": c, "femur_deg": f, "tibia_deg": t}


def _make_dance(name: str, *, tag_id=None) -> dict:
    neutral = _triplet()
    edited_to = _triplet(c=95.0)  # just leg 0 edits coxa
    return {
        "name": name,
        "tag_id": tag_id,
        "default_segment_ms": 440,
        "calibration_snapshot": {"angles": [90.0] * 18},
        "states": [
            {
                "index": 0,
                "duration_ms": 400,
                "legs": [
                    _leg(i, neutral, edited_to if i == 0 else neutral, edited=(i == 0))
                    for i in range(NUM_LEGS)
                ],
            }
        ],
    }


class DanceStorageTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.storage = DanceStorage(Path(self.tmp.name))

    def test_save_and_get(self) -> None:
        entry = self.storage.save_dance(_make_dance("Wave Alpha", tag_id=4))
        self.assertEqual(entry["preset_id"], "wave-alpha")
        self.assertEqual(entry["tag_id"], 4)
        dance = self.storage.get_dance("wave-alpha")
        self.assertEqual(dance["name"], "Wave Alpha")
        self.assertEqual(dance["tag_id"], 4)

    def test_explicit_preset_id_is_canonicalized(self) -> None:
        dance = _make_dance("Wave Alpha", tag_id=4)
        dance["preset_id"] = "../Unsafe Preset"
        entry = self.storage.save_dance(dance)
        self.assertEqual(entry["preset_id"], "unsafe-preset")
        self.assertFalse((Path(self.tmp.name) / ".." / "Unsafe Preset").exists())

    def test_tag_uniqueness_rejected(self) -> None:
        self.storage.save_dance(_make_dance("One", tag_id=3))
        with self.assertRaises(TagConflict):
            self.storage.save_dance(_make_dance("Two", tag_id=3))

    def test_reserved_stop_tag_rejected(self) -> None:
        with self.assertRaises(TagConflict):
            self.storage.save_dance(_make_dance("NoStopPlease", tag_id=RESERVED_STOP_TAG))

    def test_list_presets_returns_index(self) -> None:
        self.storage.save_dance(_make_dance("A", tag_id=2))
        self.storage.save_dance(_make_dance("B", tag_id=3))
        listed = self.storage.list_presets()
        ids = {e["preset_id"] for e in listed}
        self.assertEqual(ids, {"a", "b"})

    def test_delete_preset_removes_file_and_index(self) -> None:
        self.storage.save_dance(_make_dance("Delete Me", tag_id=4))
        self.storage.delete_preset("delete-me")
        self.assertEqual(self.storage.list_presets(), [])
        with self.assertRaises(DancePresetNotFound):
            self.storage.get_dance("delete-me")

    def test_rebind_tag_enforces_uniqueness(self) -> None:
        self.storage.save_dance(_make_dance("Red", tag_id=2))
        self.storage.save_dance(_make_dance("Blue", tag_id=3))
        self.storage.rebind_tag("blue", None)
        blue = self.storage.get_dance("blue")
        self.assertIsNone(blue["tag_id"])
        self.storage.rebind_tag("blue", 4)
        with self.assertRaises(TagConflict):
            self.storage.rebind_tag("red", 4)

    def test_rebind_reject_reserved_tag(self) -> None:
        self.storage.save_dance(_make_dance("X", tag_id=5))
        with self.assertRaises(TagConflict):
            self.storage.rebind_tag("x", RESERVED_STOP_TAG)

    def test_state_from_must_match_prev_state_to(self) -> None:
        # Build a two-state dance where state[1].from doesn't match state[0].to
        dance = _make_dance("Broken", tag_id=None)
        bad_state = {
            "index": 1,
            "duration_ms": 400,
            "legs": [
                _leg(i, _triplet(c=99.0), _triplet(c=99.0))
                for i in range(NUM_LEGS)
            ],
        }
        dance["states"].append(bad_state)
        with self.assertRaises(DanceValidationError):
            self.storage.save_dance(dance)


if __name__ == "__main__":
    unittest.main()

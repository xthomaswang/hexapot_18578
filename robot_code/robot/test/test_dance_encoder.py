"""Tests for DanceEncoder session semantics."""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path


WEB_ROOT = Path(__file__).resolve().parents[1] / "web"
if str(WEB_ROOT) not in sys.path:
    sys.path.insert(0, str(WEB_ROOT))


from dance_encoder import DanceEncoder, EncoderError, EncoderNotActive  # noqa: E402
from dance_storage import DanceStorage  # noqa: E402


NUM_LEGS = 6


class FakeCalibrationStore:
    def __init__(self, complete: bool = True) -> None:
        legs = []
        for i in range(NUM_LEGS):
            legs.append({
                "leg_index": i,
                "angles_deg": {
                    "coxa_deg": 90.0 + i,
                    "femur_deg": 80.0 + i,
                    "tibia_deg": 100.0 + i,
                },
                "saved": complete,
            })
        self._legs = legs
        self._complete = complete

    def snapshot(self):
        return {
            "legs": [dict(l, angles_deg=dict(l["angles_deg"])) for l in self._legs],
            "saved_count": NUM_LEGS if self._complete else 0,
            "complete": self._complete,
        }


class EncoderSessionTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.storage = DanceStorage(Path(self.tmp.name))
        self.cal = FakeCalibrationStore()
        self.sent: list[list[float]] = []
        self.encoder = DanceEncoder(
            calibration_store=self.cal,
            dance_storage=self.storage,
            send_frame=lambda frame: self.sent.append(list(frame)),
        )

    def test_start_requires_complete_calibration(self) -> None:
        self.cal._complete = False
        with self.assertRaises(EncoderError):
            self.encoder.start(name="x")

    def test_draft_records_edit(self) -> None:
        self.encoder.start(name="wiggle")
        self.encoder.select_leg(0)
        self.encoder.draft_leg(0, {"coxa_deg": 95.0, "femur_deg": 80.0, "tibia_deg": 100.0})
        snap = self.encoder.snapshot()
        self.assertTrue(snap["current_state_edited"][0])
        # preview frame = base neutral with leg 0's to pose applied
        frame = self.encoder.preview_frame()
        self.assertEqual(frame[0], 95.0)
        self.assertEqual(frame[1], 80.0)

    def test_save_state_requires_at_least_one_edit(self) -> None:
        self.encoder.start(name="emptytest")
        with self.assertRaises(EncoderError):
            self.encoder.save_state()

    def test_save_state_advances_and_chains_from(self) -> None:
        self.encoder.start(name="chain")
        self.encoder.select_leg(0)
        self.encoder.draft_leg(0, {"coxa_deg": 95.0, "femur_deg": 80.0, "tibia_deg": 100.0})
        self.encoder.save_state()
        snap = self.encoder.snapshot()
        self.assertEqual(snap["saved_state_count"], 1)
        self.assertEqual(snap["current_state_index"], 1)
        # The new state's from[0] must equal the saved state's to[0]
        self.assertEqual(snap["current_state_from"][0]["coxa_deg"], 95.0)
        self.assertEqual(snap["current_state_to"][0]["coxa_deg"], 95.0)  # initial to = from

    def test_save_dance_persists_and_clears(self) -> None:
        self.encoder.start(name="keeper")
        self.encoder.select_leg(2)
        self.encoder.draft_leg(2, {"coxa_deg": 92.0, "femur_deg": 82.0, "tibia_deg": 102.0})
        self.encoder.save_state()
        entry = self.encoder.save_dance(tag_id=7)
        self.assertEqual(entry["preset_id"], "keeper")
        self.assertEqual(entry["tag_id"], 7)
        # After save the encoder session is cleared
        snap = self.encoder.snapshot()
        self.assertFalse(snap["active"])
        # Dance is retrievable from storage
        dance = self.storage.get_dance("keeper")
        self.assertEqual(dance["tag_id"], 7)
        self.assertEqual(len(dance["states"]), 1)
        leg2 = dance["states"][0]["legs"][2]
        self.assertTrue(leg2["edited"])
        self.assertAlmostEqual(leg2["to"]["coxa_deg"], 92.0)

    def test_multiple_legs_in_same_state(self) -> None:
        self.encoder.start(name="multi")
        self.encoder.select_leg(0)
        self.encoder.draft_leg(0, {"coxa_deg": 95.0, "femur_deg": 80.0, "tibia_deg": 100.0})
        self.encoder.select_leg(3)
        self.encoder.draft_leg(3, {"coxa_deg": 88.0, "femur_deg": 85.0, "tibia_deg": 105.0})
        self.encoder.save_state()
        dance_state = self.encoder._saved_states[0]
        self.assertTrue(dance_state["legs"][0]["edited"])
        self.assertTrue(dance_state["legs"][3]["edited"])
        self.assertFalse(dance_state["legs"][1]["edited"])


if __name__ == "__main__":
    unittest.main()

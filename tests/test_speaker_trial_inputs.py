"""Protect physical setup variation and the nominal-target boundary."""
import csv
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim/pressure"))
from speaker_trial_inputs import COHORT, FRAME, MARKS, local_offset, placed_marks, read_attempt, verify_start
from run_axial_shear_lift import digest


class TrialInputs(unittest.TestCase):
    def test_twenty_distinct_predeclared_setups_span_each_dimension(self):
        rows = [read_attempt(COHORT, f"R{i:02}") for i in range(1, 21)]
        for key in rows[0]:
            limit = 1. if key.endswith("_deg") else 10.
            expected = np.linspace(-.95*limit, .95*limit, 20)
            np.testing.assert_allclose(sorted(r[key] for r in rows), expected, atol=1e-12)
        for mark in MARKS:
            target = FRAME.mark(mark)
            for r in rows:
                local = FRAME.to_local(target)
                local[:2] += local_offset(r, mark)
                shift = (FRAME.to_field(local) - target)[:2]
                np.testing.assert_allclose(shift*1000, [r[mark+"_x_mm"], r[mark+"_y_mm"]], atol=1e-10)

    def test_refuses_ambiguous_or_nonfinite_setup(self):
        with COHORT.open(newline="") as stream:
            rows = list(csv.reader(stream))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "roster.csv"
            for body in ([rows[0], rows[1], rows[1]], [rows[0], [*rows[1][:-1], "nan"]]):
                with path.open("w", newline="") as stream:
                    csv.writer(stream).writerows(body)
                with self.assertRaises(ValueError):
                    read_attempt(path, "R01")

    def test_measured_start_cannot_move_the_target_or_use_another_setup(self):
        roster = COHORT.parent / "speaker-pickup-development-starts.csv"
        evidence = json.loads((COHORT.parent / "speaker-repeatability-pickup-development.json").read_bytes())
        observed = evidence["runs"]["D1-tactile"]
        initial = observed["initialQ"]
        start = dict(observed, schema="varied-first-wing-acquisition/1", rosterSha256=digest(roster),
                     pieceId="blue-wing-middle", collectedMarks=["middle"], start=0.,
                     initialV=[0.]*15, initialRobotQ=initial[:9], initialRobotV=[0.]*9,
                     initialRotors=[0.]*4, initialBusV=12.)
        verify_start(start, roster)
        with self.assertRaises(ValueError):
            verify_start(dict(start, targetLocalXY=start["waitingNoteLocalXY"]), roster)
        with self.assertRaises(ValueError):
            verify_start(dict(start, attempt="D2"), roster)
        with self.assertRaises(ValueError):
            placed_marks(FRAME, start, COHORT)
        with self.assertRaises(ValueError):
            placed_marks(FRAME, start, None)

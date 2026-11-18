"""Regression checks for whole-material target and uncertain optical paths."""
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim/pressure"))
from speaker_score_observer import HIGH, LOW, MAJOR, beam_samples, opening_passage, shadow_margin


class SpeakerOpening(unittest.TestCase):
    def crossing(self, lateral):
        direction = np.cross((HIGH-LOW)/np.linalg.norm(HIGH-LOW), [0., 1., 0.])
        center = (LOW+HIGH)/2 + [0., lateral, 0.]
        return [dict(t=i*.001, q=[*(center+d*direction), 1., 0., 0., 0.])
                for i, d in enumerate(np.linspace(-.4, .4, 101))]

    def test_full_ring_crosses_before_counter(self):
        result = opening_passage(self.crossing(0.))
        self.assertGreater(result["minimumMarginM"], .025)
        self.assertGreater(result["fullyThroughByS"], result["firstPossibleS"])

    def test_ring_edge_can_miss_with_center_inside(self):
        self.assertIsNone(opening_passage(self.crossing(.45)))

    def test_starting_beyond_opening_earns_no_passage(self):
        self.assertIsNone(opening_passage(self.crossing(0.)[-10:]))


class OpticalPaths(unittest.TestCase):
    q = np.array([0., 0., 0., 1., 0., 0., 0.])

    def rectangle(self, x, half_width):
        low = [[x+i*half_width, j*.01, -.04] for i in (-1, 1) for j in (-1, 1)]
        high = np.asarray(low) + [0., 0., .08]
        return beam_samples(low, high)

    def test_ring_material_covers_every_ray(self):
        self.assertGreater(shadow_margin(self.q, self.rectangle(MAJOR, .006)), .01)

    def test_beam_through_hole_stays_clear(self):
        self.assertLess(shadow_margin(self.q, self.rectangle(0., .006)), 0.)

    def test_center_ray_alone_cannot_score(self):
        self.assertLess(shadow_margin(self.q, self.rectangle(MAJOR, .032)), 0.)

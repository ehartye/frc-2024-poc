"""Counterexamples for native route verification."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"sim/pressure"))
from native_route_envelopes import arc_bounds, envelopes
from verify_native_route_sweep import partition


class NativeEnvelopes(unittest.TestCase):
    def setUp(self):
        self.pose = dict(x=178.,y=0.,z=25.4,p=0.,theta=0.,pivot=25.4,t=0.)

    def test_carried_translation_cancels_in_both_moving_frames(self):
        b = dict(self.pose,z=75.4,pivot=75.4,t=1.)
        fixed,carriage,cradle = arc_bounds(self.pose,b,152.4,178.)
        self.assertAlmostEqual(fixed,50.)
        self.assertEqual((carriage,cradle),(0.,0.))

    def test_offset_piece_rotates_in_cradle_frame(self):
        a = dict(self.pose,x=198.)
        b = dict(a,theta=.5,p=.5)
        self.assertAlmostEqual(arc_bounds(a,b,152.4,178.)[2],10.)

    def test_returns_to_same_endpoint_after_interior_excursion(self):
        mid = dict(self.pose,z=35.4,t=.5)
        last = dict(self.pose,t=1.)
        result, = envelopes([self.pose, mid, last],2,152.4,178.)
        self.assertEqual(result["z"],mid["z"])
        self.assertTrue(math.isclose(result["padMm"]["fixed"],10.002))
        self.assertEqual((result["begin"],result["end"]),(0.,1.))

    def test_exact_lateral_bound_survives_motion_and_tight_belt_clearance(self):
        b = dict(self.pose,x=300.,theta=.5,p=.4,t=.5)
        result, = envelopes([self.pose,b],1,152.4,178.)
        self.assertAlmostEqual(result["yBoundsMm"][1],177.802)
        self.assertGreater(result["padMm"]["fixed"],100.)

    def test_missing_tail_after_replayed_subdivision(self):
        with self.assertRaisesRegex(ValueError,"gap, overlap or repeated"):
            partition([dict(first=0,last=4),dict(first=0,last=4)],0,8)
        with self.assertRaisesRegex(ValueError,"Missing"):
            partition([dict(first=0,last=4)],0,8)
        partition([dict(first=0,last=4),dict(first=4,last=8)],0,8)


if __name__ == "__main__":
    unittest.main()

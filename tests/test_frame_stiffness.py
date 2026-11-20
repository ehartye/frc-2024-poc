"""Reproduce the measured geometry selection and the matched load study."""
import json
import sys
import unittest
from pathlib import Path

FOLDER = Path(__file__).resolve().parents[1] / "cad/service-readiness/stiffness"
sys.path.insert(0,str(FOLDER))
from study import study


class FrameStiffness(unittest.TestCase):
    def test_replay(self):
        actual = study()
        stored = json.loads((FOLDER/"result.json").read_bytes())
        def compare(a,b):
            if isinstance(b,dict):
                self.assertEqual(a.keys(),b.keys())
                for key in b:
                    compare(a[key],b[key])
            elif isinstance(b,float):
                self.assertAlmostEqual(a,b,delta=max(1e-5,abs(b)*1e-4))
            else:
                self.assertEqual(a,b)
        compare(actual,stored)
        self.assertEqual(actual["selection"]["retainedParkedBodies"],909)
        self.assertEqual(len(actual["selection"]["addedArticles"]),17)
        for mode in ("rigid","firm","relaxed","softWelds"):
            self.assertEqual(actual["R9"][mode],actual["R14"][mode])
            self.assertGreater(actual["R9"][mode]["twistNmPerDegree"],actual["R13"][mode]["twistNmPerDegree"])
        self.assertGreater(actual["lowerTie"]["rigid"]["twistNmPerDegree"],actual["R9"]["rigid"]["twistNmPerDegree"])
        self.assertLess(actual["lowerTie"]["softWelds"]["twistNmPerDegree"],actual["R9"]["softWelds"]["twistNmPerDegree"])
        self.assertFalse(actual["R14"]["rocking"]["twistEquilibrium"])
        self.assertAlmostEqual(actual["controls"]["oneTieEndUnfastened"]["twistNmPerDegree"],
                               actual["R13"]["relaxed"]["twistNmPerDegree"],delta=.2)


if __name__ == "__main__":
    unittest.main()

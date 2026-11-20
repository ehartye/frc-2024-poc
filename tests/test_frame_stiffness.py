"""Reproduce the measured geometry selection and the matched load study."""
import hashlib
import json
import sys
import unittest
from pathlib import Path

FOLDER = Path(__file__).resolve().parents[1] / "cad/service-readiness/stiffness"
sys.path.insert(0,str(FOLDER))
from study import study


class FrameStiffness(unittest.TestCase):
    def test_native_service(self):
        r = json.loads((FOLDER/"native-fit.json").read_bytes())
        student = FOLDER.parents[1]/"student"
        for name,key in (("r14-removable-frame-tie","archiveSha256"),
                         ("r13-ground-frame","fixtureParentSha256"),("r9-physical-assembly","parentSha256")):
            self.assertEqual(r[key],hashlib.sha256((student/(name+".f3d")).read_bytes()).hexdigest())
        for name,digest in r["imageSha256"].items():
            self.assertEqual(digest,hashlib.sha256((FOLDER/name).read_bytes()).hexdigest())
        self.assertEqual((r["visibleBodies"],r["fullParkedVisibleBodies"]),(34,909))
        stations = r["stationResults"]
        self.assertEqual({tuple(s["centerMm"]) for s in stations},{(x,y) for x in (23,53) for y in (-329,329)})
        for station in stations:
            self.assertEqual(set(station["sweeps"]),{"driver","socket","nutAndWasherDown","shaftUp",
                                                    "headUp","inspectionStem","inspectionGrip"})
            self.assertFalse(any(station["sweeps"].values()))
            self.assertGreater(station["retainedBoltMm3"],4900)
        self.assertEqual((r["liftMm"],r["lateralRemovalMm"]),(5,800))
        self.assertFalse(r["riseHits"] or r["lateralHits"])
        self.assertGreater(r["bumperInstalledControlMm3"]["Left drilled bumper wood"],36000)
        self.assertEqual(len(r["reopens"]),3)
        for reopen in r["reopens"]:
            self.assertTrue(reopen["restored"])
            self.assertEqual(reopen["before"],reopen["after"])
            self.assertEqual(reopen["phase"],"ok")

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

"""Regression checks for interval-based retention evidence."""
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"sim/pressure"))
from verify_retained_pickup import evaluate


class HoldEvidence(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.folder = Path(self.temp.name)
        self.dt = .0005
        result = dict(outcome="HORIZON", dt=self.dt, startTimeS=0.,
                      inheritedResidualJ=0., initialMechanicalDifferenceJ=0.)
        (self.folder/"result.json").write_text(json.dumps(result))

    def measurement(self, changed=None):
        with (self.folder/"trace.jsonl").open("w") as trace:
            for step in range(1,1601):
                t = step*self.dt
                z = .0254+min(1.,t/.5)*.008
                s = dict(timeS=t, chassis=dict(x=0.,vx=0.), note=dict(p=[0.],v=[0.]),
                         zPivot=z, noteZ=z, vPivot=0., noteVz=0., theta=0., notePitch=0.,
                         thetaRate=0., notePitchRate=0.)
                row = dict(timeS=t,state=s,normalN=[2.,2.],floorForceN=[0.,0.,0.],
                           energyJ=dict(residual=0.),sourcePowerResidualW=0.,
                           portMidpointPowerDifferenceW=0.,pressureWorkResidualW=0.)
                if step==1300 and changed:
                    changed(row)
                trace.write(json.dumps(row)+"\n")
        return evaluate(self.folder)[1]["retained"]

    def test_complete_hold(self):
        self.assertTrue(self.measurement())

    def test_intervening_floor_contact(self):
        self.assertFalse(self.measurement(lambda row: row.update(floorForceN=[0.,0.,1.])))

    def test_ring_edge_intersects_despite_positive_center_rise(self):
        self.assertFalse(self.measurement(lambda row: row["state"].update(notePitch=.1)))

    def test_one_sided_release(self):
        self.assertFalse(self.measurement(lambda row: row.update(normalN=[2.])))


if __name__=="__main__":
    unittest.main()

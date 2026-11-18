"""Adversarial traces for the full-height acceptance measurement."""
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"sim/pressure"))
from verify_retained_route import assess


class RouteEvidence(unittest.TestCase):
    def measure(self,change=None):
        dt=.0005
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp)
            (folder/"result.json").write_text(json.dumps(dict(outcome="HORIZON",dt=dt,
                    startTimeS=0.,grid=1024,inputs={})))
            with (folder/"trace.jsonl").open("w") as stream:
                for i in range(1,11601):
                    t=i*dt
                    f=min(1.,t/4.)
                    angle=math.pi/3*f
                    z=.0254+(.4748333949243112-.0254)*f
                    state=dict(timeS=t,chassis=dict(x=0.,vx=0.),note=dict(p=[.178],v=[0.]),
                            zPivot=z,noteZ=z,vPivot=0.,noteVz=0.,theta=angle,notePitch=angle,
                            thetaRate=0.,notePitchRate=0.)
                    row=dict(timeS=t,state=state,floorForceN=[0.,0.,0.],normalN=[2.,2.],
                            minimumFloorClearanceM=z-.0254-.1524*math.sin(angle),
                            energyJ=dict(residual=0.),portMidpointPowerDifferenceW=0.,
                            sourcePowerResidualW=0.,pressureWorkResidualW=0.)
                    if change and i==8200:
                        change(row)
                    stream.write(json.dumps(row)+"\n")
            return assess(folder)[1]["retained"]

    def test_successful_interval(self):
        self.assertTrue(self.measure())

    def test_transient_release_while_still_at_height(self):
        self.assertFalse(self.measure(lambda r:r.update(normalN=[4.,0.])))

    def test_earlier_floor_contact_despite_successful_endpoint(self):
        self.assertFalse(self.measure(lambda r:r.update(floorForceN=[0.,0.,1.])))

    def test_nonfinite_support(self):
        with self.assertRaisesRegex(ValueError,"Nonfinite"):
            self.measure(lambda r:r.update(normalN=[float("nan"),2.]))


if __name__=="__main__":
    unittest.main()

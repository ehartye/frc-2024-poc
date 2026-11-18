"""Regression checks for native pose registration and contact evidence."""
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"sim/pressure"))
from handoff_native_poses import frames
from verify_launcher_handoff import examine


class NativeFeedPose(unittest.TestCase):
    def test_rotation_and_lift_are_removed_in_the_cradle_frame(self):
        # A ring at the cradle pivot remains at that pivot in its source frame.
        yaw, pitch = .83, .96
        cy, sy = math.cos(yaw), math.sin(yaw)
        cp, sp = math.cos(pitch/2), math.sin(pitch/2)
        c, s = math.cos(yaw/2), math.sin(yaw/2)
        q = [2., 1., yaw, .449, pitch, 0., 0., 0., 0.,
             2.+.178*cy, 1.+.178*sy, .0254+.449, c*cp, s*sp, -c*sp, s*cp]
        result = frames(q, 9)
        np.testing.assert_allclose(result["cradle"]["center"], [178.,0.,25.4], atol=1e-9)
        np.testing.assert_allclose(result["cradle"]["normal"], [0.,0.,1.], atol=1e-12)


class MountedDrive(unittest.TestCase):
    def test_wheel_mounts_preserve_generalized_mass_and_the_incoming_note_state(self):
        try:
            import mujoco
        except ImportError:
            self.skipTest("MuJoCo is required for the mechanical model")
        from launcher_handoff import RaisedFeed
        acquired = dict(chassis=dict(x=0., y=0., heading=0., vx=0., vy=0., yaw_rate=0., wheel=[0.,0.]),
                        note=dict(p=[.178,0.], v=[0.,0.], yaw=0., spin=0., w=[0.,0.]))
        q = [.3,.2,.8,.449,1.047,.4,.3,.475,1.,0.,0.,0.]
        v = [.1,.2,.23,0.,0.,.11,.22,0.,.06,.07,.08]
        row = dict(q=q,v=v,rotor=[1.,2.,3.,4.],shear=[0.]*4,busV=11.9,clock=5.)
        model = RaisedFeed(acquired,row,.0001,256,512,dict(referenceStiffness=4350.,referenceDamping=132.))
        old = model.base
        old.data.qpos[:]=q
        old.data.qvel[:]=v
        mujoco.mj_forward(old.robot,old.data)
        before=np.empty((old.robot.nv,old.robot.nv))
        after=np.empty((model.model.nv,model.model.nv))
        mujoco.mj_fullM(old.robot,old.data,before)
        mujoco.mj_fullM(model.model,model.data,after)
        retained = list(range(5))+list(range(model.nv,model.nv+6))
        np.testing.assert_allclose(after[np.ix_(retained,retained)],before,atol=1e-10,rtol=0)
        np.testing.assert_allclose(model.data.xpos[model.note],old.data.xpos[old.body["note"]])
        np.testing.assert_array_equal(model.data.qvel[retained],v)
        np.testing.assert_array_equal(model.data.qvel[model.wheel_dofs],np.zeros(4))


class ContinuousContact(unittest.TestCase):
    def test_upward_free_flight_cannot_replace_missing_simultaneous_contact(self):
        dt = .001
        rows = [dict(time=2+i*dt, q=[0.]*16, v=[0.]*9+[0.,0.,1.,0.,0.,0.],
                     beltNormalN=0., wheelNormalN=0., busV=12.) for i in range(1,42)]
        rows[0].update(wheelNormalN=1., beltNormalN=1.)
        receipt = dict(schema="native-raised-feed/1", runnerSha256="fixture", plantSha256="fixture", start=2.,end=rows[-1]["time"],initialV=[0.]*15,
                       outcome="UPWARD_FREE_RELEASE",case="nominal",dt=dt,traceStride=1,
                       sourceTraceSha256="fixture",nativeSha256="fixture",grid=256,segments=512)
        with tempfile.TemporaryDirectory() as name:
            folder = Path(name)
            def write(samples):
                content = "".join(json.dumps(x)+"\n" for x in samples).encode()
                (folder/"trace.jsonl").write_bytes(content)
                receipt["traceSha256"] = hashlib.sha256(content).hexdigest()
                (folder/"result.json").write_text(json.dumps(receipt))
            write(rows)
            self.assertTrue(examine(folder)[0]["attainedUpwardRelease"])
            separate = copy.deepcopy(rows)
            separate[0]["beltNormalN"] = 0.
            write(separate)
            with self.assertRaisesRegex(ValueError, "contact history"):
                examine(folder)
            write(rows[:10]+rows[11:])
            with self.assertRaisesRegex(ValueError, "Missing or retimed"):
                examine(folder)

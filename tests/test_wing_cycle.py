"""Test continuation and scoring conditions that a single-shot fixture cannot cover."""
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim/pressure"))
from verify_two_wing_notes import distinct_events


def event(source, time):
    return dict(sourceTrace=source, firstSimultaneousFullShadowS=time, opening=dict(fullyThroughByS=4.))


class ScoreLedger(unittest.TestCase):
    def test_annular_shell_uses_bore_and_finite_axial_extent(self):
        from refine_shell_clearance import ring_distances
        # At azimuth zero this ring basis puts the centerline point at +Y.
        normal=np.array([0.,0.,1.])
        origin=np.array([0.,-152.4,0.])
        def sampled(center):
            return ring_distances(origin+center,normal,np.zeros(3),np.array([[1.]]),np.array([[0.]])).item()
        self.assertAlmostEqual(sampled([0.,0.,0.]),42.2275-25.4)
        self.assertAlmostEqual(sampled([47.,0.,0.]),.01-25.4)
        self.assertAlmostEqual(sampled([44.,65.875,0.]),50.-25.4)

    def test_field_frame_preserves_all_six_mark_locations_after_a_turn(self):
        from wing_field_frame import SUPPLY_MARKS, WingFrame
        frame=WingFrame("middle",245.)
        np.testing.assert_allclose(frame.to_field([.658,0.,0.]),[2.8956,5.553075,0.])
        self.assertEqual(len(set(SUPPLY_MARKS.values())),6)
        for mark in SUPPLY_MARKS:
            target=frame.mark(mark)
            np.testing.assert_allclose(frame.to_field(frame.to_local(target)),target,atol=1e-10)

    def test_field_audit_reports_a_bumper_crossing_between_clear_endpoints(self):
        from field_route_clearance import audit
        from wing_field_frame import WingFrame
        frame=WingFrame("middle",0.)
        boxes=[dict(motionGroup=g,boundsMm=[-20,-20,30,20,20,80],key=g)
               for g in ("fixed","carriage","cradle")]
        center=frame.to_field([.2,0.,0.])
        field=[dict(path=["field","post"],bounds=[(center+[-.01,-.01,.035]).tolist(),
                                                  (center+[.01,.01,.075]).tolist()])]
        rows=np.array([[t,x,0.,0.,0.,0.,-1.,0.,.0254] for t,x in [(0.,0.),(.001,.2),(.002,.4)]])
        result=audit(rows,frame,boxes,field,[])
        self.assertGreater(result["unresolvedIntervals"],0)
        self.assertIn(("fixed","field/post"),result["firstUnresolved"]["pairs"])

    def test_field_processing_can_overlap_next_pickup(self):
        rows = [("wing-1","first",event("first",9.)),("wing-2","second",event("second",8.))]
        self.assertEqual([r["piece"] for r in distinct_events(rows)], ["wing-2","wing-1"])

    def test_repeat_trial_and_two_ids_for_one_release_are_rejected(self):
        for piece,source in [("wing-1","second"),("wing-2","first")]:
            with self.assertRaises(ValueError):
                distinct_events([("wing-1","first",event("first",6.)),(piece,source,event(source,7.))])

    def test_miss_and_out_of_order_counter(self):
        self.assertEqual(distinct_events([("wing-1","first",event("first",None))]),[])
        with self.assertRaises(ValueError):
            distinct_events([("wing-1","first",event("first",3.))])


class MovingReturn(unittest.TestCase):
    def test_robot_keeps_its_motion_while_a_distinct_floor_note_starts_at_rest(self):
        try:
            import mujoco
        except ImportError:
            self.skipTest("MuJoCo is required for the mechanical model")
        from second_wing_pickup import ReturnPlant, MIDDLE_WING
        acquired=dict(chassis=dict(x=0.,y=0.,heading=0.,vx=0.,vy=0.,yaw_rate=0.,wheel=[0.,0.]),
                      note=dict(p=[.178,0.],v=[0.,0.],yaw=0.,spin=0.,w=[0.,0.]))
        z=.0254-.235*9.80665/20000.
        seed=dict(q=[2.,1.4,1.54,.449,1.047,*MIDDLE_WING,z,1.,0.,0.,0.],
                  v=[0.,.30,.2,0.,0.,0.,0.,0.,0.,0.,0.],rotor=[3.,5.,-156.,-159.],
                  shear=[0.]*4,busV=10.6,clock=17.05)
        saved=dict(q=seed["q"][:5]+[77.,79.,80.,82.]+[2.2,1.9,1.,1.,0.,0.,0.],
                   v=seed["v"][:5]+[299.,298.,300.,302.]+[0.,5.,9.,0.,1.,0.],
                   time=seed["clock"],sourceJ=270.,beltNormalN=0.,wheelNormalN=0.,shearM=[0.]*4)
        model=ReturnPlant(acquired,seed,.0005,128,32,dict(referenceStiffness=4350.,referenceDamping=132.))
        model.resume(saved)
        np.testing.assert_array_equal(model.data.qpos[:9],saved["q"][:-7])
        np.testing.assert_array_equal(model.data.qvel[:9],saved["v"][:-6])
        np.testing.assert_array_equal(model.data.qvel[-6:],np.zeros(6))
        np.testing.assert_array_equal(model.data.qpos[-7:-4],[*MIDDLE_WING,z])
        belt,floor,_=model.contact()
        self.assertAlmostEqual(floor[2],.235*9.80665,places=10)
        self.assertEqual(sum(belt["normal"]),0.)
        saved["wheelNormalN"]=1.
        with self.assertRaises(ValueError):
            model.resume(saved)

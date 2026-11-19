"""Regression checks for whole-material target and uncertain optical paths."""
import sys
import unittest
import json
import tempfile
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


class ConsecutiveScores(unittest.TestCase):
    def test_join_at_another_flights_clock_integrates_the_final_fraction(self):
        from types import SimpleNamespace
        from unittest.mock import patch
        from coupled_collection import run
        from counter_reset import centers
        from run_axial_shear_lift import digest

        step=.0001
        inputs=[]
        for index in range(2):
            name=f"before-{index}"
            self.segment(name,name,0,[index,index])
            folder=self.root/name
            receipt=json.loads((folder/"result.json").read_bytes())
            receipt["parameters"].update(dt=step,nodes=32,stiffness=1.2e6,damping=450.,friction=.45,
                                         belt_speed=1.5,first_mark="middle",field_yaw_deg=245.,collection_floor=True)
            rows=[dict(t=t,q=[index,0.,2.,1.,0.,0.,0.],v=[0.]*6) for t in (0.,step)]
            trace=folder/"trace.jsonl"
            trace.write_text("".join(json.dumps(row)+"\n" for row in rows))
            receipt.update(trace=digest(trace),endS=step)
            (folder/"result.json").write_text(json.dumps(receipt))
            inputs.append(folder)

        # The free-flight case has no surfaces nearby. Gravity gives an
        # independent check that the last half step advanced physical state.
        field=SimpleNamespace(pins={"registered":"same"},close=lambda:None)
        model=SimpleNamespace(SpeakerSurfaces=lambda path:field)
        law=SimpleNamespace(field_wrench=lambda *args:(np.zeros(6),{},0.),
                            collection_floor=lambda *args:(np.zeros(6),0.))
        out=self.root/"joint"
        with patch.dict(sys.modules,{"speaker_surfaces":model,"run_speaker_field":law}):
            run(inputs,self.root,out,until=3.5*step)
        for i in range(2):
            rc,poses,_,last=centers(out/str(i))
            self.assertAlmostEqual(rc["finalStepS"],step/2)
            np.testing.assert_allclose(poses[:,0],step*np.array([1,2,3,3.5]),atol=1e-12,rtol=0)
            self.assertAlmostEqual(last["v"][2],-9.80665*2.5*step)
            self.assertAlmostEqual(last["q"][2],2.-9.80665*step**2*(1+2+1.25))
            self.assertEqual(rc["coupledManifest"],digest(out/"manifest.json"))
            rc["finalStepS"]=step
            (out/str(i)/"result.json").write_text(json.dumps(rc))
            with self.assertRaisesRegex(ValueError,"Incomplete field time history"):
                centers(out/str(i))

    def test_collection_contact_requires_the_same_joint_solve(self):
        from counter_reset import collection_pair
        low=np.array([[t,0.,0.,0.,1.,0.,0.,0.] for t in (0.,.001,.002)])
        high=low.copy()
        high[:,3]=.047
        joint=[(0.,.002,"shared")]
        result=collection_pair(low,joint,high,joint,0.,.002)
        self.assertEqual(result["coupledManifests"],["shared"])
        with self.assertRaisesRegex(ValueError,"coupled contact"):
            collection_pair(low,joint,high,[(0.,.002,"different")],0.,.002)

    def test_parallel_rings_clear_even_when_their_bounding_spheres_overlap(self):
        from counter_reset import separate_paths
        low=np.array([[t,0.,0.,0.,1.,0.,0.,0.] for t in (0.,.001,.002)])
        high=low.copy()
        high[:,3]=.12
        self.assertGreater(separate_paths(low,high,0.,.002),.06)

    def test_shared_ring_contact_conserves_total_force_and_moment(self):
        from types import SimpleNamespace
        from coupled_collection import pair_wrenches
        angles=np.arange(192)*2*np.pi/192
        ring=MAJOR*np.column_stack((np.cos(angles),np.sin(angles),np.zeros(len(angles))))
        def body(z,vy):
            center=np.array([0.,0.,z])
            return SimpleNamespace(center=center,rotation=np.eye(3),points=center+ring,
                                   velocity=np.array([0.,vy,0.]),omega=np.zeros(3))
        p=SimpleNamespace(stiffness=1200000.,damping=450.,friction=.45)
        a,b=body(0.,.25),body(.048,0.)
        wrench,depth=pair_wrenches(a,b,p)
        self.assertAlmostEqual(depth,.0028)
        self.assertLess(wrench[0,2],0.)
        self.assertLess(wrench[0,1],0.)
        np.testing.assert_allclose(wrench[:,:3].sum(axis=0),0.,atol=1e-9)
        moment=wrench[:,3:].sum(axis=0)+np.cross(a.center,wrench[0,:3])+np.cross(b.center,wrench[1,:3])
        np.testing.assert_allclose(moment,0.,atol=1e-9)
        separate,depth=pair_wrenches(a,body(.12,0.),p)
        self.assertEqual(depth,0.)
        np.testing.assert_array_equal(separate,np.zeros((2,6)))

    def setUp(self):
        from run_axial_shear_lift import digest
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.sensor = self.root / "sensor.json"
        self.sensor.write_text(json.dumps({"sensorCaseEnds": [
            {"caseEndCornersM": [[-.02, -.02, -.01], [.02, .02, .01]]}
        ]}))
        self.previous = self.root / "score.json"
        score = self.segment("prior", "first", 0, [0, 0, 1, 2, 2, 2, 2])
        self.previous.write_text(json.dumps(dict(shot="first", native="same", scoreS=1.,
            field=score, sensorObservation={"sensorFile": digest(self.sensor)})))

    def segment(self, name, shot, start, positions, release=None):
        from run_axial_shear_lift import digest
        folder = self.root / name
        folder.mkdir()
        trace = folder / "trace.jsonl"
        rows = [dict(t=float(start+i), q=[x, 0, 0, 1, 0, 0, 0], v=[0]*6)
                for i, x in enumerate(positions)]
        trace.write_text("".join(json.dumps(row)+"\n" for row in rows))
        pin = digest(trace)
        (folder / "result.json").write_text(json.dumps(dict(
            parameters=dict(trace_every_step=True, dt=1.), sourceTrace=shot, trace=pin,
            field={"registered": "same"}, nativeSha256="same", segmentStartS=start,
            releaseTimeS=start if release is None else release, endS=rows[-1]["t"])))
        return pin

    def check(self, name):
        from counter_reset import check
        return check(self.previous, [self.root/"prior"], self.root/name, self.sensor)

    def test_reset_detects_passage_and_piece_separation(self):
        self.segment("next", "second", 3, [-2, -1, 0])
        result = self.check("next")
        self.assertEqual(result["sensorClearByS"], 2.)
        self.assertGreater(result["minimumPieceSeparationM"], 1.)

    def test_intersecting_paths_between_clear_endpoints_need_contact_solve(self):
        self.segment("intersect", "second", 3, [0, 4])
        with self.assertRaisesRegex(ValueError, "coupled contact"):
            self.check("intersect")

    def test_cannot_start_at_a_later_field_segment_or_reuse_a_piece(self):
        self.segment("late", "second", 3, [0, 0], release=2)
        self.segment("duplicate", "first", 3, [0, 0])
        for name in ("late", "duplicate"):
            with self.subTest(name=name), self.assertRaisesRegex(ValueError, "Distinct concurrent"):
                self.check(name)

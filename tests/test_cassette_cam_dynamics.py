import unittest,pathlib,sys,math,json,hashlib,random
from dataclasses import replace,asdict
ROOT=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'cad/assembly'))
from cassette_cam_dynamics import Parameters,State,ports,step,equilibrium,DomainStop
from check_cassette_cam_dynamics import run,PARAMETERS,trajectory
class CassetteDynamics(unittest.TestCase):
    def test_positive_hypotheses_and_finite_validation(self):
        for name in PARAMETERS.__dataclass_fields__:
            for value in [True,float('nan'),float('inf'),-1,10**400]:
                with self.assertRaises(ValueError):replace(PARAMETERS,**{name:value}).validate()
            if name!='contact_damping_ns_m':
                with self.assertRaises(ValueError):replace(PARAMETERS,**{name:0}).validate()
        with self.assertRaises(ValueError):ports(State(.001,1e308,1,0),PARAMETERS,0)
    def test_energy_rate_and_reciprocal_note_work(self):
        rng=random.Random(492)
        for _ in range(200):
            s=State(rng.uniform(0,.0015),rng.uniform(-.1,.1),rng.uniform(0,math.pi),rng.uniform(-2,2))
            load=rng.uniform(-10,10);r=ports(s,PARAMETERS,.002,load)
            self.assertAlmostEqual(r['instantaneous_power_residual'],0,places=11)
            self.assertEqual(r['load_power'],load*s.udot)
            self.assertGreaterEqual(r['heat_rate'],0)
    def test_held_open_is_not_latch(self):
        s,t=equilibrium(.2,PARAMETERS)
        self.assertNotEqual(t,0)
        held=step(s,PARAMETERS,1e-5,t);off=step(s,PARAMETERS,1e-5,0)
        self.assertAlmostEqual(held.theta,s.theta,places=14)
        self.assertGreater(off.theta,s.theta)
        self.assertFalse(ports(s,PARAMETERS,t)['positive_latch'])
    def test_dead_center_perturbation(self):
        s,t=equilibrium(0,PARAMETERS)
        self.assertEqual(step(s,PARAMETERS,1e-5,0).theta,0)
        s=replace(s,theta=.001)
        self.assertGreater(step(s,PARAMETERS,1e-5,0).thetadot,0)
    def test_static_stop_and_atomic_impact(self):
        s=State(.0015,0,math.pi,0);r=ports(s,PARAMETERS,0)
        self.assertLess(r['stop_force'],0);self.assertEqual(r['stop_power'],0)
        self.assertEqual(step(s,PARAMETERS,1e-5,0),replace(s,time=1e-5))
        impact=State(.001499,1,math.pi,0,actuator_work=3,load_work=4,contact_heat=5)
        before=asdict(impact)
        with self.assertRaises(DomainStop):step(impact,PARAMETERS,1e-4,0)
        self.assertEqual(asdict(impact),before)
        with self.assertRaises(ValueError):step(State(.0015,0,math.pi,0,time=1e308),PARAMETERS,1e-5,0)
        with self.assertRaises(DomainStop):ports(State(0,-1,0,0),PARAMETERS,0)
    def test_fault_continuity_and_torque_bound(self):
        s,t=equilibrium(.2,PARAMETERS);s=replace(s,udot=.002,thetadot=.1,actuator_work=2,load_work=3,contact_heat=4)
        r=ports(s,PARAMETERS,1e3);self.assertEqual(r['applied_torque'],PARAMETERS.torque_limit_nm)
        off=step(s,PARAMETERS,1e-5,0)
        self.assertNotEqual(off.udot,0);self.assertNotEqual(off.thetadot,0)
        self.assertEqual(off.actuator_work,2);self.assertEqual(off.load_work,3);self.assertGreaterEqual(off.contact_heat,4)
        self.assertIsNone(r['electrical_current_a'])
    def test_energy_convergence(self):
        s,t=equilibrium(.2,PARAMETERS);s=replace(s,udot=.002,thetadot=.1)
        a=trajectory(s,4e-5,.01,t);b=trajectory(s,2e-5,.01,t)
        self.assertLess(b['maxEnergyResidualJ'],a['maxEnergyResidualJ']/4)
        self.assertLess(abs(a['final']['u']-b['final']['u']),1e-9)
    def test_mass_inertia_sensitivity_not_calibration(self):
        s,t=equilibrium(.2,PARAMETERS)
        for m in [.75,1.5,3]:
            for inertia in [5e-5,1e-4,2e-4]:
                p=replace(PARAMETERS,cassette_mass_kg=m,output_inertia_kg_m2=inertia)
                r=ports(s,p,0)
                self.assertAlmostEqual(r['angular_acceleration']*inertia,-t,places=14)
                self.assertGreater(step(s,p,1e-5,0).thetadot,0)
    def test_receipt_and_source_pins(self):
        saved=json.loads((ROOT/'cad/assembly/evidence/cassette-cam-dynamics.json').read_text())
        self.assertEqual(saved,run())
        for path,sha in saved['inputs'].items():self.assertEqual(hashlib.sha256((ROOT/path).read_bytes()).hexdigest(),sha)

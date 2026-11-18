import pathlib,sys,unittest,copy,tempfile
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/pressure'))
import run_fresh_rest_controller as m
class Fresh(unittest.TestCase):
 def test_zero_energy_and_unchanged_control(self):
  p,c,pins,bus,a=m.load();self.assertEqual(p.t,0);self.assertEqual(p.note['v'],[0.,0.]);self.assertEqual(p.note['w'],[0.,0.]);self.assertEqual(p.drive.kinetic(),0);self.assertFalse(any(p.energy.values()));self.assertEqual(c.goal,.185);self.assertEqual(c.max_speed,.25);self.assertEqual(a['nextControllerTimeS'],0.)
  before=m.export_domain(p);o=m.prior.observe(0,p.drive.state,p.note,bus);expected=m.prior.SpeedApproach(p.drive.c,goal=.185,max_speed=.25).update(0,o)
  self.assertEqual(c.update(0,o),expected);self.assertEqual(m.export_domain(p),before)
  self.assertGreater(a['baselineFirstContact']['chassisVxMps'],.8)
 def test_two_ms_fresh_smoke(self):
  with tempfile.TemporaryDirectory() as d:
   r=m.run(pathlib.Path(d)/'out.json',duration=.002)
  self.assertEqual(r['outcome'],'HORIZON');self.assertEqual(len(r['trace']),2);self.assertEqual(r['events'][0]['timeS'],0.);self.assertIsNone(r['firstContact']);self.assertEqual(r['startCheckpoint']['state']['initialEnergyJ'],0.)
 def test_reject_other_case(self):
  with self.assertRaises(ValueError):m.load('retuned')
if __name__=='__main__':unittest.main()

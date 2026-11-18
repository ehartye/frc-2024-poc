import pathlib,sys,unittest,copy,json
from dataclasses import replace
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/pressure'))
import compare_entry as entry
import constrained_cassettes as c
class Bridge(unittest.TestCase):
 @classmethod
 def setUpClass(cls):cls.original,_=entry.setup()
 def test_exact_three_step_prefix(self):
  a=copy.deepcopy(self.original);b=c.FixedCassettes(a);loads=[a.drive.c['massKg']*9.80665/2]*2
  for _ in range(3):
   expected=a.step(.001,[.2,.2],[.12,.12],loads);actual=b.step(.001,[.2,.2],[.12,.12],loads)
   self.assertEqual(actual['acquisition'],expected);self.assertEqual(actual['energy']['residual'],expected['energyJ']['residual'])
   self.assertGreater(actual['energy']['constantComponentPotentialJ'],0)
  self.assertEqual(a.note,b.plant.note);self.assertEqual(a.drive.state,b.plant.drive.state);self.assertEqual(a.energy,b.plant.energy)
 def test_same_time_contact_query(self):
  b=c.FixedCassettes(self.original);f=json.loads((R/'sim/pressure/acquisition-entry-evidence.json').read_text())['final']
  before=copy.deepcopy((b.plant.note,b.plant.drive.state,b.plant.energy,b.plant.t))
  expected=self.original.acquisition_rates(f['note'],f['chassis'],f['busV'],f['commands']['pinch'],{})
  actual=b.plant.acquisition_rates(f['note'],f['chassis'],f['busV'],f['commands']['pinch'],{})
  self.assertEqual(expected,actual)
  sides=b.constraint_ports(f['chassis'],f['note']);self.assertTrue(any(abs(s['pressureGeneralizedForceN'])>0 for s in sides))
  self.assertTrue(all(s['relativeConstraintWorkW']==0 for s in sides))
  self.assertEqual(before,(b.plant.note,b.plant.drive.state,b.plant.energy,b.plant.t))
 def test_reject_nonfresh_and_release(self):
  p=copy.deepcopy(self.original);p.t=.001
  with self.assertRaises(ValueError):c.FixedCassettes(p)
  b=c.FixedCassettes(self.original);before=copy.deepcopy((b.plant.note,b.plant.drive.state,b.plant.energy,b.plant.t))
  with self.assertRaisesRegex(ValueError,'mass partition'):b.release()
  self.assertEqual(before,(b.plant.note,b.plant.drive.state,b.plant.energy,b.plant.t))
 def test_bad_hypotheses(self):
  for side in [c.LockedSide(theta=True),c.LockedSide(spring_free_length=-1),c.LockedSide(cam_stiffness=float('inf'))]:
   with self.assertRaises(ValueError):c.FixedCassettes(self.original,side)
 def test_failed_step_atomic(self):
  b=c.FixedCassettes(self.original);before=copy.deepcopy((b.plant.note,b.plant.drive.state,b.plant.energy,b.plant.t))
  with self.assertRaises(ValueError):b.step(.001,[.2,.2],[2.,.12],[220.,220.])
  self.assertEqual(before,(b.plant.note,b.plant.drive.state,b.plant.energy,b.plant.t))
if __name__=='__main__':unittest.main()

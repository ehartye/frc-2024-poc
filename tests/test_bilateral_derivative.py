import pathlib,sys,unittest,json,copy
import numpy as np
R=pathlib.Path(__file__).resolve().parents[1];sys.path[:0]=[str(R/'sim/pressure'),str(R/'cad/assembly')]
import compare_entry as entry
import bilateral_derivative as c
class Derivative(unittest.TestCase):
 @classmethod
 def setUpClass(cls):cls.plant,_=entry.setup();cls.final=json.loads((R/'sim/pressure/acquisition-entry-evidence.json').read_text())['final']
 def test_same_time_zero_u_note_rotor_friction_exact(self):
  p=self.plant;f=self.final;m=c.Mechanism(tp=.2,tn=.2)
  before=copy.deepcopy((p.drive.state,p.note,p.energy,p.t));expected=p.acquisition_rates(f['note'],f['chassis'],f['busV'],f['commands']['pinch'],{})
  actual=c.acquisition_rates(p,f['note'],f['chassis'],f['busV'],f['commands']['pinch'],m,{})
  self.assertEqual(len(actual.pop('cassetteContact')),2);self.assertEqual(actual,expected);self.assertEqual(before,(p.drive.state,p.note,p.energy,p.t))
 def test_locked_chassis_rates_and_new_energy_explicit(self):
  p=self.plant;f=self.final;loads=[p.drive.c['massKg']*9.80665/2]*2
  r=c.derivative(p,f['chassis'],f['note'],c.Mechanism(tp=.2,tn=.2),f['busV'],[.2,.2],[.12,.12],loads,locked=(3,4,5,6))
  for key in ['dvx','dvy','dyaw']:self.assertAlmostEqual(r['driveRates'][key],r['originalDriveRates'][key],places=12)
  for key in r['originalDriveRates']:
   if key not in ['dvx','dvy','dyaw']:self.assertEqual(r['driveRates'][key],r['originalDriveRates'][key])
  self.assertGreater(r['componentPotentialJ'],0);self.assertEqual(r['inertia']['constraintPowerW'],0)
  oldkin=p.drive.kinetic(f['chassis'])+p.note_kinetic(f['note']);self.assertAlmostEqual(r['kineticJ'],oldkin,places=12)
 def test_free_fresh_derivative_not_ideal_attachment(self):
  p=self.plant;loads=[p.drive.c['massKg']*9.80665/2]*2;r=c.derivative(p,p.drive.state,p.note,c.Mechanism(tp=.2,tn=.2),12.,[.2,.2],[.12,.12],loads)
  self.assertGreater(r['mechanismAcceleration'][0],0);self.assertGreater(r['mechanismAcceleration'][1],0)
  self.assertEqual(r['noteRates']['contacts'],[])
 def test_reject_moving_impact_and_mass_mismatch(self):
  p=self.plant;before=copy.deepcopy((p.drive.state,p.note,p.energy,p.t))
  with self.assertRaises(ValueError):c.derivative(p,p.drive.state,p.note,c.Mechanism(udp=-.1),12.,[.2,.2],[.12,.12],[220,220])
  self.assertEqual(before,(p.drive.state,p.note,p.energy,p.t))
if __name__=='__main__':unittest.main()

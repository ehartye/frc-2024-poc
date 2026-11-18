"""Validate recorded experiment lineage and bounded numerical checks, not hardware."""
import unittest,json,pathlib,hashlib,copy
R=pathlib.Path(__file__).resolve().parents[1]
EXPECTED={'sim/drivetrain/plant.py','sim/drivetrain/config.json','sim/drivetrain/run.py','sim/integrated/shared_supply.py','sim/component-launcher/power_state.py','cad/drivetrain/candidate.json'}

def validate(d):
 if set(d['inputs'])!=EXPECTED:raise ValueError('exact source inventory')
 for name,pin in d['inputs'].items():
  if hashlib.sha256((R/name).read_bytes()).hexdigest()!=pin:raise ValueError('changed source')
 names=['drive-only','shared-load','spin','coast','brake','brownout','external-reaction','mass30','mass60','center-wheels-only','shared-load-half']
 if [c['name'] for c in d['cases']]!=names:raise ValueError('case inventory')
 return {c['name']:c for c in d['cases']}

class EvidenceTest(unittest.TestCase):
 @classmethod
 def setUpClass(cls):cls.d=json.loads((R/'sim/drivetrain/evidence.json').read_text())
 def test_exact_sources_and_negative_controls(self):
  validate(self.d)
  for kind in ['missing','changed','extra']:
   d=copy.deepcopy(self.d)
   if kind=='missing':del d['inputs']['sim/drivetrain/plant.py']
   elif kind=='changed':d['inputs']['sim/drivetrain/plant.py']='0'*64
   else:d['inputs']['extra.py']='0'*64
   with self.assertRaises(ValueError):validate(d)
 def test_matrix_and_energy(self):
  for c in validate(self.d).values():
   self.assertEqual(c['status'],'HORIZON');self.assertFalse(c['acceptance']);self.assertAlmostEqual(c['final']['state']['t'],.3)
   self.assertLess(c['maxAbsGlobalResidualJ'],.004)
   for row in c['trace']:
    self.assertEqual(len(row['electrical']['perMotor']),4)
    self.assertLess(abs(row['globalEnergyJ']['residual']),.004)
 def test_half_step_and_response(self):
  c=validate(self.d);a=c['shared-load'];b=c['shared-load-half']
  self.assertLess(abs(a['final']['state']['x']-b['final']['state']['x']),3e-6)
  self.assertLess(b['maxAbsGlobalResidualJ'],a['maxAbsGlobalResidualJ'])
  self.assertLess(a['minimumBusV'],c['drive-only']['minimumBusV'])
  self.assertLess(c['brake']['final']['state']['vx'],c['coast']['final']['state']['vx'])
  self.assertLess(c['mass60']['final']['state']['vx'],c['mass30']['final']['state']['vx'])
 def test_disable_retains_motion_and_external_work_separate(self):
  c=validate(self.d);b=c['brownout']['final']
  self.assertFalse(b['controller']['outputsEnabled']);self.assertEqual(b['modes'],['coast','coast'])
  self.assertGreater(b['state']['vx'],0);self.assertGreater(b['state']['wheel'][0],0)
  self.assertLess(c['external-reaction']['final']['energyJ']['externalWork'],0)
if __name__=='__main__':unittest.main()

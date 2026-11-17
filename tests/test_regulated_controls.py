import pathlib,sys,json,hashlib,unittest
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/integrated'))
from continuation_state import restore
class Regulated(unittest.TestCase):
 def load(self,name):
  r=json.loads((R/('sim/integrated/evidence/'+name+'.json')).read_text())
  for p,h in r['inputs'].items():self.assertEqual(hashlib.sha256((R/p).read_bytes()).hexdigest(),h,p)
  return r
 def test_half_step_and_no_pinch_do_not_establish_capture(self):
  a=self.load('speed-approach-nominal');b=self.load('speed-approach-half');z=self.load('speed-approach-no-pinch')
  self.assertEqual(a['outcome'],b['outcome']);self.assertLess(abs(a['final']['relativeRobotXY'][0]-b['final']['relativeRobotXY'][0]),1e-5)
  self.assertLess(b['maxAbsEnergyResidualJ'],a['maxAbsEnergyResidualJ'])
  self.assertEqual(z['outcome'],'GROUND_DWELL_DIAGNOSTIC_ONLY');self.assertFalse(z['acceptance'])
  self.assertTrue(all(x['controller']['pinchDemands']==[0.,0.] for x in z['trace']));self.assertGreater(min(z['final']['note']['w']),1.)
 def test_stale_braking_preserves_motion_and_no_acquisition(self):
  r=self.load('speed-approach-stale');self.assertEqual(r['outcome'],'HORIZON');self.assertEqual(r['events'][0]['toState'],'OBSERVATION_FAULT')
  self.assertGreater(r['events'][0]['observation']['forwardSpeed'],.2)
  self.assertGreater(r['events'][0]['observation']['relativeX']-r['final']['relativeRobotXY'][0],.05)
  self.assertGreater(r['final']['relativeRobotXY'][0],.49);self.assertGreater(min(r['final']['note']['w']),10.)
 def test_fast_nominal_is_exact_trace(self):
  a=self.load('speed-approach-nominal');b=self.load('fast-regulated-nominal')
  for k in ['trace','final','events','outcome','maxAbsEnergyResidualJ']:self.assertEqual(a[k],b[k])
 def test_index_half_step_preserves_outcome_and_reduces_residual(self):
  a=self.load('fast-regulated-index');b=self.load('fast-regulated-index-half')
  self.assertEqual(a['outcome'],b['outcome']);self.assertLess(abs(a['final']['relativeRobotXY'][0]-b['final']['relativeRobotXY'][0]),1e-5)
  self.assertLess(b['maxAbsEnergyResidualJ'],a['maxAbsEnergyResidualJ'])
 def test_deeper_index_keeps_full_checkpoint_not_lift_acceptance(self):
  r=self.load('fast-regulated-index');s=r['checkpoint'];self.assertEqual(s['drive'],r['final']['chassis']);self.assertEqual(s['note'],r['final']['note'])
  self.assertEqual(s['modelIdentity'],r['inputs']);self.assertFalse(r['acceptance']);self.assertLess(r['final']['relativeRobotXY'][0],.19)
  self.assertGreater(abs(r['final']['note']['v'][0]),0);self.assertGreater(min(r['final']['note']['w']),0)
  p=restore(s,lambda q:dict(status='SUPPORTED',contacts=[]),r['inputs'],s['driveConfig'],s['noteConfig'])
  self.assertEqual(p.t,r['final']['timeS']);self.assertEqual(p.energy,{k:r['final']['energyJ'][k] for k in p.energy})
if __name__=='__main__':unittest.main()

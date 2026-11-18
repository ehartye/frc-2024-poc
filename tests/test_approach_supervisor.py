import pathlib,sys,json,hashlib,unittest
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/integrated'))
from approach_supervisor import ApproachSupervisor
class Approach(unittest.TestCase):
 def test_fresh_threshold_and_latched_fault(self):
  c=ApproachSupervisor(.34);self.assertEqual(c.update(0,dict(timestamp=0,relativeX=.4,relativeVx=-.4))['state'],'APPROACH')
  self.assertEqual(c.update(.01,dict(timestamp=.01,relativeX=.33,relativeVx=-.4))['modes'],['brake','brake'])
  self.assertEqual(c.update(.2,dict(timestamp=.01,relativeX=.33,relativeVx=-.4))['state'],'OBSERVATION_FAULT')
  self.assertEqual(c.update(.3,dict(timestamp=.3,relativeX=.4,relativeVx=0))['state'],'OBSERVATION_FAULT')
 def test_invalid_observation_commands_braking_not_reset(self):
  c=ApproachSupervisor();r=c.update(1,dict(timestamp=1,relativeX=float('nan'),relativeVx=0));self.assertEqual(r['state'],'OBSERVATION_FAULT');self.assertEqual(r['pinchDemands'],[0.,0.])
 def test_bound_negative_cases(self):
  for name in ['late-brake','early-brake','stale']:
   r=json.loads((R/('sim/integrated/evidence/approach-'+name+'.json')).read_text())
   for p,h in r['inputs'].items():self.assertEqual(hashlib.sha256((R/p).read_bytes()).hexdigest(),h,p)
   self.assertTrue(r['outcome'].startswith('UNKNOWN_DOMAIN'));self.assertTrue(r['events']);self.assertFalse(r['acceptance']);self.assertGreater(r['final']['chassis']['vx'],.1)
   self.assertEqual(r['final']['controller']['pinchDemands'],[0.,0.]);self.assertGreater(max(r['final']['note']['w']),1.)
if __name__=='__main__':unittest.main()

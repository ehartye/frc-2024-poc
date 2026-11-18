import copy,json,pathlib,sys,tempfile,unittest
from unittest.mock import patch
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/pressure'))
import run_early_continuation as r
class ContinuePolicy(unittest.TestCase):
 def test_exact_endpoint_and_fault_latch(self):
  for name in r.CASES:
   p,controller,pins,bus,_=r.load(name)
   self.assertAlmostEqual(p.t,.662,places=12);self.assertGreater(p.drive.state['vx'],.5)
   self.assertEqual(controller.fault,'stale-or-invalid-observation' if name=='stale' else None)
   self.assertTrue(all(r.c.factory.r.sha(R/f)==h for f,h in pins.items()))
   before=r.c.export(p);controller.update(p.t,r.prior.observe(p.t,p.drive.state,p.note,bus));self.assertEqual(before,r.c.export(p))
   if name=='stale':self.assertEqual(controller.state,'OBSERVATION_FAULT')
 def test_command_history_mutation_rejected(self):
  report=json.loads((R/'sim/pressure/early-policy-evidence/nominal-1ms.json').read_text());p,_,_,_,_=r.load()
  report['events'][3]['command']['desiredSpeed']+=.1
  with self.assertRaises(ValueError):r.restore_controller(p.drive.c,report)
 def test_atomic_unknown_and_rejected_query(self):
  with tempfile.TemporaryDirectory() as d:
   def fail(p,*a,**k):
    # A separate failed query is recorded before state verification re-queries accepted geometry.
    p.geometry([.177,0])
    raise r.prior.DomainExit('test-domain-rejection')
   with patch.object(r.c.factory.r.PressureAcquisition,'step',fail):out=r.run(pathlib.Path(d)/'r.json',duration=.002)
   self.assertEqual(out['startCheckpoint'],out['endCheckpoint']);self.assertEqual(out['trace'],[])
   self.assertEqual(out['rejectedGeometryQuery']['relativeRobotXY'],[.177,0]);self.assertEqual(out['outcome'],'UNKNOWN_DOMAIN')
if __name__=='__main__':unittest.main()

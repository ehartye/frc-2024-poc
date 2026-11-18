import unittest,pathlib,sys,json,tempfile
from unittest.mock import patch
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/pressure'))
import run_early_policy as r
class EarlyPolicy(unittest.TestCase):
 def test_source_row_and_no_reset(self):
  p,pins,bus,ancestry=r.load();self.assertAlmostEqual(p.t,.562,places=12);self.assertEqual(ancestry['rowIndex'],561);self.assertGreater(ancestry['distanceToDomainLowerBoundM'],.16);self.assertGreater(p.drive.state['vx'],.8);self.assertGreater(p.note['w'][0],70);self.assertGreater(p.energy['sourceWork'],0);before=r.c.export(p);controller=r.SpeedApproach(p.drive.c,goal=.185);cmd=controller.update(p.t,r.observe(p.t,p.drive.state,p.note,bus));self.assertEqual(before,r.c.export(p));self.assertEqual(cmd['modes'],['brake','brake']);self.assertTrue(all(r.c.factory.r.sha(R/f)==h for f,h in pins.items()))
 def test_atomic_rejection_is_saved(self):
  with tempfile.TemporaryDirectory() as d:
   output=pathlib.Path(d)/'r.json'
   with patch.object(r.c.factory.r.PressureAcquisition,'step',side_effect=r.DomainExit('test-domain')):r.run(output)
   report=json.loads(output.read_text());self.assertEqual(report['startCheckpoint'],report['endCheckpoint']);self.assertEqual(report['reason'],'test-domain');self.assertEqual(report['trace'],[])
if __name__=='__main__':unittest.main()

import copy,json,pathlib,sys,tempfile,unittest
from unittest.mock import patch
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/pressure'))
import run_braking_boundary as r
class Boundary(unittest.TestCase):
 def test_exact_continuity(self):
  p,ctl,pins,bus,a=r.load();self.assertAlmostEqual(p.t,.862,places=12);self.assertGreater(p.drive.state['vx'],.2)
  self.assertTrue(all(r.c.factory.r.sha(R/f)==h for f,h in pins.items()));b=r.c.export(p)
  cmd=ctl.update(p.t,r.prior.observe(p.t,p.drive.state,p.note,bus));self.assertEqual(cmd['modes'],['brake','brake']);self.assertEqual(b,r.c.export(p))
 def test_history_mutation(self):
  d=json.loads((R/r.SOURCE).read_text());d['events'][3]['command']['desiredSpeed']+=.1
  with self.assertRaises(ValueError):r.replay(None,d)
 def test_atomic_rejected_trial(self):
  def fail(p,*a,**kw):p.geometry([.177,0]);raise r.prior.DomainExit('bounded-domain-test')
  with tempfile.TemporaryDirectory() as d,patch.object(r.c.factory.r.PressureAcquisition,'step',fail):o=r.run(pathlib.Path(d)/'out.json',duration=.002)
  self.assertEqual(o['startCheckpoint'],o['endCheckpoint']);self.assertEqual(o['trace'],[]);self.assertEqual(o['rejectedGeometryQuery']['relativeRobotXY'],[.177,0])
if __name__=='__main__':unittest.main()

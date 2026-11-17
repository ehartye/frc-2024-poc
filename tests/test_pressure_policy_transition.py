import unittest,pathlib,sys,copy
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/pressure'))
import run_policy_transition as r
class PolicyTest(unittest.TestCase):
 def test_engagement_preserves_physical_state(self):
  p,pins=r.load();before=r.c.export(p);ctl=r.engage(p,'nominal');cmd,obs=r.command(ctl,p,11.96,'nominal');self.assertEqual(before,r.c.export(p));self.assertEqual(cmd['modes'],['brake','brake']);self.assertEqual(cmd['driveDemands'],[0.,0.]);self.assertGreater(p.drive.state['vx'],.8);self.assertGreater(p.note['w'][0],70);self.assertTrue(all(r.c.factory.r.sha(R/f)==h for f,h in pins.items()))
 def test_fault_and_no_pinch_are_commands_not_resets(self):
  for scenario in ['stale','no-pinch']:
   p,_=r.load();before=r.c.export(p);ctl=r.engage(p,scenario);cmd,_=r.command(ctl,p,12.,scenario);self.assertEqual(before,r.c.export(p));self.assertEqual(cmd['pinchDemands'],[0.,0.])
   if scenario=='stale':self.assertEqual(cmd['state'],'OBSERVATION_FAULT')
 def test_first_finite_brake_step(self):
  p,_=r.load();ctl=r.engage(p,'nominal');cmd,_=r.command(ctl,p,12.,'nominal');v=p.drive.state['vx'];t=p.t;heat=p.energy['driveWinding']+p.energy['driveBrake']
  row=p.step(.001,cmd['driveDemands'],cmd['pinchDemands'],[p.drive.c['massKg']*9.80665/2]*2,modes=cmd['modes']);self.assertGreater(row['chassis']['vx'],0);self.assertLess(row['chassis']['vx'],v);self.assertAlmostEqual(p.t,t+.001);self.assertGreater(row['energyJ']['driveWinding']+row['energyJ']['driveBrake'],heat)
if __name__=='__main__':unittest.main()

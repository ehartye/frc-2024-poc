import pathlib,sys,copy,unittest,json
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/integrated'))
from coupled_acquisition import CoupledAcquisition
from continuation_state import checkpoint,restore
D=json.loads((R/'sim/drivetrain/config.json').read_text());C=json.loads((R/'sim/acquisition/free-planar-config.json').read_text())['model'];ID={'fixture':'a'*64}
def geometry(q):return dict(status='SUPPORTED',contacts=[])
class Continuity(unittest.TestCase):
 def plant(self):
  p=CoupledAcquisition(D,dict(C,groundMu=0),geometry,dict(p=[.3,.001],v=[.02,.01],yaw=.01,spin=.02,w=[2.,3.]));p.drive.state.update(vx=.1,vy=.01,yaw_rate=.02)
  p.step(.0001,[.1,.11],[.03,.04],[220.,220.]);return p
 def test_resume_matches_uninterrupted_next_step_exactly(self):
  p=self.plant();s=checkpoint(p,ID);q=restore(s,geometry,ID,D,dict(C,groundMu=0))
  self.assertEqual(p.drive.state,q.drive.state);self.assertEqual(p.note,q.note);self.assertEqual(p.energy,q.energy)
  for _ in range(3):self.assertEqual(p.step(.0001,[0.,0.],[0.,0.],[220.,220.],modes=['brake','brake']),q.step(.0001,[0.,0.],[0.,0.],[220.,220.],modes=['brake','brake']))
  self.assertNotEqual(q.note['v'],[0.,0.]);self.assertNotEqual(q.note['w'],[0.,0.])
 def test_reject_incomplete_changed_or_nonfinite_state(self):
  s=checkpoint(self.plant(),ID)
  for mutate in [lambda x:x['note'].pop('w'),lambda x:x['driveConfig'].update(massKg=100),lambda x:x['energy'].pop('groundHeat'),lambda x:x['drive'].update(t=-1),lambda x:x['note'].update(spin=float('nan'))]:
   bad=copy.deepcopy(s);mutate(bad)
   with self.assertRaises(ValueError):restore(bad,geometry,ID,D,dict(C,groundMu=0))
  with self.assertRaises(ValueError):restore(s,geometry,{'fixture':'b'*64},D,dict(C,groundMu=0))
 def test_checkpoint_is_not_live_alias(self):
  p=self.plant();s=checkpoint(p,ID);s['note']['p'][0]=999;self.assertNotEqual(p.note['p'][0],999)
if __name__=='__main__':unittest.main()

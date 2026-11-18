import copy,pathlib,sys,unittest
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/integrated'))
import coupled_lift_sweep as s
class Dummy:
 def commands(self,name):return {'drive':[.1,.1],'pinch':[.2,.2],'actuator':[.04,.05],'actuatorModes':['drive','drive']}
class SweepTest(unittest.TestCase):
 def test_exact_plan(self):
  self.assertEqual(len(s.PLAN),7);self.assertEqual(len({r[0] for r in s.PLAN}),7)
  p=Dummy()
  for row in s.PLAN:
   c=s.command(p,row);self.assertEqual(c['drive'],[.1,.1]);self.assertEqual(c['pinch'],[.2,.2]);self.assertEqual(c['actuator'],[.04+row[1],.05+row[2]]);self.assertEqual(c['actuatorModes'],[row[3]]*2)
 def test_invalid_offsets(self):
  for x in (True,float('nan'),float('inf'),.031):
   with self.assertRaises(ValueError):s.command(Dummy(),('invalid',x,0.,'drive'))
 def test_bad_mode(self):
  with self.assertRaises(ValueError):s.command(Dummy(),('invalid',0.,0.,'hold'))
 def test_no_pose_controller_or_kernel_edits(self):
  self.assertEqual(s.sha(R/'sim/integrated/coupled_lift_phase.py'),__import__('json').loads((R/'sim/integrated/coupled_lift_evidence.json').read_text())['inputs']['sim/integrated/coupled_lift_phase.py'])
if __name__=='__main__':unittest.main()

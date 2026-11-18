import pathlib,sys,unittest,json,copy
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/pressure'))
import run_inward_extended as m
class Restore(unittest.TestCase):
 def test_restore_and_clock(self):
  p,c,pins,bus,a=m.load();r=json.loads((R/m.SOURCE).read_text())
  self.assertEqual(m.export_domain(p),r['endCheckpoint']);self.assertAlmostEqual(a['nextControllerTimeS'],.962)
  self.assertFalse(m.controller_due(.960,a['nextControllerTimeS']))
 def test_reject_identity_and_shapes(self):
  r=json.loads((R/m.SOURCE).read_text())['endCheckpoint']
  for key,value in [('schema','pressure-acquisition-checkpoint/1'),('digest','bad')]:
   q=copy.deepcopy(r);q[key]=value
   with self.assertRaises(ValueError):m.restore_domain(q)
  q=copy.deepcopy(r);q['state']['note']['v']=[[],0];q['digest']=m.c.digest({k:v for k,v in q.items() if k!='digest'})
  with self.assertRaises(ValueError):m.restore_domain(q)
if __name__=='__main__':unittest.main()

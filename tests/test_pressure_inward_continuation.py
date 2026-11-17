import pathlib,sys,unittest,json
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/pressure'))
import run_inward_continuation as m
class Transition(unittest.TestCase):
 def test_state_and_domain_identity(self):
  p,c,pins,bus,a=m.load();old=json.loads((R/m.SOURCE).read_text())['endCheckpoint'];new=m.export_domain(p)
  self.assertEqual(old['state'],new['state']);self.assertEqual(old['diagnostics'],new['diagnostics'])
  self.assertNotEqual(old['schema'],new['schema']);self.assertNotEqual(old['digest'],new['digest'])
  self.assertEqual(p.geometry([.168,0])['status'],'SUPPORTED')
  self.assertEqual(p.geometry([.137,0])['status'],'OUTSIDE_BOUNDED_PLANAR_EXPERIMENT')
  with self.assertRaises(ValueError):m.c.restore(new)
if __name__=='__main__':unittest.main()

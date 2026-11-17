import pathlib,sys,unittest,json
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/pressure'))
import run_inward_tail as m
class Restore(unittest.TestCase):
 def test_restored_state_and_clock(self):
  p,c,pins,bus,a=m.load();r=json.loads((R/m.SOURCE).read_text())
  self.assertEqual(m.export_domain(p),r['endCheckpoint']);self.assertAlmostEqual(a['nextControllerTimeS'],1.262)
  self.assertFalse(m.controller_due(1.260,a['nextControllerTimeS']));self.assertTrue(m.controller_due(1.262,a['nextControllerTimeS']))
if __name__=='__main__':unittest.main()

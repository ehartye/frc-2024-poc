import pathlib,sys,unittest
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/integrated'))
from retention_entry_bound import static_bound
class Bound(unittest.TestCase):
 def test_current_preload_is_not_old_retention_budget(self):
  r=static_bound([1.7552133272048422]*2,.6,.235)
  self.assertFalse(r['necessaryStaticConditionMet']);self.assertAlmostEqual(r['marginN'],-.1983067573541893)
 def test_positive_sum_is_not_acceptance(self):
  r=static_bound([10.,10.],.6,.235);self.assertTrue(r['necessaryStaticConditionMet']);self.assertFalse(r['retentionAccepted'])
 def test_invalid_and_missing_side(self):
  for args in [([1.],.6,.235),([True,1.],.6,.235),([1.,1.],float('nan'),.235),([1.,1.],.6,0.)]:
   with self.assertRaises(ValueError):static_bound(*args)
if __name__=='__main__':unittest.main()

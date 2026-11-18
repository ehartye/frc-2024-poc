import unittest,sys,pathlib,math
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'cad/assembly'))
from opening_support_compliance import screen
class Compliance(unittest.TestCase):
 def test_centerline_datums(self):
  a=screen(1,revision='slender');b=screen(1,revision='wide')
  self.assertEqual((a['armCentroidZmm'],a['verticalCenterlineLengthMm'],a['loadBelowArmCentroidMm']),(200.5,84.5,15.5))
  self.assertEqual((b['armCentroidZmm'],b['verticalCenterlineLengthMm'],b['loadBelowArmCentroidMm']),(203,87,18))
  self.assertEqual(b['attachmentFaceCentroidXmm'],117.6)
 def test_zero_load(self):self.assertEqual(screen(0)['totalEstimatedMm'],0)
 def test_linear_load(self):
  self.assertAlmostEqual(screen(20)['totalEstimatedMm'],2*screen(10)['totalEstimatedMm']);self.assertAlmostEqual(screen(20)['elasticEnergyJ'],4*screen(10)['elasticEnergyJ'])
 def test_modulus_scaling(self):self.assertAlmostEqual(screen(10,138000,52000)['totalEstimatedMm'],.5*screen(10)['totalEstimatedMm'])
 def test_reject_bad_inputs(self):
  for x in [True,float('nan'),float('inf'),-1]:
   with self.assertRaises(ValueError):screen(x)
  with self.assertRaises(ValueError):screen(1,0)
  with self.assertRaises(ValueError):screen(1e308)
 def test_torsion_not_omitted(self):
  a=screen(14.6234,revision='slender');b=screen(14.6234,revision='wide')
  self.assertGreater(a['stemTorsionMm'],a['stemBendingMm']);self.assertGreater(a['totalEstimatedMm'],.15);self.assertLess(b['totalEstimatedMm'],.15)
  self.assertAlmostEqual(a['totalEstimatedMm'],sum(a[k] for k in ['stemBendingMm','stemTorsionMm','armBendingMm']))
if __name__=='__main__':unittest.main()

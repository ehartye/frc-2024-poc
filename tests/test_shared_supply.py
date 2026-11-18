import pathlib,sys,unittest,math
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'sim/integrated'))
from shared_supply import solve_supply,SupplyDomainError
class Supply(unittest.TestCase):
 def test_resistive_parallel_ports(self):
  r=solve_supply({'a':lambda v:{'busPowerW':v*v/2},'b':lambda v:{'busPowerW':v*v/3}},12,.02)
  self.assertAlmostEqual(r['busV'],12/(1+.02*(1/2+1/3)),places=9);self.assertAlmostEqual(r['powerResidualW'],0,places=7)
 def test_added_load_sags_same_bus(self):
  a=solve_supply({'drive':lambda v:{'busPowerW':v*10}},12,.02)
  b=solve_supply({'drive':lambda v:{'busPowerW':v*10},'launcher':lambda v:{'busPowerW':v*20}},12,.02)
  self.assertAlmostEqual(a['busV'],11.8);self.assertAlmostEqual(b['busV'],11.4)
 def test_constant_power_root_and_collapse(self):
  r=solve_supply({'load':lambda v:{'busPowerW':100}},12,.1)
  self.assertEqual(len(r['allEquilibriumCandidatesV']),2);self.assertAlmostEqual(r['busV'],(12+math.sqrt(144-40))/2,places=9)
  with self.assertRaises(SupplyDomainError):solve_supply({'load':lambda v:{'busPowerW':400}},12,.1)
 def test_regeneration_explicit(self):
  port={'brake':lambda v:{'busPowerW':-v*5}}
  with self.assertRaises(SupplyDomainError):solve_supply(port,12,.02)
  r=solve_supply(port,12,.02,allow_battery_regeneration=True);self.assertAlmostEqual(r['busV'],12.1);self.assertLess(r['sourcePowerW'],0);self.assertAlmostEqual(r['powerResidualW'],0,places=7)
 def test_internal_power_exchange(self):
  r=solve_supply({'brake':lambda v:{'busPowerW':-v*5},'drive':lambda v:{'busPowerW':v*10}},12,.02)
  self.assertAlmostEqual(r['busCurrentA'],5);self.assertFalse(r['regenerating'])
 def test_invalid_and_open_circuit(self):
  for value in [True,float('nan'),float('inf'),-1]:
   with self.assertRaises(ValueError):solve_supply({},value,.02)
  r=solve_supply({},12,0);self.assertEqual(r['busV'],12);self.assertEqual(r['busCurrentA'],0)
if __name__=='__main__':unittest.main()

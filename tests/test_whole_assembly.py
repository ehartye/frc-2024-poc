import unittest,json,pathlib,sys
ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'cad/assembly'))
import build_robot as b
class AssemblyTest(unittest.TestCase):
 @classmethod
 def setUpClass(cls):cls.parts,cls.dc=b.build()
 def test_real_chassis(self):
  self.assertAlmostEqual(self.dc['centerTrackMm'],579.60768)
  self.assertEqual(self.dc['frameNativeEnvelopeMm'][:2],[719.074,711.2])
 def test_no_legacy_chassis_or_battery(self):
  self.assertFalse(any(n.startswith(('frame longitudinal','wheel (','batteryReserve','electronicsReserve','original-crossmember')) for n,s,k in self.parts))
 def test_corrected_tread(self):
  names=[n for n,s,k in self.parts]
  self.assertEqual(sum('nominal tread annulus' in n for n in names),4)
  tx=json.loads((ROOT/'cad/components/a2-transmission.json').read_text());t=tx['serviceInterfaces']['tread'];self.assertAlmostEqual(t['nativeSeatRadiusMm']+t['thicknessMm'],48.26)
 def test_valid_original_solids(self):
  for n,s,k in self.parts:self.assertTrue(b.h.BRepCheck_Analyzer(s).IsValid(),n)
 def test_intake_not_misrepresented(self):
  self.assertTrue(any(n.startswith('UNRESOLVED intake') for n,s,k in self.parts))
  self.assertTrue(all(k=='service' for n,s,k in self.parts if n.startswith('UNRESOLVED')))
 def test_partial_mass_is_not_whole_cog(self):
  q=b.mass_ledger(self.parts,self.dc);self.assertIsNone(q['wholeRobotCoG']);self.assertGreater(q['estimatedOriginalHardware']['subtotalKg'],0);self.assertAlmostEqual(q['sourceKnown']['subtotalKg'],13.08549774,places=6)
if __name__=='__main__':unittest.main()

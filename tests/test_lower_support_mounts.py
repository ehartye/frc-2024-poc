import unittest,sys,pathlib,json,hashlib
ROOT=pathlib.Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'cad'),str(ROOT/'cad/assembly')]
import lower_support_mounts as m
class LowerSupportTest(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.base=m.assembly.build('floor');cls.parts,cls.joints=m.apply(cls.base)
 def test_registered_brackets_and_selected_stacks(self):
  self.assertEqual(len(self.parts)-len(self.base),52);self.assertEqual(len(self.joints),4)
  for n,s,k in self.parts[len(self.base):]:self.assertTrue(m.h.BRepCheck_Analyzer(s).IsValid(),n)
 def test_preserves_nominal_clearance_and_material_stack(self):
  self.assertTrue(all(j['materialStackMm']==6.35 for j in self.joints))
  self.assertEqual({j['bridge'] for j in self.joints},{'frame fastening bridge-0','frame fastening bridge-1'})
 def test_saved_evidence_bound_and_controls(self):
  r=json.loads((ROOT/'cad/assembly/evidence/lower-support-mounts.json').read_text())
  self.assertEqual(r['assemblyInputs'],m.assembly.input_pins())
  self.assertEqual(r['scriptSha256'],hashlib.sha256(pathlib.Path(m.__file__).read_bytes()).hexdigest())
  self.assertEqual(r['rawIntersections'],[]);self.assertGreater(r['undrilledPostNegativeMm3'],50)
  self.assertEqual(len(r['movingHardwareSamples']),15);self.assertFalse(any(x['hits'] for x in r['movingHardwareSamples']))
if __name__=='__main__':unittest.main()

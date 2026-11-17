import json,pathlib,sys,unittest
ROOT=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'cad/components'))
import gearbox_output_coupling as q
class CouplingStudyTest(unittest.TestCase):
 def test_source_bound_receipt(self):
  r=json.loads((ROOT/'cad/components/gearbox-output-coupling.json').read_text())
  self.assertEqual(r['inputs'],q.input_pins());self.assertTrue(r['inputsStable'])
  self.assertEqual(r['nativeSolidCount'],35);self.assertEqual(r['nativeLooseFaceCount'],0)
  self.assertFalse(any(x['nativeIntersections'] for x in r['checks']))
  self.assertGreater(r['wrongClock30DegreeNativeOverlapMm3'],70)
  self.assertGreater(r['nominalThreadZoneOverlapMm3'],40)
  self.assertLess(r['threadIntersectionOutsideDeclaredZoneMm3'],1e-6)
  self.assertEqual(r['originalReopenedRoots'],3);self.assertFalse(r['acceptance'])
 def test_original_shapes_valid(self):
  parts=q.originals();self.assertEqual(len(parts),3)
  self.assertTrue(all(q.h.BRepCheck_Analyzer(s).IsValid() for _,s in parts))
if __name__=='__main__':unittest.main()

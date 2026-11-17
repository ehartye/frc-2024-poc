import json,pathlib,sys,unittest
ROOT=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'cad/assembly'))
import check_native_floor_entry as q
class NativeFloorEntryTest(unittest.TestCase):
 def test_bound_full_domain(self):
  r=json.loads((ROOT/'cad/assembly/evidence/native-floor-entry.json').read_text());self.assertEqual(r['inputs'],q.pins())
  self.assertEqual([(x['lateralOffsetMm'],x['advanceMm']) for x in r['rows']],[(y,x) for y in [0,-5,5] for x in range(0,481,10)])
  self.assertEqual(r['sources']['pulley']['expectedSolids'],3);self.assertEqual(r['sources']['bearing']['expectedSolids'],1)
  self.assertTrue(all(x['equivalentCradleLocalGuideAdvanceMm']==480-x['advanceMm'] for x in r['rows']))
 def test_centered_clear_and_offsets_real_material(self):
  r=json.loads((ROOT/'cad/assembly/evidence/native-floor-entry.json').read_text())
  self.assertFalse(any(x['hits'] for x in r['rows'] if x['lateralOffsetMm']==0))
  for offset in [-5,5]:
   rows=[x for x in r['rows'] if x['lateralOffsetMm']==offset]
   for kind,count,threshold in [('pulley',11,400),('bearing',10,60)]:
    self.assertEqual(sum(any(h['part'].startswith(kind) for h in x['hits']) for x in rows),count)
    self.assertGreater(max(h['commonMm3'] for x in rows for h in x['hits'] if h['part'].startswith(kind)),threshold)
  self.assertFalse(r['acceptance'])
if __name__=='__main__':unittest.main()

import json,pathlib,sys,unittest
ROOT=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'cad/assembly'))
import audit_floor_entry as q
class FloorEntryTest(unittest.TestCase):
 def test_source_domain_and_nominal_corridor(self):
  r=json.loads((ROOT/'cad/assembly/evidence/floor-entry.json').read_text());self.assertEqual(r['inputs'],q.pins());self.assertEqual(r['assemblyPartCount'],711)
  for offset in [0,-5,5]:self.assertEqual([x['advanceMm'] for x in r['rows'] if x['lateralOffsetMm']==offset],list(range(0,481,10)))
  nominal=[x for x in r['rows'] if x['lateralOffsetMm']==0]
  self.assertFalse(any(h['contactGroup']=='unresolved' for x in nominal for h in x['hits']))
  first=next(x for x in nominal if any(h['contactGroup']=='pinch' for h in x['hits']));self.assertEqual(first['centerRobotMm'],[338,0,25.4])
 def test_offset_counterexamples_preserved(self):
  r=json.loads((ROOT/'cad/assembly/evidence/floor-entry.json').read_text())
  for offset in [-5,5]:
   bad=[x for x in r['rows'] if x['lateralOffsetMm']==offset and any(h['contactGroup']=='unresolved' for h in x['hits'])]
   self.assertEqual(len(bad),11);self.assertGreater(max(h['overlapMm3'] for x in bad for h in x['hits'] if h['contactGroup']=='unresolved'),1000)
  self.assertFalse(r['acceptance'])
if __name__=='__main__':unittest.main()

import json,pathlib,hashlib,unittest
ROOT=pathlib.Path(__file__).resolve().parents[1]
def read(name):return json.loads((ROOT/'cad/assembly/evidence'/name).read_text())
class SeatedBearingCandidate(unittest.TestCase):
 def test_current_pins(self):
  for filename in ['seated-bearing-assembly.json','seated-bearing-note.json']:
   for name,digest in read(filename)['inputs'].items():self.assertEqual(hashlib.sha256((ROOT/name).read_bytes()).hexdigest(),digest,name)
 def test_full_assembly_delta_motion(self):
  r=read('seated-bearing-assembly.json');self.assertEqual(len(r['samples']),15)
  for s in r['samples']:
   self.assertEqual(s['partCount'],711);self.assertEqual(s['changedPartCount'],10);self.assertEqual(s['hits'],[]);self.assertEqual(s['invalid'],[]);self.assertGreater(s['changedMinimumZmm'],2.64)
  self.assertFalse(r['adopted']);self.assertFalse(r['acceptance'])
 def test_actual_native_material_contained(self):
  r=read('seated-bearing-note.json');self.assertEqual(len(r['nativeMaterialContainment']),8)
  for c in r['nativeMaterialContainment']:self.assertEqual(c['nativeOutsideProxyMm3'],0);self.assertEqual(c['remainingSolids'],0)
 def test_center_clear_offset_counterexamples_remain(self):
  r=read('seated-bearing-note.json');self.assertEqual(len(r['rows']),147)
  for dy,count in [(0,0),(-5,8),(5,8)]:self.assertEqual(sum(bool(q['hits']) for q in r['rows'] if q['lateralOffsetMm']==dy),count)
  self.assertEqual(r['actualFreeFinal']['hits'],[])
  self.assertNotEqual(r['actualFreeFinal']['state']['notePitchRad'],r['actualFreeFinal']['state']['cradlePitchRad'])
if __name__=='__main__':unittest.main()

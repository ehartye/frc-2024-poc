import unittest,json,pathlib,sys,hashlib
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'cad/components'))
from check_transmission import center_distance
class TransmissionTests(unittest.TestCase):
 def setUp(self):self.c=json.loads((R/'cad/components/a2-transmission.json').read_text())
 def test_belt_pitch_geometry(self):
  self.assertAlmostEqual(center_distance(self.c),88.9)
  self.c['layout']['beltTeeth']=42;self.assertAlmostEqual(center_distance(self.c),95.25)
 def test_invalid_geometry(self):
  self.c['layout']['beltTeeth']=12
  with self.assertRaises(ValueError):center_distance(self.c)
 def test_recorded_native_evidence(self):
  e=json.loads((R/'cad/components/transmission-evidence.json').read_text())
  self.assertEqual(e['inputSha256'],hashlib.sha256(json.dumps(self.c,sort_keys=True,separators=(',',':')).encode()).hexdigest())
  self.assertEqual(len(e['pairCommonVolumesMm3']),78)
  self.assertLess(max(e['pairCommonVolumesMm3'].values()),1e-5)
  self.assertGreater(e['wrongDrivenClock30OverlapMm3'],140)
  self.assertFalse(e['acceptance'])
if __name__=='__main__':unittest.main()

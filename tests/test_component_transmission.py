import unittest,json,pathlib,sys,hashlib
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'cad/components'))
from check_transmission import center_distance,screw_stack,motor_originals
class TransmissionTests(unittest.TestCase):
 def setUp(self):self.c=json.loads((R/'cad/components/a2-transmission.json').read_text())
 def test_belt_pitch_geometry(self):
  self.assertAlmostEqual(center_distance(self.c),88.9)
  self.c['layout']['beltTeeth']=42;self.assertAlmostEqual(center_distance(self.c),95.25)
 def test_mount_depth_and_negative_control(self):
  self.assertTrue(screw_stack(self.c)['nominalDepthScreen'])
  self.assertAlmostEqual(screw_stack(self.c)['nominalEngagementMm'],3.7625)
  self.c['motorMount']['candidateScrewUnderHeadMm']=12.7
  self.assertFalse(screw_stack(self.c)['nominalDepthScreen'])
 def test_key_negative_control(self):
  e=json.loads((R/'cad/components/transmission-evidence.json').read_text())
  self.assertGreater(e['unclockedDriverKeyOverlapMm3'],23)
  self.assertAlmostEqual(e['keyNearestSurfaceDistancesMm']['motor'],0)
  self.assertAlmostEqual(e['keyNearestSurfaceDistancesMm']['driver pulley'],.025)
  self.assertLess(e['pairCommonVolumesMm3']['motor / original key envelope'],1e-5)
  self.assertLess(e['pairCommonVolumesMm3']['driver pulley / original key envelope'],1e-5)
 def test_tread_and_retention(self):
  from check_transmission import service_originals,rejected_service_originals
  b=json.loads((R/'cad/components/a2-wheel-cartridge.json').read_text())
  self.assertEqual(len(service_originals(self.c,b)),1)
  self.assertAlmostEqual(b['layout']['bareWheelRadius']+self.c['serviceInterfaces']['tread']['thicknessMm'],49.53)
  self.c['serviceInterfaces']['motorRetention']['shaftEndFromFaceMm']=20
  with self.assertRaises(ValueError):rejected_service_originals(self.c,b)
 def test_invalid_geometry(self):
  self.c['layout']['beltTeeth']=12
  with self.assertRaises(ValueError):center_distance(self.c)
 def test_recorded_native_evidence(self):
  e=json.loads((R/'cad/components/transmission-evidence.json').read_text())
  self.assertEqual(e['inputSha256'],hashlib.sha256(json.dumps(self.c,sort_keys=True,separators=(',',':')).encode()).hexdigest())
  self.assertEqual(len(e['pairCommonVolumesMm3']),171)
  self.assertLess(max(v for k,v in e['pairCommonVolumesMm3'].items() if k not in e['intentionalPressFitPairs']),1e-5)
  for k in e['intentionalPressFitPairs']:self.assertGreater(e['pairCommonVolumesMm3'][k],.8)
  self.assertGreater(e['wrongDrivenClock30OverlapMm3'],140)
  self.assertFalse(e['acceptance'])
if __name__=='__main__':unittest.main()

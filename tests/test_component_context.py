import unittest,pathlib,json,hashlib
R=pathlib.Path(__file__).resolve().parents[1]
class ContextTests(unittest.TestCase):
 def setUp(self):self.r=json.loads((R/'cad/components/a2-context-evidence.json').read_text())
 def test_input_pins(self):
  for key,path in [('contractSha256','experiments/a2-campaign/cad/cycle-contract-A2.json'),('transmissionSha256','cad/components/a2-transmission.json')]:self.assertEqual(self.r[key],hashlib.sha256((R/path).read_bytes()).hexdigest())
 def test_centered_negative_control(self):
  rows=self.r['contactSampling']['rejected central wheel'];center=next(x for x in rows if x['noteCenterFromWheelStationMm']==0);self.assertLess(center['summedNominalOverlapMm3'],1e-5)
  self.assertGreater(max(x['summedNominalOverlapMm3'] for x in rows),1)
 def test_two_lane_window_not_old_stroke(self):
  rows=self.r['contactSampling']['two lane candidate'];contact=[x['noteCenterFromWheelStationMm'] for x in rows if x['summedNominalOverlapMm3']>1e-5]
  self.assertIn(0,contact);self.assertEqual(min(contact),-80);self.assertEqual(max(contact),80);self.assertLess(max(contact)-min(contact),240)
 def test_bom_exposes_four_drives(self):self.assertEqual(len(self.r['bomPerCartridge']),4)
if __name__=='__main__':unittest.main()

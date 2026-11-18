import unittest,pathlib,json,hashlib,importlib.util,copy
R=pathlib.Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('cartridge',R/'cad/components/build_cartridge.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
class CartridgeTests(unittest.TestCase):
 def setUp(self):self.c=json.loads((R/'cad/components/a2-wheel-cartridge.json').read_text());self.r=json.loads((R/'cad/components/cartridge-evidence.json').read_text())
 def test_receipt_input_binding(self):self.assertEqual(self.r['inputConfigSha256'],hashlib.sha256(json.dumps(self.c,sort_keys=True,separators=(',',':')).encode()).hexdigest())
 def test_originals_valid_and_parameter_driven(self):
  a=m.originals(self.c,0);self.assertEqual(len(a),5)
  for s in a.values():self.assertTrue(m.BRepCheck_Analyzer(s).IsValid())
  c=copy.deepcopy(self.c);c['layout']['plateBore']+=1;b=m.originals(c,0);self.assertLess(m.props(b['original bored mount plate 1']).Mass(),m.props(a['original bored mount plate 1']).Mass())
 def test_recorded_native_negative_control(self):
  self.assertEqual(self.r['selectedClockingDegrees'],0);self.assertGreater(self.r['clockingTrials'][1]['shaftWheelOverlapMm3'],100);self.assertEqual(self.r['originalReopenedSolids'],5)
 def test_partial_inertia_not_stationary_plate(self):
  expected=self.r['bareWheelShaftInertiaKgM2']+sum(x['shaftAxisInertiaKgM2'] for x in self.r['originalPartMasses'] if x['rotating']);self.assertAlmostEqual(expected,self.r['accountedRotatingInertiaKgM2']);self.assertFalse(self.c['simulation']['transferAccepted']);self.assertIn('partial',self.r['massCompleteness'])
if __name__=='__main__':unittest.main()

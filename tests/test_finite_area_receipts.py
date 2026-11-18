import json,pathlib,sys,unittest,hashlib
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/pressure'))
import run_comparison as runner
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
class PressureReceiptTest(unittest.TestCase):
 def test_exact_source_inventory(self):
  r=json.loads((R/'sim/pressure/evidence.json').read_text());expected=set(runner.FILES)
  for path in ['sim/integrated/evidence/fast-regulated-index.json','sim/integrated/evidence/lift-duration-domain/100ms.json']:
   expected.add(path);expected.update(json.loads((R/path).read_text())['inputs'])
  self.assertEqual(set(r['inputs']),expected)
  for f,h in r['inputs'].items():self.assertEqual(sha(R/f),h,f)
  self.assertEqual(len(r['fixtures']),24);self.assertEqual(len(r['stiffnessSensitivity']),3);self.assertFalse(r['physicalHandoff'])
 def test_model_change_and_aliased_zero_are_explicit(self):
  r=json.loads((R/'sim/pressure/evidence.json').read_text());rows=[x for x in r['fixtures'] if x['fixture']=='indexed-diagnostic']
  self.assertEqual(rows[0]['result']['activeSamples'],0);self.assertGreater(rows[-1]['result']['activeSamples'],0)
  self.assertAlmostEqual(rows[-1]['modelChangePotentialDeltaJ'],rows[-1]['result']['potentialJ']-r['legacyNormalSpringJ']);self.assertGreater(abs(rows[-1]['modelChangePotentialDeltaJ']),.0005)
 def test_phase_variation_is_not_physical_torque(self):
  r=json.loads((R/'sim/pressure/phase-evidence.json').read_text());base=json.loads((R/'sim/pressure/evidence.json').read_text());expected=dict(base['inputs']);expected['sim/pressure/evidence.json']=sha(R/'sim/pressure/evidence.json');expected['sim/pressure/phase_sensitivity.py']=sha(R/'sim/pressure/phase_sensitivity.py');self.assertEqual(r['inputs'],expected)
  cases=[x for x in r['cases'] if x['resolution']==64];self.assertGreater(max(x['result']['sides']['1']['normalMagnitudeN'] for x in cases)-min(x['result']['sides']['1']['normalMagnitudeN'] for x in cases),.4)
  self.assertFalse(r['acceptance'])
if __name__=='__main__':unittest.main()

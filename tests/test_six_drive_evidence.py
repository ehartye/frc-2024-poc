import unittest,pathlib,json,hashlib
R=pathlib.Path(__file__).resolve().parents[1];H=R/'sim/component-launcher'
class EvidenceTest(unittest.TestCase):
 def setUp(self):self.r=json.loads((H/'six-drive-evidence.json').read_text());self.c={x['scenario']:x for x in self.r['cases']}
 def test_pins(self):
  for f,h in dict(self.r['inputs'],**self.r['cadInputs']).items():self.assertEqual(hashlib.sha256((R/f).read_bytes()).hexdigest(),h,f)
 def test_nominal_and_omission_control(self):
  self.assertEqual(self.c['nominal']['outcome'],'PASSED_TABLE_BOUNDARY');self.assertEqual(self.c['zero-pinch']['outcome'],'REVERSED')
  release=next(e for e in self.c['nominal']['events'] if not e['held']);self.assertLess(abs(release['speedMps']),.001)
 def test_energy_and_refinement(self):
  a,b=self.c['nominal'],self.c['half-step'];self.assertLess(a['maxAbsEnergyResidualJ'],.02);self.assertLess(b['maxAbsEnergyResidualJ'],a['maxAbsEnergyResidualJ']*.51)
  self.assertLess(abs(a['lastModeledDriveContact']['speedMps']-b['lastModeledDriveContact']['speedMps']),.005)
 def test_sensitivities_not_robustness_claim(self):
  self.assertLess(self.c['low-preload']['lastModeledDriveContact']['speedMps'],self.c['nominal']['lastModeledDriveContact']['speedMps'])
  self.assertIsNone(self.c['nine-volt']['releaseTimeS']);self.assertTrue(any(e['stage']=='BROWNOUT' for e in self.c['nine-volt']['events']))
 def test_off_preserves_motion(self):
  for key in ['sensor-loss','load-step','power-off']:
   c=self.c[key];off=[r for r in c['trace'] if max(r['duties'])==0 and not r['held']]
   self.assertTrue(off);self.assertGreater(off[0]['pieceSpeedMps'],0);self.assertTrue(any(w>0 for w in off[0]['omegaRadps']))
 def test_no_acceptance(self):
  self.assertFalse(self.r['acceptance']);self.assertTrue(all(not c['acceptance'] for c in self.r['cases']))
if __name__=='__main__':unittest.main()

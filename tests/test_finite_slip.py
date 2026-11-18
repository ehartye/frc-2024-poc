import pathlib,json,hashlib,sys,unittest,math
R=pathlib.Path(__file__).resolve().parents[1];H=R/'sim/retention';sys.path.insert(0,str(H));import finite_slip as f
import numpy as np
class FiniteSlip(unittest.TestCase):
 def setUp(self):self.r=json.loads((H/'finite-slip-evidence.json').read_text());self.c={x['scenario']:x for x in self.r['cases']}
 def test_prestressed_zero_slip_corotation(self):
  history=np.array([[.001,0.],[0.,-.002]])
  rotated=f.rotate_shear_history(history,math.pi/2)
  np.testing.assert_allclose(rotated,[[0.,.001],[.002,0.]],atol=1e-15)
  self.assertAlmostEqual(float(np.sum(history**2)),float(np.sum(rotated**2)))
  np.testing.assert_allclose(f.rotate_shear_history(rotated,-math.pi/2),history,atol=1e-15)
 def test_source_closure(self):
  for p,h in self.r['inputs'].items():self.assertEqual(hashlib.sha256((R/p).read_bytes()).hexdigest(),h,p)
 def test_free_note_not_ideal_attachment(self):
  n=self.c['locked-hypothesis'];self.assertEqual(n['outcome'],'WITHIN_NOMINAL_PATCH_DOMAIN_HORIZON');self.assertGreater(abs(n['final']['notePitchRad']-n['final']['cradlePitchRad']),.005)
  self.assertTrue(any(abs(r['noteOmegaRadps'])>0 for r in n['trace']));self.assertFalse(n['acceptance'])
 def test_adverse_cases_stop_at_validity_domain(self):
  for key in ['coast','fast','low-preload']:
   self.assertEqual(self.c[key]['outcome'],'NOMINAL_FOOTPRINT_DOMAIN_LOST');self.assertTrue(self.c[key]['domainLostPointIndices']);self.assertLess(self.c[key]['final']['timeS'],.3)
 def test_coast_backdrives_rotors(self):
  self.assertTrue(any(abs(x)>1e-3 for x in self.c['coast']['final']['rotorOmegaRadps']));self.assertEqual(self.c['locked-hypothesis']['final']['rotorOmegaRadps'],[0.,0.])
 def test_energy_and_refinement(self):
  a,b,c=[self.c[k] for k in ['locked-hypothesis','half-step','quarter-step']]
  self.assertLess(a['maxAbsEnergyResidualJ'],.0002);self.assertLess(b['maxAbsEnergyResidualJ'],a['maxAbsEnergyResidualJ']*.6);self.assertLess(c['maxAbsEnergyResidualJ'],b['maxAbsEnergyResidualJ']*.7)
  self.assertLess(abs(a['final']['timeS']-c['final']['timeS']),1e-8)
  for case in self.r['cases']:
   for row in case['trace']:
    e=row['energyJ'];self.assertGreaterEqual(e['slipHeat'],-1e-12);self.assertGreaterEqual(e['groundHeat'],-1e-12)
    expected=e['prescribedCradleWork']-sum(e[k] for k in ['slipHeat','groundHeat','kinetic','potential','shearElastic','groundElastic']);self.assertAlmostEqual(e['residual'],expected)
 def test_finite_friction_slip_not_reset(self):
  self.assertGreater(self.c['low-preload']['final']['energyJ']['slipHeat'],0);self.assertGreater(self.c['coast']['final']['energyJ']['kinetic'],0)
 def test_invalid_parameter_rejection(self):
  cfg=json.loads((H/'finite-slip-config.json').read_text());geo=json.loads((R/'cad/assembly/evidence/retention-patch.json').read_text())['cases'][0]
  for k,v in [('stepS',0),('normalSideN',True),('mu',float('nan')),('lockedBelts',1)]:
   with self.assertRaises(ValueError):f.simulate(dict(cfg,**{k:v}),geo,'locked-hypothesis')
if __name__=='__main__':unittest.main()

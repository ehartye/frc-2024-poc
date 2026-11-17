import pathlib,sys,unittest,json,hashlib,math
R=pathlib.Path(__file__).resolve().parents[1];H=R/'sim/acquisition';sys.path.insert(0,str(H));import ground_entry as g
class GroundModel(unittest.TestCase):
 def setUp(self):self.r=json.loads((H/'evidence.json').read_text());self.c={c['case']:c for c in self.r['cases']}
 def test_current_source_pins(self):
  for f,h in self.r['inputs'].items():self.assertEqual(hashlib.sha256((R/f).read_bytes()).hexdigest(),h,f)
 def test_nominal_and_coast_are_distinct(self):
  self.assertEqual(self.c['braked-hypothesis']['outcome'],'CENTERED_GROUND_DWELL_ONLY');self.assertEqual(self.c['coast']['outcome'],'LEFT_SAMPLED_ENTRY_DOMAIN')
 def test_ground_dwell_does_not_prove_capture(self):
  z=self.c['zero-belt'];self.assertEqual(z['outcome'],'CENTERED_GROUND_DWELL_ONLY');self.assertEqual(z['final']['energyJ']['electricSource'],0);self.assertGreater(z['final']['energyJ']['prescribedBaseWork'],0)
 def test_world_ground_start(self):
  for c in self.r['cases']:
   self.assertEqual(c['trace'][0]['pieceWorldSpeedMps'],0);self.assertAlmostEqual(c['trace'][0]['pieceWorldX'],.658)
 def test_energy_accounts_for_moving_base(self):
  for c in self.r['cases']:
   e=c['final']['energyJ'];expected=e['electricSource']+e['prescribedBaseWork']-sum(e[k] for k in ['windingHeat','sourceHeat','slipHeat','groundHeat','brakeHeat','kinetic'])
   self.assertAlmostEqual(e['residual'],expected);self.assertTrue(all(r['energyJ']['slipHeat']>=0 and r['energyJ']['groundHeat']>=0 for r in c['trace']))
 def test_half_step_improves_energy(self):
  a,b=self.c['braked-hypothesis'],self.c['half-step'];self.assertLess(b['maxAbsEnergyResidualJ'],a['maxAbsEnergyResidualJ']*.51);self.assertLess(abs(a['final']['relativeRobotX']-b['final']['relativeRobotX']),.0001)
 def test_invalid_config(self):
  rows,_=g.load_geometry();c=json.loads((H/'config.json').read_text())
  for k,v in [('stepS',0),('groundMu',float('nan')),('normalEachN',True)]:
   with self.assertRaises(ValueError):g.run_case(dict(c,**{k:v}),rows,'braked-hypothesis')
 def test_offsets_not_accepted(self):
  rows,source=g.load_geometry();self.assertEqual(len(rows),49)
  for o in [-5,5]:self.assertEqual(sum(any(h['contactGroup']=='unresolved' for h in r['hits']) for r in source['rows'] if r['lateralOffsetMm']==o),11)
if __name__=='__main__':unittest.main()

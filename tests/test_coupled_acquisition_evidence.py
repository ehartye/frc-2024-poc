import pathlib,json,hashlib,unittest
R=pathlib.Path(__file__).resolve().parents[1]
class CoupledEvidence(unittest.TestCase):
 def setUp(self):self.cases={k:json.loads((R/('sim/integrated/evidence/acquisition-'+k+'.json')).read_text()) for k in ['nominal','half','off']}
 def test_complete_current_inputs(self):
  for r in self.cases.values():
   self.assertIn('sim/integrated/coupled_acquisition.py',r['inputs']);self.assertIn('sim/drivetrain/plant.py',r['inputs']);self.assertIn('sim/acquisition/free-obstacles.json',r['inputs'])
   for p,h in r['inputs'].items():self.assertEqual(hashlib.sha256((R/p).read_bytes()).hexdigest(),h,p)
 def test_reaction_shared_energy_and_domain_limit(self):
  for r in self.cases.values():
   self.assertTrue(r['outcome'].startswith('UNKNOWN_DOMAIN'));self.assertFalse(r['acceptance']);self.assertIsNotNone(r['firstContact']);self.assertGreater(r['peakReactionForceN'],1.)
   for row in r['trace']:
    e=row['energyJ'];self.assertAlmostEqual(e['residual'],e['initial']+e['sourceWork']-sum(v for k,v in e.items() if k not in ['initial','sourceWork','kinetic','spring','residual'])-e['kinetic']-e['spring'])
   self.assertGreaterEqual(r['final']['relativeRobotXY'][0],.178)
  self.assertLess(self.cases['half']['maxAbsEnergyResidualJ'],self.cases['nominal']['maxAbsEnergyResidualJ']*.3)
 def test_off_preserves_motion_and_removes_demand(self):
  rows=[r for r in self.cases['off']['trace'] if r['timeS']>.61];self.assertTrue(rows)
  for r in rows:self.assertEqual(r['commands']['modes'],['coast','coast']);self.assertEqual(r['commands']['pinch'],[0.,0.])
  self.assertGreater(rows[0]['chassis']['vx'],.5);self.assertGreater(max(abs(w) for w in rows[0]['note']['w']),1.)
if __name__=='__main__':unittest.main()

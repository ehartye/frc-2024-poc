import unittest,pathlib,sys,json,hashlib
R=pathlib.Path(__file__).resolve().parents[1];sys.path[:0]=[str(R/'sim/pressure'),str(R/'sim/acquisition')]
import run_acquisition as runner
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
class EntryReceiptTest(unittest.TestCase):
 def test_exact_source_inventory(self):
  r=json.loads((R/'sim/pressure/acquisition-entry-evidence.json').read_text());_,_,pins=runner.run_free_planar.load(R/'sim/acquisition/free-obstacles.json');pins.update({f:sha(R/f) for f in runner.FILES+runner.drive_source.FILES});self.assertEqual(r['inputs'],pins);self.assertFalse(r['acceptance']);self.assertEqual(r['pressureConfig']['resolution'],[1024,1024])
 def test_fresh_energy_state_and_continuous_entry(self):
  r=json.loads((R/'sim/pressure/acquisition-entry-evidence.json').read_text());s=r['initialState'];self.assertEqual(s['timeS'],0);self.assertEqual(s['pressurePotentialJ'],0);self.assertEqual(s['note']['w'],[0.,0.]);self.assertEqual(s['note']['v'],[0.,0.]);self.assertTrue(all(v==0 for v in s['energy'].values()));self.assertEqual(r['outcome'],'HORIZON');self.assertEqual(len(r['trace']),580)
  for i,row in enumerate(r['trace']):
   self.assertAlmostEqual(row['timeS'],(i+1)*.001,places=12);self.assertEqual(row['energyJ']['initial'],0.);self.assertLess(abs(row['pressureWorkResidualW']),1e-10)
  self.assertIsNotNone(r['firstContact']);self.assertGreater(r['final']['energyJ']['spring'],0.);self.assertGreater(r['final']['pressureFeatures']['outer-wrap'],0)
if __name__=='__main__':unittest.main()

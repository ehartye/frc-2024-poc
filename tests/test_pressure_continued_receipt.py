import json,pathlib,sys,hashlib,unittest
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/pressure'))
import run_early_continuation as r
class ContinuedReceipt(unittest.TestCase):
 def test_prefix_exact_lineage_and_nonzero_momentum(self):
  p,c,pins,bus,ancestry=r.load();report=json.loads((R/'sim/pressure/early-continued-20ms-evidence.json').read_text())
  self.assertEqual(report['inputs'],pins);self.assertEqual(report['ancestry'],ancestry)
  self.assertEqual(report['startCheckpoint'],r.c.export(p));self.assertEqual(report['controllerStart'],vars(c))
  self.assertTrue(all(hashlib.sha256((R/f).read_bytes()).hexdigest()==h for f,h in pins.items()))
  self.assertEqual(len(report['trace']),20);self.assertEqual(report['outcome'],'HORIZON')
  end=r.c.restore(report['endCheckpoint']);self.assertGreater(end.drive.state['vx'],.4);self.assertGreater(abs(end.note['v'][0]),.09)
  self.assertAlmostEqual(end.t,.682,places=12);self.assertEqual(end.energy,report['endCheckpoint']['state']['ledger'])
 def test_longer_prefix_and_finite_momentum(self):
  short=json.loads((R/'sim/pressure/early-continued-20ms-evidence.json').read_text())
  long=json.loads((R/'sim/pressure/early-continued-200ms-evidence.json').read_text())
  self.assertEqual(short['inputs'],long['inputs']);self.assertEqual(short['startCheckpoint'],long['startCheckpoint'])
  self.assertEqual(short['controllerStart'],long['controllerStart']);self.assertEqual(short['trace'],long['trace'][:20])
  self.assertEqual(short['events'],long['events'][:2]);self.assertEqual(long['outcome'],'HORIZON')
  self.assertEqual(len(long['trace']),200);end=r.c.restore(long['endCheckpoint'])
  self.assertGreater(end.drive.state['vx'],.2);self.assertGreater(abs(end.note['v'][0]),.015)
  q=end.note['p'][0]-end.drive.state['x'];self.assertGreater(q,.178);self.assertLess(q,.19)
  self.assertAlmostEqual(end.t,.862,places=12);self.assertFalse(long['acceptance'])
if __name__=='__main__':unittest.main()

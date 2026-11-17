import unittest,pathlib,sys,json
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/pressure'))
import run_continuation as r
class ContinuationReceiptTest(unittest.TestCase):
 def test_exact_lineage_and_state(self):
  e=json.loads((R/'sim/pressure/continuation-20ms-evidence.json').read_text());p,origin=r.c.from_pressure_report(R/'sim/pressure/acquisition-entry-evidence.json');start=r.c.export(p)
  pins=dict(start['identity']['inputs']);pins.update({f:r.c.factory.r.sha(R/f) for f in r.FILES});pins['sim/pressure/acquisition-entry-evidence.json']=origin
  self.assertEqual(pins,e['inputs']);self.assertEqual(start,e['startCheckpoint']);self.assertEqual(e['ancestrySha256'],origin);self.assertEqual(len(e['trace']),20);self.assertEqual(e['outcome'],'HORIZON')
  q=r.c.restore(e['endCheckpoint']);self.assertAlmostEqual(q.t,.6,places=12);self.assertEqual(q.note,e['trace'][-1]['note']);self.assertGreater(q.energy['sourceWork'],p.energy['sourceWork']);self.assertLess(q.note['p'][0],p.note['p'][0])
if __name__=='__main__':unittest.main()

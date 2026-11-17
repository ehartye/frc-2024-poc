import unittest,json,pathlib,hashlib
ROOT=pathlib.Path(__file__).resolve().parents[1]
class IndexingCorridor(unittest.TestCase):
 def test_pins_and_counterexamples(self):
  r=json.loads((ROOT/'cad/assembly/evidence/indexing-corridor.json').read_text())
  for n,v in r['inputs'].items():self.assertEqual(hashlib.sha256((ROOT/n).read_bytes()).hexdigest(),v,n)
  self.assertEqual(len(r['samples']),90)
  passing=[]
  for offset in r['offsetCandidatesMm']:
   rows=[s for s in r['samples'] if s['offsetMm']==offset];self.assertEqual(len(rows),15)
   if not any(not h['intendedContactHypothesis'] for s in rows for h in s['hits']):passing.append(offset)
  self.assertEqual(passing,[0]);self.assertFalse(r['acceptance'])
  row=next(s for s in r['samples'] if s['offsetMm']==5 and s['phase']=='rotate' and s['pitchDeg']==10)
  self.assertTrue(any(h['part']=='original rigid front backing support proposal' for h in row['hits']))
if __name__=='__main__':unittest.main()

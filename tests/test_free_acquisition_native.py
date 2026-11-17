import unittest,json,pathlib,hashlib
ROOT=pathlib.Path(__file__).resolve().parents[1]
class NativeAcquisitionWitness(unittest.TestCase):
 def test_bound_inputs(self):
  r=json.loads((ROOT/'cad/assembly/evidence/free-acquisition-native.json').read_text())
  for name,digest in r['inputs'].items():self.assertEqual(hashlib.sha256((ROOT/name).read_bytes()).hexdigest(),digest,name)
  self.assertEqual(r['experimentReceiptSha256'],'1e3bf6d0566691bce05bfaed512f0ab1226eecef8587803fe8da988f25cb0d26')
 def test_exact_diagnostic_inventory(self):
  r=json.loads((ROOT/'cad/assembly/evidence/free-acquisition-native.json').read_text())
  self.assertEqual([(x['case'],x['state']) for x in r['rows']],[(c,s) for c in ['right','stiff-right','stiffer-right'] for s in ['final','stoppedAttempt']])
  for row in r['rows']:
   self.assertEqual(len(row['checks']),28)
   self.assertEqual(row['hits'],[])
   self.assertTrue(all(x['commonMm3']==0 for x in row['checks']))
   self.assertEqual(row['centerRobotMm'][2],25.4)
  self.assertFalse(r['acceptance'])
  self.assertTrue(any('phase' in s for s in r['limits']))
if __name__=='__main__':unittest.main()

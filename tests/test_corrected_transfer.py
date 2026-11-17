import unittest,pathlib,json,hashlib,sys,tempfile
R=pathlib.Path(__file__).resolve().parents[1];H=R/'sim/component-launcher';sys.path.insert(0,str(H));import run_six_drive as q
class CorrectedTransfer(unittest.TestCase):
 def test_current_complete_gate(self):
  t,n=q.validate_reports(H/'corrected-transfer-evidence.json',H/'corrected-native-wheel-evidence.json')
  self.assertEqual(t['assemblyPartCount'],711);self.assertFalse(t['acceptance']);self.assertEqual(n['rows'],[])
  self.assertTrue(all(h['contactGroup']!='unintended' for r in t['rows'] for h in r['hits']))
 def test_missing_dependency_rejected(self):
  t=json.loads((H/'corrected-transfer-evidence.json').read_text());t['inputs'].pop(next(iter(t['inputs'])))
  with tempfile.TemporaryDirectory() as d:
   p=pathlib.Path(d)/'report.json';p.write_text(json.dumps(t))
   with self.assertRaisesRegex(ValueError,'inventory'):q.validate_reports(p,H/'corrected-native-wheel-evidence.json')
 def test_incomplete_domain_rejected(self):
  t=json.loads((H/'corrected-transfer-evidence.json').read_text());t['rows']=t['rows'][:1]
  with tempfile.TemporaryDirectory() as d:
   p=pathlib.Path(d)/'report.json';p.write_text(json.dumps(t))
   with self.assertRaisesRegex(ValueError,'domain'):q.validate_reports(p,H/'corrected-native-wheel-evidence.json')
 def test_native_identity_rejected(self):
  n=json.loads((H/'corrected-native-wheel-evidence.json').read_text());n['source']['sha256']='0'*64
  with tempfile.TemporaryDirectory() as d:
   p=pathlib.Path(d)/'report.json';p.write_text(json.dumps(n))
   with self.assertRaisesRegex(ValueError,'identity'):q.validate_reports(H/'corrected-transfer-evidence.json',p)
if __name__=='__main__':unittest.main()

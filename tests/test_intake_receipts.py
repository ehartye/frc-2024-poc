import pathlib,tempfile,json,sys,unittest
ROOT=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'cad/assembly'))
import verify_receipts as v
class ReceiptTest(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.root=pathlib.Path(self.tmp.name);self.pins={'generator':'abc'};self.report={'schema':'2024-intake-component-candidate/1','inputs':self.pins,'states':[]}
  for state in v.STATES:
   p=self.root/state/'original-robot.step';p.parent.mkdir();p.write_bytes(b'unit fixture:'+state.encode());self.report['states'].append({'state':state,'parts':1,'reopenedRoots':1,'stepSha256':v.sha(p)})
  self.save()
 def tearDown(self):self.tmp.cleanup()
 def save(self):(self.root/'report.json').write_text(json.dumps(self.report))
 def valid(self):return v.verify_exports(self.root,self.pins)['valid']
 def test_exact_files_match(self):self.assertTrue(self.valid())
 def test_missing_step(self):(self.root/'floor/original-robot.step').unlink();self.assertFalse(self.valid())
 def test_modified_step(self):(self.root/'floor/original-robot.step').write_bytes(b'mutated');self.assertFalse(self.valid())
 def test_forged_hash(self):self.report['states'][0]['stepSha256']='0'*64;self.save();self.assertFalse(self.valid())
 def test_missing_state(self):self.report['states'].pop();self.save();self.assertFalse(self.valid())
 def test_duplicate_state(self):self.report['states'][0]['state']='handoff';self.save();self.assertFalse(self.valid())
 def test_mutated_source(self):self.assertFalse(v.verify_exports(self.root,{'generator':'changed'})['valid'])
 def test_source_changed_during_audit(self):self.assertFalse(v.source_stable(self.pins,{'generator':'changed'}))
 def test_missing_receipt(self):(self.root/'report.json').unlink();self.assertFalse(self.valid())
if __name__=='__main__':unittest.main()

import pathlib,sys,json,tempfile,unittest,copy
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/integrated'))
from unittest.mock import patch
import run_current_checkpoint_chain as c
class Chain(unittest.TestCase):
 def test_valid_current_handoff(self):
  c.source_pins();c.check_handoff(json.loads((R/c.INDEX).read_text()),json.loads((R/c.PILOT).read_text()))
 def test_byte_difference_requires_registration(self):
  with tempfile.TemporaryDirectory() as d:
   a=pathlib.Path(d)/'a';b=pathlib.Path(d)/'b';a.write_bytes(b'{"x":1}');b.write_bytes(a.read_bytes());self.assertEqual(c.check_index(a,b),{'x':1});a.write_bytes(b'{"x":2}')
   with self.assertRaisesRegex(ValueError,'registration'):c.check_index(a,b)
 def test_no_reset_or_ledger_loss(self):
  i=json.loads((R/c.INDEX).read_text());p=json.loads((R/c.PILOT).read_text())
  for key in ['drive','note']:
   bad=copy.deepcopy(p);bad['cases'][0]['initialState'][key]={}
   with self.assertRaises(ValueError):c.check_handoff(i,bad)
  bad=copy.deepcopy(p);bad['cases'][0]['initialCumulativeEnergyJ']['sourceWork']=0
  with self.assertRaises(ValueError):c.check_handoff(i,bad)
 def test_existing_output_refused(self):
  with tempfile.TemporaryDirectory() as d:
   with self.assertRaises(FileExistsError):c.run(pathlib.Path(d))
 def test_failed_execution_never_produces_chain_receipt(self):
  with tempfile.TemporaryDirectory() as d:
   out=pathlib.Path(d)/'fresh'
   with patch.object(c.subprocess,'run',side_effect=c.subprocess.CalledProcessError(1,['producer'])) as producer:
    with self.assertRaises(c.subprocess.CalledProcessError):c.run(out)
   self.assertEqual(producer.call_count,1);self.assertFalse((out/'chain.json').exists())
if __name__=='__main__':unittest.main()

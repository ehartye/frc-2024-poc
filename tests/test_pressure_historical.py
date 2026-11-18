import unittest,pathlib,sys,json,tarfile,io
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/pressure'))
import verify_historical as v
class HistoricalReplay(unittest.TestCase):
 def test_exact_old_blob_available_after_fix(self):
  r=json.loads((R/'sim/pressure/acquisition-entry-evidence.json').read_text());name='sim/pressure/fast_geometry.py';old=v.recover(R,name,r['inputs'][name]);self.assertEqual(v.sha(old),r['inputs'][name]);self.assertNotEqual(v.sha(old),v.sha((R/name).read_bytes()))
 def test_paths_and_unknown_hash_fail_closed(self):
  for path in ['../outside','/absolute','C:/absolute','sim\\outside']:
   with self.assertRaises(ValueError):v.recover(R,path,'0'*64)
  with self.assertRaises(ValueError):v.recover(R,'sim/pressure/fast_geometry.py','0'*64)
 def test_registry_and_archive_are_explicit(self):
  case=v.reviewed_case('entry');self.assertEqual(set(case['tests']),{'test_pressure_acquisition.py','test_pressure_entry_receipt.py'})
  with self.assertRaises(ValueError):v.reviewed_case('arbitrary')
  for kind in [tarfile.SYMTYPE,tarfile.LNKTYPE,tarfile.FIFOTYPE]:
   stream=io.BytesIO()
   with tarfile.open(fileobj=stream,mode='w') as t:
    m=tarfile.TarInfo('payload');m.type=kind;m.linkname='elsewhere';t.addfile(m)
   stream.seek(0)
   with tarfile.open(fileobj=stream) as t:
    with self.assertRaises(ValueError):v.checked_archive(t)
if __name__=='__main__':unittest.main()

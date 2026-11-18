import unittest,sys,pathlib,json,hashlib,math
ROOT=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'cad/assembly'))
import servo_opening_relocation as m
class Relocation(unittest.TestCase):
 def test_unilateral_domain(self):
  for bad in [-1,.1,True,float('nan')]:
   with self.assertRaises(ValueError):m.local_parts(0,bad)
  self.assertEqual(len(m.local_parts(math.pi,0)),len(m.local_parts(math.pi,1.5)))
 def test_receipt_closure(self):
  required={'servo_opening_relocation.py','servo_opening_mount.py','cassette_opening_cam.py','cassette-opening-source-candidate.json','audit_cartridge_interfaces.py'}
  for name in ['servo-opening-relocation-screen.json','servo-opening-relocation-native.json']:
   r=json.loads((ROOT/'cad/assembly/evidence'/name).read_text(encoding='utf-8-sig'))
   self.assertTrue({'cad/assembly/'+n for n in required}<=r['inputs'].keys())
   for p,sha in r['inputs'].items():self.assertEqual(hashlib.sha256((ROOT/p).read_bytes()).hexdigest(),sha,p)
 def test_no_installation_claim(self):
  r=json.loads((ROOT/'cad/assembly/evidence/servo-opening-relocation-native.json').read_text())
  self.assertFalse(r['accepted']);self.assertGreater(sum(r['nativeHornServoCommonMm3PerSolid']),0)
  self.assertEqual(r['authoredMutualHits'],[])
  for row in r['rows']:self.assertTrue(all(v==0 for v in row['commonMm3PerSolid']))
 def test_separated_and_contact_states(self):
  r=json.loads((ROOT/'cad/assembly/evidence/servo-opening-relocation-screen.json').read_text())
  self.assertEqual(len(r['rows']),27)
  self.assertTrue(all(not row['hits'] for row in r['rows']))
  for pose in ['floor','rotated','handoff']:
   self.assertEqual(len([x for x in r['rows'] if x['pose']==pose]),9)
if __name__=='__main__':unittest.main()

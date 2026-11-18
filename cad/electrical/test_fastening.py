import json,pathlib,sys,unittest,hashlib
HERE=pathlib.Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
import build_electrical as e
from fasteners import stack
from check_fastening import estimates,service
class FasteningTests(unittest.TestCase):
 def test_short_screw_rejected(self):
  with self.assertRaises(ValueError):stack('bad',0,0,10,0,10)
 def test_selected_stack(self):
  p=stack('good',0,0,6,0,15.875);self.assertEqual(len(p),3)
  self.assertTrue(all(e.BRepCheck_Analyzer(s).IsValid() for _,s,_ in p))
 def test_separate_material_estimates(self):
  q=estimates(e.build());self.assertGreater(q['estimatedOriginalMountMassKg'],3);self.assertTrue(q['estimateIsNotMeasuredMass'])
  self.assertNotIn('battery',[r['id'] for r in q['items']]);self.assertNotIn('pdh',[r['id'] for r in q['items']])
 def test_battery_removal(self):
  q=service(e.build());self.assertTrue(all(not r['overlaps'] for r in q['batteryLiftSamples']));self.assertFalse(q['isContinuousSweepProof'])
 def test_native_bound_receipt(self):
  q=json.loads((HERE/'generated/fastening-verification.json').read_text());self.assertEqual(q['nativePDHFastenerOverlaps'],[]);self.assertEqual(q['nativeBreakerMountOverlaps'],[])
  self.assertEqual(set(q['inputs']), {'manifest.json','mounts.py','fasteners.py','fastening-spec.json','build_electrical.py','check_drivetrain.py','check_fastening.py','pdh-mount-evidence.json','breaker_mount.py','breaker-mount-evidence.json','../a2_primitives.py'})
  for n,h in q['inputs'].items():self.assertEqual(hashlib.sha256((HERE/n).read_bytes()).hexdigest(),h)
 def test_deliberate_breaker_revision(self):
  c=next(x for x in e.load()['components'] if x['id']=='main-breaker');self.assertIn('CB185',c['partNumber']);v=json.loads((HERE/'breaker-mount-evidence.json').read_text());self.assertIn('1/4-28',v['electricalStudThread']);self.assertIn('CB285',v['supersedes'])
if __name__=='__main__':unittest.main()

import pathlib,json,hashlib,unittest,math
R=pathlib.Path(__file__).resolve().parents[1]
class Handoff(unittest.TestCase):
 def test_source_and_state_preserved(self):
  r=json.loads((R/'sim/retention/free-handoff-evidence.json').read_text());ref=json.loads((R/'sim/retention/refinement-evidence.json').read_text())
  for p,h in r['inputs'].items():self.assertEqual(hashlib.sha256((R/p).read_bytes()).hexdigest(),h,p)
  self.assertEqual(r['state'],ref['cases'][-1]['final']);self.assertEqual(r['assemblyPartCount'],711);self.assertFalse(r['acceptance']);self.assertFalse(r['invalidParts'])
  self.assertGreater(abs(r['state']['notePitchRad']-math.pi/3),.05);self.assertGreater(abs(r['state']['noteVelocityXZ'][1]),.04)
 def test_saved_pose_contacts_not_transfer_acceptance(self):
  r=json.loads((R/'sim/retention/free-handoff-evidence.json').read_text());self.assertEqual(len(r['hits']),2);self.assertTrue(all(x['group']=='pinch' and x['overlapMm3']>0 for x in r['hits']))
if __name__=='__main__':unittest.main()

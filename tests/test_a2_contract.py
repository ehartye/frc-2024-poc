import json,math,pathlib,unittest
R=pathlib.Path(__file__).resolve().parents[1]
class A2Contract(unittest.TestCase):
 def test_release_is_one_geometry(self):
  c=json.loads((R/'cad/cycle-contract-A2.json').read_text());s=c['geometry']['shooter'];a=math.radians(s['releasePitchDeg']);self.assertAlmostEqual(s['pivot'][0]+s['guideLength']*math.cos(a),.25);self.assertAlmostEqual(s['pivot'][2]+s['guideLength']*math.sin(a),.7);self.assertIsNone(s['spinRadPerS']);self.assertIn('unverified',s['requiredSpeedBasis'])
 def test_release_guard_preserves_unknown(self):
  c=json.loads((R/'cad/cycle-contract-A2.json').read_text());self.assertIn('unresolved',c['releaseAcceptance']);self.assertIn('verified release envelope',next(s for s in c['states'] if s['id']=='release')['guard'])
if __name__=='__main__':unittest.main()

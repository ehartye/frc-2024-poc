import unittest,pathlib,json,hashlib,math
ROOT=pathlib.Path(__file__).resolve().parents[1]
class ProfileTest(unittest.TestCase):
 def test_source_and_script_bound(self):
  q=json.loads((ROOT/'cad/components/pulley-outer-profile.json').read_text())
  self.assertEqual(q['nativeSolidCount'],3)
  self.assertEqual(q['scriptSha256'],hashlib.sha256((ROOT/'cad/components/check_pulley_outer_profile.py').read_bytes()).hexdigest())
  self.assertTrue(all(b['nativeRadialUpperBoundMm']<b['proposedOuterRadiusMm'] for b in q['bands']))
 def test_directional_radius_bound(self):
  for j in range(257):
   angle=2*math.pi*j/257;x=13.16*math.cos(angle);y=13.16*math.sin(angle)
   bound=max(max(abs(x*math.cos(a)-y*math.sin(a)),abs(x*math.sin(a)+y*math.cos(a))) for a in [k*math.pi/32 for k in range(16)])/math.cos(math.pi/64)
   self.assertGreaterEqual(bound+1e-12,13.16)
if __name__=='__main__':unittest.main()

import pathlib,sys,unittest,math,json
R=pathlib.Path(__file__).resolve().parents[1];sys.path[:0]=[str(R/'sim/integrated'),str(R/'cad/assembly')]
import finite_retention_guide as old,coupled_lift_guide_adapter as adapter
class AdapterTest(unittest.TestCase):
 @classmethod
 def setUpClass(cls):cls.c=old.load_contract(R)
 def test_only_import_diff(self):
  a=(R/'sim/integrated/coupled_lift_phase.py').read_text();b=(R/'sim/integrated/coupled_lift_separated_phase.py').read_text()
  self.assertEqual(a.replace('finite_retention_guide as guide','coupled_lift_guide_adapter as guide'),b)
 def test_supported_unchanged(self):
  self.assertEqual(old.query_round_xz([.1885674400200612,.0254],0.,self.c),adapter.query_round_xz([.1885674400200612,.0254],0.,self.c))
 def test_negative_pitch_separated(self):
  q=adapter.query_round_xz([.1885674400200612,.0254],-1e-7,self.c)
  self.assertTrue(q['separationFallbackUsed']);self.assertEqual(q['penetrationM'],0.);self.assertEqual(q['originalReason'],'unsupported attitude');self.assertIn('UNKNOWN',q['nativeMaterialStatus'])
 def test_overlapping_remains_unknown(self):
  g=self.c['guide'];p=[g['guideFaceXmm']/1000,(g['bottomZmm']+g['topZmm'])/2000]
  original=old.query_round_xz(p,-.01,self.c);self.assertEqual(adapter.query_round_xz(p,-.01,self.c),original);self.assertEqual(original['status'],'UNKNOWN')
 def test_archived_disagreement_unchanged(self):
  for v in self.c['knownDisagreements']:
   x,y,z=[q/1000 for q in v['centerRobotMm']];a=math.radians(v['pitchDeg']);q=old.query_round_xz([x,z],a,self.c,y_m=y)
   self.assertEqual(q['status'],'UNKNOWN');self.assertEqual(adapter.query_round_xz([x,z],a,self.c,y_m=y),q)
if __name__=='__main__':unittest.main()

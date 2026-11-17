import pathlib,json,sys,unittest,hashlib
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/retention'))
from refine_slip import compare
class Refinement(unittest.TestCase):
 def setUp(self):self.r=json.loads((R/'sim/retention/refinement-evidence.json').read_text())
 def test_input_closure(self):
  for p,h in self.r['inputs'].items():self.assertEqual(hashlib.sha256((R/p).read_bytes()).hexdigest(),h,p)
 def test_saved_metrics_not_acceptance(self):
  self.assertFalse(self.r['acceptance']);a,b=self.r['comparisons']
  for key in ['notePitchRadMaxAbsDelta','noteOmegaRadpsMaxAbsDelta','noteCenterXZMaxNormDelta','noteVelocityXZMaxNormDelta']:self.assertLess(b[key],a[key])
  self.assertLess(b['noteCenterXZMaxNormDelta'],.000012);self.assertGreater(b['noteOmegaRadpsMaxAbsDelta'],.05)
  self.assertEqual([x['stepS'] for x in self.r['cases']],[.000025,.0000125,.00000625])
 def test_interpolation_on_manufactured_trace(self):
  def trace(times,offset):return dict(trace=[dict(timeS=t,notePitchRad=2*t+offset,noteOmegaRadps=2,noteCenterXZ=[t,2*t],noteVelocityXZ=[1,2]) for t in times])
  result=compare(trace([0,.5,1],0),trace([0,.25,.5,.75,1],.1));self.assertAlmostEqual(result['notePitchRadMaxAbsDelta'],.1);self.assertAlmostEqual(result['noteCenterXZMaxNormDelta'],0)
if __name__=='__main__':unittest.main()

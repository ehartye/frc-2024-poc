import pathlib,sys,unittest,copy
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/integrated'))
from coupled_lift_run import setup
from coupled_lift_diagnostic import CoupledLift,Reference,diagnose
class DiagnosticTest(unittest.TestCase):
 @classmethod
 def setUpClass(cls):cls.args=setup()
 def plant(self,kind):
  source,patch,a,h,g,c,pins=self.args
  return kind(source['checkpoint'],patch,a,h,g,c,source['inputs'])
 def test_supported_exact_pure_rates(self):
  a=self.plant(CoupledLift);b=self.plant(Reference);before=copy.deepcopy((a.s,a.ledger));cmd=a.commands('gravity-feedforward')
  self.assertEqual(a.rates(a.s,12.,cmd,{}),b.rates(b.s,12.,cmd,{}));self.assertEqual(before,(a.s,a.ledger))
 def test_exact_rejected_witness_trial_not_committed(self):
  p=self.plant(CoupledLift);s=copy.deepcopy(p.s);s['noteZ']+=.1;before=copy.deepcopy((p.s,p.ledger))
  try:p.rates(s,11.3,p.commands('gravity-feedforward'),{})
  except ValueError as e:
   q=diagnose(p,e.rejectedEvaluation);self.assertEqual(q['state'],s);self.assertEqual(q['busV'],11.3);self.assertFalse(q['accepted']);self.assertFalse(q['patchWitnesses']['allNominalWitnessesInside'])
  else:self.fail('expected witness-domain rejection')
  self.assertEqual(before,(p.s,p.ledger))
if __name__=='__main__':unittest.main()

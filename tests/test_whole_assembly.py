import unittest,json,pathlib,sys
ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'cad/assembly'))
import build_robot as b
class AssemblyTest(unittest.TestCase):
 @classmethod
 def setUpClass(cls):cls.parts,cls.dc=b.build()
 def test_dependency_mutation_invalidates_receipt(self):
  import tempfile
  with tempfile.TemporaryDirectory() as tmp:
   r=pathlib.Path(tmp)
   for name in b.DEPENDENCIES:
    q=r/name;q.parent.mkdir(parents=True,exist_ok=True);q.write_bytes((ROOT/name).read_bytes())
   report={'inputs':b.input_pins(r)};self.assertTrue(b.receipt_matches(report,r))
   p=r/'cad/components/a2-transmission.json';p.write_bytes(p.read_bytes()+b' ')
   self.assertFalse(b.receipt_matches(report,r))
 def test_flat_note_underpass_and_pitch_negative(self):
  import bumper_candidate as bumper
  self.assertAlmostEqual(bumper.underpass_margin(0),6.2)
  self.assertLess(bumper.underpass_margin(5),0)
  self.assertLess(bumper.CONFIG['topMm']+bumper.CONFIG['clothAllowanceMm'],190.5)
 def test_bridge_bolts_have_registered_clearance(self):
  from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
  from OCP.BRepGProp import BRepGProp
  from OCP.GProp import GProp_GProps
  parts={n:s for n,s,k in self.parts};count=0
  for n,s in parts.items():
   if 'bridge bolt nominal' not in n:continue
   prefix=n.split(' bridge bolt nominal')[0];bridge=parts[prefix+' slotted carrier bridge proposal'];g=GProp_GProps();BRepGProp.VolumeProperties_s(BRepAlgoAPI_Common(s,bridge).Shape(),g);self.assertLess(abs(g.Mass()),1e-5);shifted=b.h.move(s,dx=1);bad=GProp_GProps();BRepGProp.VolumeProperties_s(BRepAlgoAPI_Common(shifted,bridge).Shape(),bad);self.assertGreater(bad.Mass(),1);count+=1
  self.assertEqual(count,8)
 def test_real_chassis(self):
  self.assertAlmostEqual(self.dc['centerTrackMm'],579.60768)
  self.assertEqual(self.dc['frameNativeEnvelopeMm'][:2],[719.074,711.2])
 def test_no_legacy_chassis_or_battery(self):
  self.assertFalse(any(n.startswith(('frame longitudinal','wheel (','batteryReserve','electronicsReserve','original-crossmember')) for n,s,k in self.parts))
 def test_corrected_tread(self):
  names=[n for n,s,k in self.parts]
  self.assertEqual(sum('nominal tread annulus' in n for n in names),4)
  tx=json.loads((ROOT/'cad/components/a2-transmission.json').read_text());t=tx['serviceInterfaces']['tread'];self.assertAlmostEqual(t['nativeSeatRadiusMm']+t['thicknessMm'],48.26)
 def test_valid_original_solids(self):
  for n,s,k in self.parts:self.assertTrue(b.h.BRepCheck_Analyzer(s).IsValid(),n)
 def test_intake_not_misrepresented(self):
  self.assertTrue(any(n.startswith('UNRESOLVED intake') for n,s,k in self.parts))
  self.assertTrue(all(k=='service' for n,s,k in self.parts if n.startswith('UNRESOLVED')))
 def test_partial_mass_is_not_whole_cog(self):
  q=b.mass_ledger(self.parts,self.dc);self.assertIsNone(q['wholeRobotCoG']);self.assertGreater(q['estimatedOriginalHardware']['subtotalKg'],0);self.assertAlmostEqual(q['sourceKnown']['subtotalKg'],13.42049774,places=6)
if __name__=='__main__':unittest.main()

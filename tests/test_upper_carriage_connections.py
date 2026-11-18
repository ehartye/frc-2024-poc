import unittest,sys,pathlib,json,hashlib
ROOT=pathlib.Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'cad'),str(ROOT/'cad/assembly')]
import upper_carriage_connections as q
class UpperConnectionTest(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.base,_=q.lower.apply(q.assembly.build('floor'));cls.parts,cls.joints=q.apply(cls.base,25.4)
 def test_all_new_parts_valid(self):
  self.assertEqual(len(self.parts)-len(self.base),52)
  for n,s,k in self.parts[len(self.base):]:self.assertTrue(q.h.BRepCheck_Analyzer(s).IsValid(),n)
 def test_original_yokes_are_connected(self):
  from OCP.TopExp import TopExp_Explorer
  from OCP.TopAbs import TopAbs_SOLID
  for n,s,k in self.parts:
   if k!='original-drilled-yoke':continue
   e=TopExp_Explorer(s,TopAbs_SOLID);count=0
   while e.More():count+=1;e.Next()
   self.assertEqual(count,1,n)
 def test_undrilled_upper_post_blocks_fastener(self):
  shaft=next(s for n,s,k in self.parts[len(self.base):] if n.startswith('original upper portal gusset -1') and n.endswith('-shaft'))
  guide=next(s for n,s,k in self.base if n=='intake candidate stationary aluminum guide -1')
  self.assertGreater(q.lower.volume(shaft,guide),50)
  drilled=next(s for n,s,k in self.parts if n=='intake candidate stationary aluminum guide -1')
  self.assertLess(abs(q.lower.volume(shaft,drilled)),1e-5)
 def test_receipt_bound_and_no_unknown_contacts(self):
  r=json.loads((ROOT/'cad/assembly/evidence/upper-carriage-connections.json').read_text())
  self.assertEqual(r['assemblyInputs'],q.assembly.input_pins())
  self.assertEqual(r['scriptSha256'],hashlib.sha256(pathlib.Path(q.__file__).read_bytes()).hexdigest())
  self.assertEqual(len(r['samples']),15);self.assertFalse(any(x['hits'] for x in r['samples']))
if __name__=='__main__':unittest.main()

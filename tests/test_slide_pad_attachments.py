import unittest,sys,pathlib
ROOT=pathlib.Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'cad'),str(ROOT/'cad/assembly')]
import slide_pad_attachments as q
class PadTest(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.base=q.assembly.intake_components.build('floor');cls.parts,cls.joints=q.apply(cls.base,25.4)
 def test_count_valid_and_connected_brackets(self):
  from OCP.TopExp import TopExp_Explorer
  from OCP.TopAbs import TopAbs_SOLID
  self.assertEqual(len(self.parts)-len(self.base),104)
  for n,s,k in self.parts[len(self.base):]:
   self.assertTrue(q.h.BRepCheck_Analyzer(s).IsValid(),n)
   if k.startswith('original-machined'):
    e=TopExp_Explorer(s,TopAbs_SOLID);count=0
    while e.More():count+=1;e.Next()
    self.assertEqual(count,1,n)
 def test_nominal_material_and_wear(self):
  for j in self.joints:
   self.assertGreater(j['remainingPadBelowCountersinkMm'],3)
   self.assertGreater(j['padThreadBeyondNutMm'],2*25.4/32)
   self.assertEqual(j['wearBeforeHeadExposureMm'],.5)
   self.assertLess(j['wearBeforeHeadExposureMm']-.6,0) # wear control exposes head
 def test_fasteners_clear_drilled_material(self):
  for n,s,k in self.parts[len(self.base):]:
   if not (n.endswith('shaft') or n.endswith('head')):continue
   for fn,fs,fk in self.parts:
    if 'UHMW slide pad' in fn or 'lift carriage plate' in fn or fk.startswith('original-machined'):
     self.assertLess(abs(q.lower.volume(s,fs)),1e-5,(n,fn))
 def test_undrilled_pad_blocks_screw(self):
  n,s,k=next(p for p in self.parts[len(self.base):] if p[0].endswith(' pad 235.4 shaft'))
  self.assertGreater(sum(max(0,q.lower.volume(s,fs)) for fn,fs,fk in self.base if 'UHMW slide pad' in fn),10)
 def test_guide_clearance_and_shift_control(self):
  from OCP.BRepExtrema import BRepExtrema_DistShapeShape
  for side in [-1,1]:
   guide=next(s for n,s,k in self.base if n=='intake candidate stationary aluminum guide '+str(side))
   for n,s,k in self.base:
    if not n.startswith('intake candidate UHMW slide pad ('+str(side)+','):continue
    self.assertAlmostEqual(BRepExtrema_DistShapeShape(s,guide).Value(),.15,places=5)
    xx=-1 if n.endswith('-1)') else 1
    self.assertGreater(q.lower.volume(q.h.move(s,-xx*.2,0,0),guide),1)
 def test_pad_backing_and_foot_contact(self):
  from OCP.BRepExtrema import BRepExtrema_DistShapeShape
  parts={n:s for n,s,k in self.parts}
  for j in self.joints:
   for target in [j['pad'],j['carriage']]:
    self.assertLess(BRepExtrema_DistShapeShape(parts[j['name']],parts[target]).Value(),1e-6)
    self.assertLess(abs(q.lower.volume(parts[j['name']],parts[target])),1e-6)
 def test_receipt_sources_and_pose_inventory(self):
  import json,hashlib
  report=json.loads((ROOT/'cad/assembly/evidence/slide-pad-attachments.json').read_text())
  self.assertEqual(report['sourcePins']['assembly'],q.assembly.input_pins())
  self.assertEqual(set(report['sourcePins']['dependencies']),{'cad/assembly/slide_pad_attachments.py','cad/assembly/upper_carriage_connections.py','cad/assembly/lower_support_mounts.py','cad/assembly/audit_intake_motion.py','cad/electrical/fasteners.py'})
  for p,digest in report['sourcePins']['dependencies'].items():self.assertEqual(hashlib.sha256((ROOT/p).read_bytes()).hexdigest(),digest)
  self.assertEqual(len(report['samples']),15)
  self.assertFalse(any(r['hits'] for r in report['samples']))
if __name__=='__main__':unittest.main()

import pathlib,sys,unittest,inspect,textwrap,ast,json,copy
R=pathlib.Path(__file__).resolve().parents[1];sys.path[:0]=[str(R/'sim/integrated'),str(R/'tests')]
import coupled_acquisition as reference,electrical_port,fast_coupled_acquisition as fast
import test_electrical_midpoint as previous

class ExplicitFast(unittest.TestCase):
 def test_factory_exact_reviewed_function(self):
  source=(R/'sim/integrated/electrical_midpoint.py').read_text();tree=ast.parse(source);node=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='make_port')
  old=ast.get_source_segment(source,node);new=textwrap.dedent(inspect.getsource(electrical_port.make_port)).strip();self.assertEqual(old,new)
 def test_step_only_port_block_differs(self):
  old=inspect.getsource(reference.CoupledAcquisition.step)
  block="  def port(v):\n   p=self.preview(dt,drive_demands,pinch_demands,v,normal_loads,modes,geometry=cached_geometry);return dict(busPowerW=p['a']['busPowerW']+p['b']['electrical']['busPowerW'])"
  self.assertEqual(old.count(block),1)
  expected=old.replace(block,'  port=make_port(self,dt,drive_demands,pinch_demands,normal_loads,modes,geometry=cached_geometry)')
  self.assertEqual(expected,inspect.getsource(fast.FastCoupledAcquisition.step))
 def test_production_modules_have_no_dynamic_code(self):
  for path in ['electrical_port.py','fast_coupled_acquisition.py']:
   tree=ast.parse((R/'sim/integrated'/path).read_text())
   for n in ast.walk(tree):
    if isinstance(n,ast.Call) and isinstance(n.func,ast.Name):self.assertNotIn(n.func.id,['eval','exec','compile'])
 def test_exact_step_and_atomic_failure(self):
  a=reference.CoupledAcquisition(previous.D,previous.C,previous.contacts,previous.initial());b=fast.FastCoupledAcquisition(previous.D,previous.C,previous.contacts,previous.initial())
  for modes in [('drive','drive'),('coast','brake')]:
   for _ in range(4):self.assertEqual(a.step(.001,(.2,.2),(.12,.12),(220,220),modes=modes),b.step(.001,(.2,.2),(.12,.12),(220,220),modes=modes))
  b.geometry=lambda q:dict(status='UNKNOWN_CONTACT',contacts=[]);before=copy.deepcopy((b.drive.state,b.note,b.energy))
  with self.assertRaises(reference.DomainExit):b.step(.001,(.2,.2),(.12,.12),(220,220))
  self.assertEqual(before,(b.drive.state,b.note,b.energy))
 def test_fresh_receipt_inventory_and_domain(self):
  import verify_fast_coupled as runner,hashlib
  r=json.loads((R/'sim/integrated/fast-coupled-evidence.json').read_text())
  _,_,pins=runner.run_free_planar.load(R/'sim/acquisition/free-obstacles.json')
  self.assertEqual(set(r['inputs']),set(pins)|set(runner.FILES))
  for name,pin in r['inputs'].items():self.assertEqual(hashlib.sha256((R/name).read_bytes()).hexdigest(),pin)
  self.assertEqual(r['traceSha256'][0],r['traceSha256'][1]);self.assertTrue(r['exactEveryRowEqual']);self.assertEqual(r['steps'],752)
  self.assertEqual(r['status'],'UNKNOWN_DOMAIN: OUTSIDE_BOUNDED_PLANAR_EXPERIMENT');self.assertLess(r['maxAbsEnergyResidualJ'],.002)
if __name__=='__main__':unittest.main()

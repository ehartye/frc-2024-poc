import unittest,pathlib,sys,copy,json
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/pressure'))
import checkpoint as c
class CheckpointTest(unittest.TestCase):
 def step(self,p):return p.step(.001,[.2,.2],[.12,.12],[p.drive.c['massKg']*9.80665/2]*2)
 def test_split_byte_parity(self):
  p,_=c.factory.setup()
  for _ in range(3):self.step(p)
  record=c.export(p);q=c.restore(json.loads(json.dumps(record)))
  for _ in range(4):self.assertEqual(c.digest(self.step(p)),c.digest(self.step(q)))
  self.assertEqual(c.export(p),c.export(q))
 def test_contact_phase_split_byte_parity(self):
  p,origin=c.from_pressure_report(R/'sim/pressure/acquisition-entry-evidence.json')
  self.assertEqual(len(origin),64);self.assertGreater(c.diagnostics(p)['potentialJ'],0.)
  self.step(p);q=c.restore(json.loads(json.dumps(c.export(p))))
  for _ in range(2):self.assertEqual(c.digest(self.step(p)),c.digest(self.step(q)))
  self.assertEqual(c.export(p),c.export(q))
 def test_reject_identity_shapes_and_diagnostics(self):
  p,_=c.factory.setup();r=c.export(p)
  mutations=[lambda x:x.update(schema='legacy/1'),lambda x:x['identity']['pressure'].update(kArea=1e8),lambda x:x['identity']['pressure'].update(resolution=[512,512]),lambda x:x['identity']['pressure'].update(materialPhase='other'),lambda x:x['state']['note'].update(p=[0]),lambda x:x['state']['note'].update(spin=True),lambda x:x['state'].update(timeS=-1),lambda x:x['state'].update(initialEnergyJ=1),lambda x:x['diagnostics'].update(potentialJ=1)]
  for f in mutations:
   bad=copy.deepcopy(r);f(bad);bad['digest']=c.digest({k:v for k,v in bad.items() if k!='digest'})
   with self.assertRaises(ValueError):c.restore(bad)
 def test_corruption_and_purity(self):
  p,_=c.factory.setup();before=copy.deepcopy((p.drive.state,p.note,p.energy,p.t));r=c.export(p);self.assertEqual(before,(p.drive.state,p.note,p.energy,p.t));r['state']['note']['p'][0]+=.01
  with self.assertRaises(ValueError):c.restore(r)
if __name__=='__main__':unittest.main()

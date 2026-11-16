import pathlib,sys,json,unittest,math
R=pathlib.Path(__file__).resolve().parents[1];H=R/'sim/component-launcher';sys.path.insert(0,str(H));from transfer_plant import SixDrivePlant,require_clear_transfer
class SixDriveTests(unittest.TestCase):
 def setUp(self):
  self.c=json.loads((H/'config.json').read_text());self.c.update(pinchRadiusM=12*6.35/1000/(2*math.pi),pinchInertiaKgM2=.00003,pinchNormalEachN=10.,pinchFriction=.6)
  self.rows=[{'advanceMm':x,'pinchOverlapMm3':1.,'launcherOverlapMm3':1.} for x in range(0,481,10)]
 def test_current_real_geometry_rejected(self):
  with self.assertRaisesRegex(ValueError,'collision'):require_clear_transfer(json.loads((H/'transfer-evidence.json').read_text()))
 def test_shared_bus(self):
  p=SixDrivePlant(self.c,self.rows);r=p.step([.3,.3,.8,.8,.8,.8]);self.assertLess(r['busV'],12);self.assertEqual(len(r['phaseCurrentA']),6);self.assertTrue(all(0<=i<=40 for i in r['phaseCurrentA']))
 def test_virtual_hold_no_teleport(self):
  p=SixDrivePlant(self.c,self.rows)
  for _ in range(100):r=p.step([.3,.3,.8,.8,.8,.8])
  self.assertEqual(p.x,0);self.assertEqual(p.v,0);r=p.step([.3,.3,.8,.8,.8,.8],release=True);self.assertFalse(r['held']);self.assertFalse(r['acceptance'])
 def test_off_preserves_momentum(self):
  c=dict(self.c,pinchNormalEachN=0.,normalEachN=0.);p=SixDrivePlant(c,self.rows);p.w=[100.]*6;p.step([0.]*6);self.assertEqual(p.w,[100.]*6)
 def test_invalid_commands(self):
  for commands in [[True]*6,[float('nan')]*6,[1]*5,[2]*6]:
   with self.assertRaises(ValueError):SixDrivePlant(self.c,self.rows).step(commands)
 def test_invalid_configuration(self):
  for k,v in [('stepS',0),('pieceMassKg',True),('pinchFriction',float('nan')),('sourceResistanceOhm',-1)]:
   with self.assertRaises(ValueError):SixDrivePlant(dict(self.c,**{k:v}),self.rows)
 def test_free_body_energy_and_momentum(self):
  p=SixDrivePlant(dict(self.c,pinchNormalEachN=0.,normalEachN=0.),self.rows)
  for _ in range(100):r=p.step([0.]*6,release=True)
  self.assertLess(p.v,0);self.assertLess(abs(r['energyJ']['residual']),1e-10)
 def test_energy_step_refinement(self):
  results=[]
  for scale in [1,2]:
   p=SixDrivePlant(dict(self.c,stepS=self.c['stepS']/scale),self.rows)
   for _ in range(1000*scale):r=p.step([.3,.3,.8,.8,.8,.8])
   results.append(abs(r['energyJ']['residual']));self.assertGreaterEqual(r['energyJ']['slipLoss'],0)
  self.assertLess(results[1],results[0]*.6)
if __name__=='__main__':unittest.main()

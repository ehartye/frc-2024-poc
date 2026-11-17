import pathlib,sys,unittest,json,copy
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/pressure'))
import braking_headroom as m
class Headroom(unittest.TestCase):
 def test_actual_model_decay_pure_and_refinement(self):
  config=json.loads((R/'sim/drivetrain/config.json').read_text());p=m.fresh.c.factory.r.drive_source.DrivePlant(config);p.state['vx']=.25;p.state['wheel']=[.25/config['wheelRadiusM']]*2;s=copy.deepcopy(p.state)
  a=m.project(config,s,.001);b=m.project(config,s,.0005)
  self.assertEqual(s,p.state);self.assertEqual(a['outcome'],'LOW_SPEED_OBSERVATION');self.assertGreater(a['travelXM'],0);self.assertLess(a['travelXM'],.1);self.assertLess(abs(a['travelXM']-b['travelXM']),1e-5);self.assertLess(b['energyJ']['kinetic'],b['initialKineticJ'])
if __name__=='__main__':unittest.main()

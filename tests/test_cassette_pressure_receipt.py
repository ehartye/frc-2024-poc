import json,pathlib,sys,unittest,hashlib
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/pressure'))
import run_cassette_coupling as runner
class Receipt(unittest.TestCase):
 def setUp(self):self.r=json.loads((R/'sim/pressure/cassette-coupling-evidence.json').read_text())
 def test_exact_inputs(self):
  self.assertEqual(self.r['inputs'],runner.inventory());self.assertFalse(self.r['acceptance']);self.assertEqual(len(self.r['rows']),5)
 def test_independent_states_refinement_and_controls(self):
  rows=self.r['rows'];a=rows['holding_input'];b=rows['holding_input_half']
  self.assertLess(a['final']['y'],a['initial']['y']);self.assertNotEqual(a['final']['u'],a['initial']['u']);self.assertNotEqual(a['final']['theta'],a['initial']['theta'])
  self.assertLess(b['maxResidualJ'],a['maxResidualJ'])
  z=rows['zero_spring_preload'];zh=rows['zero_spring_preload_half'];self.assertEqual(z['initialPorts']['springForce'],0);self.assertLess(z['initialPorts']['noteForceY'],0);self.assertLess(zh['maxResidualJ'],z['maxResidualJ'])
  self.assertEqual(rows['torque_removed']['final']['actuator_work'],0);self.assertNotEqual(rows['torque_removed']['final']['thetadot'],a['final']['thetadot'])
 def test_grid_artifact_and_constraints_retained(self):
  grids=self.r['gridControls'];self.assertEqual([q['resolution'] for q in grids],[512,1024,1024]);self.assertGreater(abs(grids[-1]['result']['noteConstraintMoment'][2]),1e-6)
  for row in self.r['rows'].values():
   for item in row['rows']:
    self.assertEqual(item['ports']['constraintPowerW'],0);self.assertIsNone(item['ports']['electricalCurrentA']);self.assertLess(abs(item['ports']['powerResidualW']),1e-12)
if __name__=='__main__':unittest.main()

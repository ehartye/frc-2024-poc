import unittest,pathlib,sys,json
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/pressure'))
import run_policy_transition as r
class PolicyReceipts(unittest.TestCase):
 def test_exact_inventory_and_identical_physics_controls(self):
  p,pins=r.load();base=R/'sim/pressure/policy-evidence';reports=[json.loads((base/(s+'.json')).read_text()) for s in ['nominal','stale','no-pinch']]
  for e in reports:
   self.assertEqual(e['inputs'],pins);self.assertEqual(e['startCheckpoint'],r.c.export(p));self.assertEqual(e['outcome'],'UNKNOWN_DOMAIN');self.assertEqual(len(e['trace']),84);self.assertAlmostEqual(e['endCheckpoint']['state']['timeS'],.764,places=12)
  physical=[[{k:v for k,v in row.items() if k!='command'} for row in e['trace']] for e in reports];self.assertEqual(physical[0],physical[1]);self.assertEqual(physical[0],physical[2]);self.assertEqual(reports[1]['events'][0]['command']['state'],'OBSERVATION_FAULT');self.assertEqual(reports[0]['events'][0]['command']['state'],'REGULATE')
  b=json.loads((base/'boundaries.json').read_text());self.assertTrue(all(r.c.factory.r.sha(R/f)==h for f,h in b['inputs'].items()))
  for e in b['rows']:self.assertEqual(e['policyReportSha256'],r.c.factory.r.sha(base/(e['scenario']+'.json')));self.assertLess(e['rejectedGeometryQuery']['relativeRobotXY'][0],.178)
if __name__=='__main__':unittest.main()

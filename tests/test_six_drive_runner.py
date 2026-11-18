import pathlib,sys,json,unittest,copy
R=pathlib.Path(__file__).resolve().parents[1];H=R/'sim/component-launcher';sys.path.insert(0,str(H));import run_six_drive as q
class RunnerTest(unittest.TestCase):
 def test_baseline_native_contacts_rejected(self):
  with self.assertRaisesRegex(ValueError,'Stale'):q.validate_reports(H/'connected-transfer-evidence.json',H/'connected-native-wheel-evidence.json')
 def test_synthetic_power_off_keeps_state(self):
  c=json.loads((H/'six-drive-config.json').read_text());c.update(stepS=.0002,maxTimeS=.4,readyRpm=10,pinchReadySurfaceMps=.01,normalEachN=1.,pinchNormalEachN=1.,pitchDegrees=0)
  rows=[dict(advanceMm=x,pinchOverlapMm3=1.,launcherOverlapMm3=1.) for x in range(0,481,10)]
  r=q.simulate(c,rows,'power-off');self.assertIsNotNone(r['releaseTimeS']);self.assertFalse(r['acceptance'])
  off=[x for x in r['trace'] if max(x['duties'])==0]
  self.assertTrue(off);self.assertTrue(any(w!=0 for w in off[0]['omegaRadps']))
 def test_unknown_scenario_rejected(self):
  with self.assertRaises(ValueError):q.simulate({},[],'invented')
if __name__=='__main__':unittest.main()

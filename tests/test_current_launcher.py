import pathlib,json,hashlib,sys,unittest,math,tempfile
R=pathlib.Path(__file__).resolve().parents[1];H=R/'sim/component-launcher';sys.path.insert(0,str(H));import run_current as run
class CurrentGeometry(unittest.TestCase):
 def setUp(self):self.g=json.loads((H/'current-geometry.json').read_text());self.e=json.loads((H/'evidence.json').read_text())
 def test_all_cad_pins(self):
  for f,h in self.g['inputs'].items():self.assertEqual(hashlib.sha256((R/f).read_bytes()).hexdigest(),h,f)
 def test_run_pins(self):
  for f,h in self.e['inputs'].items():self.assertEqual(hashlib.sha256((R/f if f.startswith('scripts/') else H/f).read_bytes()).hexdigest(),h,f)
 def test_current_radius_and_pose(self):
  self.assertAlmostEqual(self.g['wheelRadiusMm'],48.26);self.assertAlmostEqual(self.g['shooter']['pivot'][0],.188);self.assertAlmostEqual(self.g['handoffFromStationMm'],-220)
 def test_finite_occupancy(self):
  rows=self.g['contactSampling']['two lane candidate'];self.assertEqual(len(rows),49)
  self.assertEqual([r['noteCenterFromWheelStationMm'] for r in rows if r['summedNominalOverlapMm3']>1e-6],list(range(-80,81,10)))
  for r in rows:self.assertAlmostEqual(sum(r['byTreadMm3'].values()),r['summedNominalOverlapMm3']);self.assertTrue(all(x>=0 and math.isfinite(x) for x in r['byTreadMm3'].values()))
 def test_reversal_not_a_release(self):
  for c in self.e['cases']:
   if c['plant']['outcome']!='passed contact-table boundary':self.assertIn('releaseNotRegistered',c);self.assertNotIn('registeredRelease',c);self.assertNotIn('thinApertureScreen',c)
 def test_reversal_halfstep(self):
  a=self.e['cases'][0]['plant'];b=self.e['cases'][-1]['plant'];self.assertEqual(a['outcome'],'reversed before exit');self.assertEqual(a['outcome'],b['outcome']);self.assertLess(abs(a['endXFromStationM']-b['endXFromStationM']),1e-5)
 def test_forward_transform(self):
  c=next(x for x in self.e['cases'] if x['name']=='faster handoff');p=c['plant']['lastPositiveEngagementSample'];local=c['releaseRobotM'];self.assertAlmostEqual(local[0],.188+(.2+p['xFromStationM'])*.5);self.assertFalse(self.e['wholeRobotSimulation']);self.assertFalse(self.e['scoreAccepted'])
 def test_gate_controls(self):
  for outcome in ['reversed before exit','timeout','not ready']:
   self.assertFalse(run.can_register({'outcome':outcome,'lastPositiveEngagementSample':{'speedMps':8}}))
  self.assertFalse(run.can_register({'outcome':'passed contact-table boundary','lastPositiveEngagementSample':None}))
 def test_fresh_directory_required(self):
  with tempfile.TemporaryDirectory() as p:
   with self.assertRaisesRegex(ValueError,'Fresh'):run.run(p)
if __name__=='__main__':unittest.main()

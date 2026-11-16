import json
import importlib.util,pathlib,unittest,math
P=pathlib.Path(__file__).resolve().parent
s=importlib.util.spec_from_file_location('electrical',P/'build_electrical.py');m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
class ElectricalPackageTests(unittest.TestCase):
 def test_inventory_and_units(self):
  d=m.load();self.assertEqual(d['units'],'mm');self.assertEqual(len({x['id'] for x in d['components']}),len(d['components']))
  for c in d['components']:
   self.assertTrue(all(math.isfinite(x) and x>0 for x in c['sizeMm']))
   if c['role']=='electrical':self.assertTrue(c['sourceUrl'].startswith('https://'))
 def test_no_solid_overlap(self):self.assertEqual(m.report(m.load())['solidEnvelopeConflicts'],[])
 def test_unknown_mass_not_complete(self):
  q=m.report(m.load());self.assertAlmostEqual(q['knownComponentMassKg'],8.34026319777);self.assertNotIn('roborio',q['unknownMassIds']);self.assertFalse(q['fullRobotMassOrCoG'])
 def test_native_dimensions_and_channels(self):
  d=m.load();c=next(c for c in d['components'] if c['id']=='roborio');self.assertEqual(c['sourceSha256'],'6e19ed3e825c8c521f57ec510eda860556ac422f134dc584b8342ca4d94be791');self.assertAlmostEqual(c['sizeMm'][0],146.1518148290822)
  self.assertEqual(d['powerAllocation']['driveAllocation'],4);self.assertFalse(d['powerAllocation']['simultaneousCurrentBudgetEstablished'])
 def test_native_validity(self):
  p=m.build();self.assertEqual(len(p),55)
  for _,shape,_ in p:self.assertTrue(m.BRepCheck_Analyzer(shape).IsValid())
 def test_native_crosspackage_record(self):
  import hashlib
  q=json.loads((P/'generated/crosscheck.json').read_text());self.assertTrue(q['nativeRailReplacement']);self.assertEqual(q['crossOverlaps'],[]);self.assertEqual(q['internalOverlaps'],[]);self.assertGreater(q['shiftedOneMmBoltOverlap'][0]['volumeMm3'],.1)
  for n in ['manifest.json','mounts.py','build_electrical.py','check_drivetrain.py']:self.assertEqual(q['inputs'][n],hashlib.sha256((P/n).read_bytes()).hexdigest())
if __name__=='__main__':unittest.main()

import unittest,pathlib,sys,json
from unittest.mock import patch
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/acquisition'))
import run_free_planar as r
class PlanarReceiptTest(unittest.TestCase):
 def test_complete_current_sources_and_unchanged_offset_controls(self):
  o,f,p=r.load(R/'sim/acquisition/free-obstacles.json');self.assertEqual(len(o['obstacles']),744)
  for y in [-5,5]:self.assertTrue(any(x['lateralOffsetMm']==y and any(h['contactGroup']=='unresolved' for h in x['hits']) for x in f['rows']))
 def test_changed_source_rejected(self):
  real=r.sha
  with patch.object(r,'sha',side_effect=lambda p:'changed' if p.name=='intake_components.py' else real(p)):
   with self.assertRaises(ValueError):r.load(R/'sim/acquisition/free-obstacles.json')
 def test_evidence_inventory_and_binding(self):
  report=json.loads((R/'sim/acquisition/free-planar-evidence.json').read_text());self.assertEqual(report['inputs'],r.load(R/'sim/acquisition/free-obstacles.json')[2]);self.assertEqual(len(report['cases']),14)
  cases={c['name']:c for c in report['cases']};self.assertFalse(report['acceptance'])
  for c in cases.values():
   self.assertIsNotNone(c['stoppedAttempt']);self.assertLessEqual(c['final']['t'],c['stoppedAttempt']['timeS']);self.assertFalse(c['acceptance'])
  self.assertEqual(cases['aabb-control-right']['final']['relativeRobotXY'][1],.005)
  self.assertLess(abs(cases['stiff-right']['final']['relativeRobotXY'][1]),.001)
  self.assertLess(abs(cases['right']['final']['relativeRobotXY'][1]-cases['right-half']['final']['relativeRobotXY'][1]),1e-6)
  self.assertLess(abs(cases['stiffer-right']['final']['relativeRobotXY'][1]-cases['stiffer-right-half']['final']['relativeRobotXY'][1]),1e-6)
if __name__=='__main__':unittest.main()

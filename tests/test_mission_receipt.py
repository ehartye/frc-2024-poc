import pathlib,unittest,tempfile,importlib.util,subprocess,sys,xml.etree.ElementTree as ET
ROOT=pathlib.Path(__file__).resolve().parents[1]/'experiments/a2-campaign'
s=importlib.util.spec_from_file_location('runner',ROOT/'scripts/run-a2-mission.py');r=importlib.util.module_from_spec(s);s.loader.exec_module(r)
class MissionReceipt(unittest.TestCase):
 def xml(self,token):
  t=ET.Element('testsuite',tests=str(len(r.TESTS)),failures='0',errors='0',skipped='0')
  for name in sorted(r.TESTS):ET.SubElement(t,'testcase',name=name)
  ET.SubElement(t,'system-out').text='MISSION_RUN_ID='+token+'\n'+'\n'.join('A2_RESULT scenario='+s for s in r.SCENARIOS)
  return ET.tostring(t)
 def test_canonical_geometry_drift_rejected(self):
  with tempfile.TemporaryDirectory() as d:
   root=pathlib.Path(d);(root/'cad').mkdir();(root/'mission').mkdir()
   for name in ['cycle-contract-A2.json','a2_kinematics.py']:
    (root/'cad'/name).write_bytes(b'one');(root/'mission'/name).write_bytes(b'one')
   r.check_canonical_geometry(root);(root/'cad'/'cycle-contract-A2.json').write_bytes(b'changed')
   with self.assertRaisesRegex(ValueError,'differ'):r.check_canonical_geometry(root)
 def test_stale_run_token_rejected(self):
  with self.assertRaises(ValueError):r.validate_xml(self.xml('old'),'new')
 def test_missing_scenario_rejected(self):
  with self.assertRaises(ValueError):r.validate_xml(self.xml('same').replace(b'A2_RESULT scenario=STRICT_12V',b''),'same')
 def test_complete_matching_xml_inventory(self):self.assertEqual(len(r.validate_xml(self.xml('same'),'same')),7)
 def test_existing_output_not_reused(self):
  with tempfile.TemporaryDirectory() as d:
   with self.assertRaisesRegex(ValueError,'fresh output'):r.run(d,'missing','missing')
 def test_geometry_generated_from_pinned_contract(self):subprocess.run([sys.executable,str(ROOT/'scripts/generate-mission-geometry.py'),'--check'],check=True)
if __name__=='__main__':unittest.main()

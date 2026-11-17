import unittest,pathlib,sys,json
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/pressure'))
import verify_corrected_lineage as v
class CorrectedLineage(unittest.TestCase):
 def test_fresh_reconstruction_and_all_rows(self):
  self.assertEqual(v.verify(),json.loads((R/'sim/pressure/corrected-lineage.json').read_text()))
if __name__=='__main__':unittest.main()

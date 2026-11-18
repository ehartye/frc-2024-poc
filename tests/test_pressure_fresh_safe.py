import pathlib,sys,tempfile,unittest
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'sim/pressure'))
import run_fresh_rest_safe as wrapper
class Entry(unittest.TestCase):
 def test_nested_directory_and_exact_physical_output(self):
  with tempfile.TemporaryDirectory() as d:
   root=pathlib.Path(d);a=wrapper.frozen.run(root/'reference.json',duration=.002);b=wrapper.run(root/'new'/'nested'/'candidate.json',duration=.002)
  a.pop('runtimeSeconds');b.pop('runtimeSeconds');self.assertEqual(a,b)
if __name__=='__main__':unittest.main()

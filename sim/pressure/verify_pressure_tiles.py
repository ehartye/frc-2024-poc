import argparse,hashlib,importlib.util,json,pathlib,sys,unittest
p=argparse.ArgumentParser();p.add_argument('--project',type=pathlib.Path,required=True);p.add_argument('--output',type=pathlib.Path,required=True);a=p.parse_args();assert not a.output.exists()
sys.path.insert(0,str(a.project/'sim/pressure'))
import pressure_tiled_geometry as tiled
spec=importlib.util.spec_from_file_location('original_geometry_tests',a.project/'tests/test_pressure_fast_geometry.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);module.fast=tiled
result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromModule(module))
assert result.wasSuccessful()
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
receipt={'schema':'pressure-tiled-culling-tests/1','testsRun':result.testsRun,'passed':True,'candidateSha256':sha(pathlib.Path(tiled.__file__)),'testSourceSha256':sha(a.project/'tests/test_pressure_fast_geometry.py'),'scriptSha256':sha(pathlib.Path(__file__)),'limits':['Existing reference parity tests executed against the additive tiled candidate','No trajectory or hardware qualification from these tests alone']}
a.output.write_text(json.dumps(receipt,indent=2)+'\n',encoding='utf-8',newline='\n')

"""Check the current kernel's negative-command boundary without integrating."""
import argparse,copy,hashlib,json,pathlib,sys
p=argparse.ArgumentParser();p.add_argument('--project',type=pathlib.Path,required=True);p.add_argument('--output',type=pathlib.Path,required=True);a=p.parse_args();assert not a.output.exists()
sys.path.insert(0,str(a.project/'sim/pressure'))
import run_fresh_rest_controller as fresh
model,controller,pins,bus,ancestry=fresh.load();before=copy.deepcopy((model.drive.state,model.note,model.energy,model.t));reason=None
try:model.step(.001,[0.,0.],[-.0034,-.0034],[model.drive.c['massKg']*9.80665/2]*2)
except ValueError as e:reason=str(e)
assert reason=='pinch duties' and before==(model.drive.state,model.note,model.energy,model.t)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
r={'schema':'pressure-actuation-domain-control/1','acceptance':False,'scriptSha256':sha(pathlib.Path(__file__)),'kernelSha256':sha(a.project/'sim/pressure/pressure_acquisition.py'),'rejectedDuties':[-.0034,-.0034],'reason':reason,'physicalStateUnchanged':True,'limits':['Invalid-input guard checked before integration; not a transport run or hardware limitation']}
a.output.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps(r))

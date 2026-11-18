"""Verify invalid commands and unsupported native-domain proposals are atomic."""
import argparse,copy,hashlib,json,pathlib,sys
p=argparse.ArgumentParser()
for n in ['project','binding','output']:p.add_argument('--'+n,type=pathlib.Path,required=True)
a=p.parse_args();assert not a.output.exists();sys.path[:0]=[str(a.project/'sim/pressure'),str(a.project/'sim/integrated')]
from native_signed_pressure import NativeSignedPressure
from signed_pressure_acquisition import DomainExit
load=lambda p:json.loads(p.read_bytes());dc=load(a.project/'sim/drivetrain/config.json');nc=load(a.project/'sim/acquisition/free-planar-config.json')['model'];cad=load(a.project/'cad/assembly/intake_components.json');binding=load(a.binding);rows=[]
for case in ['duty-range','nonfinite-duty','floor-corridor','nonplanar-yaw']:
 m=NativeSignedPressure(dc,nc,cad,binding);duties=[0.,0.]
 if case=='duty-range':duties=[-1.01,0.]
 if case=='nonfinite-duty':duties=[float('nan'),0.]
 if case=='floor-corridor':m.note['p'][0]=.177
 if case=='nonplanar-yaw':m.note['yaw']=.0001
 snapshot=lambda:copy.deepcopy(dict(drive=m.drive.state,note=m.note,ledger=m.energy,time=m.t,initialEnergy=m.initial_energy))
 before=snapshot()
 try:m.step(.0005,[0.,0.],duties,[dc['massKg']*9.80665/2]*2,modes=['brake','brake'])
 except (ValueError,DomainExit) as e:assert snapshot()==before;rows.append(dict(case=case,rejected=True,reason=str(e),physicalMutation=False))
 else:raise AssertionError('invalid input accepted')
r=dict(schema='native-signed-atomic-rejection/1',passed=True,rows=rows,acceptance=False,scriptSha256=hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest(),limits=['Prescribed invalid input/state diagnostics; not attained physical states'])
a.output.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps(r,indent=2))

"""Check coupled mechanics work identity and atomic envelope rejection."""
import argparse,copy,hashlib,json,pathlib,sys
import numpy as np
p=argparse.ArgumentParser()
for name in ['project','source','binding','contract','trajectory','output']:p.add_argument('--'+name,type=pathlib.Path,required=True)
a=p.parse_args();assert not a.output.exists();sys.path[:0]=[str(a.project/f'sim/{x}') for x in ['pressure','integrated','drivetrain','actuation','acquisition']]
import native_pressure_lift as lift
load=lambda p:json.loads(p.read_bytes());source=load(a.source);trajectory=load(a.trajectory);contract=load(a.contract);binding=load(a.binding);dc=load(a.project/'sim/drivetrain/config.json');nc=load(a.project/'sim/acquisition/free-planar-config.json')['model'];cad=load(a.project/'cad/assembly/intake_components.json');ac=load(a.project/'sim/actuation/config.json');m=lift.NativePressureLift(source['final'],dc,nc,cad,binding,ac,contract);cases=[]
for name,s,cmd,bus in [('entry',trajectory['initial'],trajectory['events'][0]['command'],12.),('floorUnloaded',trajectory['final'],trajectory['events'][-1]['command'],trajectory['trace'][-1]['busV'])]:
 r=m.rates(s,bus,cmd,{});power=r['powerW']-sum(r['energyRates'].values());errors=[]
 for eps in [1e-7,1e-8]:
  plus=m.shifted(s,r,eps);minus=m.shifted(s,r,-eps);derivative=(m.mechanical(plus,m.contact(plus,{}))-m.mechanical(minus,m.contact(minus,{})))/(2*eps);errors.append(abs(derivative-power))
 assert errors[-1]<1e-5,(name,errors,power)
 terms=m.act.terms([s['chassis']['x'],s['zPivot'],s['theta']],[s['chassis']['vx'],s['vPivot'],s['thetaRate']]);matrix=np.array(terms['massMatrix']);matrix[0,0]+=38;eigenvalues=np.linalg.eigvalsh(matrix);assert min(eigenvalues)>0
 cases.append(dict(name=name,workResidualW=errors,finiteDifferenceStepsS=[1e-7,1e-8],massMatrixEigenvalues=eigenvalues.tolist(),pressureWorkResidualW=r['contact']['q']['workResidualW']))
m.s['theta']=.021;before=copy.deepcopy(m.s)
try:m.step(.0005,trajectory['events'][0]['command'])
except lift.LiftDomainError as e:assert m.s==before;rejection=str(e)
else:raise AssertionError('unsupported envelope accepted')
r=dict(schema='native-lift-work-identity/1',passed=True,scriptSha256=hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest(),sourceTrajectorySha256=hashlib.sha256(a.trajectory.read_bytes()).hexdigest(),cases=cases,atomicRejection=rejection,physicalMutation=False,acceptance=False,limits=['Directional mechanical work check on declared uncalibrated model, not native mass or hardware calibration'])
a.output.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps(r,indent=2))

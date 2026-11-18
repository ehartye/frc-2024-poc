import argparse,copy,hashlib,json,pathlib,sys,time
p=argparse.ArgumentParser()
for n in ['project','source','binding','contract','output']:p.add_argument('--'+n,type=pathlib.Path,required=True)
a=p.parse_args();sys.path[:0]=[str(a.project/f'sim/{x}') for x in ['pressure','integrated','drivetrain','actuation','acquisition']];sys.path.insert(0,str(pathlib.Path(__file__).parent))
import native_static_cycle as model
load=lambda p:json.loads(p.read_bytes());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();s=load(a.source);c=load(a.contract);dc=load(a.project/'sim/drivetrain/config.json');nc=load(a.project/'sim/acquisition/free-planar-config.json')['model'];cad=load(a.project/'cad/assembly/intake_components.json');ac=load(a.project/'sim/actuation/config.json');binding=load(a.binding);assert s['outcome']=='MODEL_DOMAIN_EXIT' and s['reason']=='FRICTION_ITERATION_LIMIT';assert not a.output.exists();rows=[]
for budget in [1000,10000]:
 m=model.NativeStaticCycle(dc,nc,cad,binding,ac,c,mu=s['mu']);m.s=copy.deepcopy(s['final']);m.ledger=copy.deepcopy(m.s['ledger']);m.solver_iterations=budget;before=copy.deepcopy(m.s);start=time.perf_counter()
 try:r=m.step(s['stepS'],s['events'][-1]['command']);row=dict(budget=budget,accepted=True,friction=r['friction'],state=r['state'],energyJ=r['energyJ'])
 except ValueError as e:assert m.s==before;row=dict(budget=budget,accepted=False,reason=str(e),stateUnchanged=True)
 row['runtimeSeconds']=time.perf_counter()-start;rows.append(row);print(json.dumps(row),flush=True)
result=dict(schema='native-static-cycle-rejected-step/1',producerSha256=sha(pathlib.Path(__file__)),sourceSha256=sha(a.source),sourceTimeS=s['final']['timeS'],sourceAcceptedSteps=len(s['trace']),samePhysicalLaw=True,sameTolerance=True,cases=rows,acceptance=False)
a.output.write_text(json.dumps(result,indent=2)+'\n')

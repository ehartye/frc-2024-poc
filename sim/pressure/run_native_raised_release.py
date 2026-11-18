"""Remove friction from an attained unsupported hold; preserve physical state."""
import argparse,copy,hashlib,json,pathlib,sys,time
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
p=argparse.ArgumentParser()
for name in ['project','source','binding','base-contract','lift-contract','contract','output']:p.add_argument('--'+name,type=pathlib.Path,required=True)
p.add_argument('--dt',type=float,default=.0005);a=p.parse_args();assert not a.output.exists();sys.path[:0]=[str(a.project/f'sim/{x}') for x in ['pressure','integrated','drivetrain','actuation','acquisition']]
import native_static_lift as lift
import native_dry_friction as dry
from signed_pressure_acquisition import DomainExit
load=lambda p:json.loads(p.read_bytes());s=load(a.source);lc=load(a.lift_contract);c=load(a.contract);assert a.dt in c['stepsS'] and s['retentionGatePassed'];m=lift.NativeStaticLift(s['initial'],load(a.project/'sim/drivetrain/config.json'),load(a.project/'sim/acquisition/free-planar-config.json')['model'],load(a.project/'cad/assembly/intake_components.json'),load(a.binding),load(a.project/'sim/actuation/config.json'),load(a.base_contract),friction_mu=.6,solver_tolerance=lc['solverGapToleranceJ'],solver_iterations=lc['solverMaxIterations']);m.s=copy.deepcopy(s['trace'][-2]['state']);m.ledger=copy.deepcopy(m.s['ledger']);m.step(s['stepS'],s['events'][-1]['command']);assert m.s==s['final'];initial=copy.deepcopy(m.s);m.friction_mu=0.;cmd=copy.deepcopy(s['events'][-1]['command']);a.output.mkdir();rows=[];outcome='HORIZON';reason=None;start=time.perf_counter();files={'producer':pathlib.Path(__file__),'model':pathlib.Path(lift.__file__),'friction':pathlib.Path(dry.__file__),'sourceReceipt':a.source,'binding':a.binding,'baseContract':a.base_contract,'liftContract':a.lift_contract,'contract':a.contract};pins={k:sha(p) for k,p in files.items()}
for _ in range(round(c['durationS']/a.dt)):
 before=copy.deepcopy(m.s)
 try:r=m.step(a.dt,cmd)
 except (ValueError,DomainExit) as e:assert m.s==before;outcome='MODEL_DOMAIN_EXIT';reason=str(e);break
 rows.append(r)
 if abs(r['energyJ']['residual']-s['trace'][-1]['energyJ']['residual'])>.002:outcome='ENERGY_GATE_EXIT';break
assert all(sha(p)==pins[k] for k,p in files.items());out=dict(schema='native-raised-release/1',inputs=pins,stepS=a.dt,initial=initial,final=copy.deepcopy(m.s),command=cmd,outcome=outcome,reason=reason,inheritedResidualJ=s['trace'][-1]['energyJ']['residual'],trace=rows,runtimeSeconds=time.perf_counter()-start,acceptance=False,limits=c['limits']);(a.output/'receipt.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps({k:out[k] for k in ['outcome','reason','runtimeSeconds']}))

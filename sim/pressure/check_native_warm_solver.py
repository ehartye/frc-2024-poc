import argparse,copy,hashlib,json,pathlib,sys,time
import numpy as np
p=argparse.ArgumentParser()
for n in ['project','source','binding','contract','output']:p.add_argument('--'+n,type=pathlib.Path,required=True)
a=p.parse_args();assert not a.output.exists();sys.path[:0]=[str(a.project/f'sim/{x}') for x in ['pressure','integrated','drivetrain','actuation','acquisition']];sys.path.insert(0,str(pathlib.Path(__file__).parent))
import native_static_cycle as cycle
from native_dry_friction import solve_disks,contact_jacobian
from native_warm_friction import solve_disks as warm
from shared_supply import solve_supply
load=lambda p:json.loads(p.read_bytes());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();dc=load(a.project/'sim/drivetrain/config.json');nc=load(a.project/'sim/acquisition/free-planar-config.json')['model'];cad=load(a.project/'cad/assembly/intake_components.json');ac=load(a.project/'sim/actuation/config.json');s=load(a.source);m=cycle.NativeStaticCycle(dc,nc,cad,load(a.binding),ac,load(a.contract));cmd=s['events'][-1]['command'];dt=s['stepS']
def problem(state):
 m.s=copy.deepcopy(state);m.ledger=copy.deepcopy(m.s['ledger']);cache={};con=m.contact(m.s,cache)
 def port(bus):
  d,p,a=m.electrical(m.s,bus,cmd);return {'busPowerW':d['busPowerW']+sum(x['terminalPowerW'] for x in p)+a['busPowerW']}
 supply=solve_supply({'test':port},12.,.02);r=m.rates(m.s,supply['busV'],cmd,cache,con);j,_=contact_jacobian(con['pressure'],nc['radiusM']);mass=m.mass(m.s);free=m.velocity(m.s)+dt*np.array([*r['acc'],*r['noteAccel'],r['notePitchAccel'],*r['pinchAccel']]);return j,mass,free,dt*.6*con['pressure']['normalN']
previous=solve_disks(*problem(s['trace'][-2]['state']),max_iterations=10000);args=problem(s['final']);start=time.perf_counter();cold=solve_disks(*args,max_iterations=10000);cold_time=time.perf_counter()-start;cases=[]
for name,seed in [('prior-step',previous['impulse']),('zero',np.zeros_like(previous['impulse'])),('adversarial',-100*previous['impulse'])]:
 start=time.perf_counter();r=warm(*args,max_iterations=10000,initial_impulse=seed);duration=time.perf_counter()-start;err=float(np.max(abs(cold['velocity']-r['velocity'])));assert err<1e-7;cases.append(dict(seed=name,iterations=r['iterations'],maximumVelocityDifference=err,gapJ=r['gapJ'],coneExcessNs=r['maximumConeExcessNs'],runtimeSeconds=duration))
out=dict(schema='native-warm-friction-controls/1',producerSha256=sha(pathlib.Path(__file__)),warmSolverSha256=sha(pathlib.Path('native_warm_friction.py')),sourceSha256=sha(a.source),coldIterations=cold['iterations'],coldRuntimeSeconds=cold_time,cases=cases,sameLaw=True,sameGapTolerance=True,seedIsNumericalNotPhysical=True,acceptance=False);a.output.write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))

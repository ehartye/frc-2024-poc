"""Controller-only engagement at an exact same-law pressure checkpoint."""
import copy,json,pathlib,sys,time,math
import checkpoint as c
import run_continuation as previous
from speed_approach import SpeedApproach,observe
R=c.R
SOURCE='sim/pressure/continuation-100ms-evidence.json'
FILES=['sim/pressure/run_policy_transition.py','sim/integrated/speed_approach.py','tests/test_pressure_policy_transition.py']
def load():
 report=json.loads((R/SOURCE).read_text());p,origin=c.from_pressure_report(R/'sim/pressure/acquisition-entry-evidence.json');start=c.export(p)
 pins=dict(start['identity']['inputs']);pins.update({f:c.factory.r.sha(R/f) for f in previous.FILES});pins['sim/pressure/acquisition-entry-evidence.json']=origin
 if report['schema']!='pressure-continuation/1' or report['inputs']!=pins or report['startCheckpoint']!=start or report['ancestrySha256']!=origin or report['outcome']!='HORIZON':raise ValueError('continuation lineage mismatch')
 p=c.restore(report['endCheckpoint'])
 if abs(p.t-.68)>1e-12 or report['trace'][-1]['note']!=p.note:raise ValueError('engagement endpoint mismatch')
 pins.update({f:c.factory.r.sha(R/f) for f in FILES+[SOURCE]})
 return p,pins

def engage(p,scenario):
 if scenario not in ['nominal','stale','no-pinch']:raise ValueError('scenario')
 before=c.export(p);controller=SpeedApproach(p.drive.c,goal=.185,pinch_enabled=scenario!='no-pinch')
 if c.export(p)!=before:raise AssertionError('controller initialization changed plant')
 return controller

def command(controller,p,bus,scenario):
 observation=observe(p.t,p.drive.state,p.note,bus)
 if scenario=='stale':observation['timestamp']=p.t-.051
 return controller.update(p.t,observation),observation

def run(output,scenario='nominal',duration=.1,dt=.001):
 if output.exists() or duration not in [.1,.2] or dt not in [.001,.0005]:raise ValueError('fresh bounded output required')
 p,pins=load();initial=c.export(p);controller=engage(p,scenario);rows=[];events=[];status='HORIZON';reason=None;bus=12.;started=time.perf_counter();inherited=initial['diagnostics']['residualJ']
 # Initial bus observation is the archived last solved bus, not an invented source voltage.
 bus=json.loads((R/SOURCE).read_text())['trace'][-1]['busV']
 for i in range(round(duration/dt)):
  if i%round(.01/dt)==0:
   cmd,obs=command(controller,p,bus,scenario);events.append(dict(timeS=p.t,command=copy.deepcopy(cmd),observation=obs))
  before=c.export(p)
  try:row=p.step(dt,cmd['driveDemands'],cmd['pinchDemands'],[p.drive.c['massKg']*9.80665/2]*2,modes=cmd['modes'])
  except ValueError as error:
   if c.export(p)!=before:raise AssertionError('non-atomic rejection')
   status='UNKNOWN_DOMAIN';reason=str(error);break
  bus=row['busV'];row['command']=copy.deepcopy(cmd);row['newPhaseResidualJ']=row['energyJ']['residual']-inherited;rows.append(row)
 if any(c.factory.r.sha(R/f)!=h for f,h in pins.items()):raise ValueError('changed source')
 result=dict(schema='pressure-policy-transition/1',inputs=pins,scenario=scenario,startCheckpoint=initial,endCheckpoint=c.export(p),stepS=dt,durationS=duration,controllerPeriodS=.01,events=events,trace=rows,outcome=status,reason=reason,runtimeSeconds=time.perf_counter()-started,acceptance=False,limits=['Controller initializes at.68s; physical state and cumulative energy are not reset.','Unchanged SpeedApproach with virtual exact observations; goal.185m is a controller hypothesis.','Brake is modeled motor dissipation; zero pinch command is coast, not instant stopping.','Same pressure source lineage; correction14b55ca is not silently adopted or historical receipts repinned.','No capture, retention, lift or calibrated hardware claim.'])
 output.parent.mkdir(parents=True,exist_ok=True);tmp=output.with_suffix('.tmp');tmp.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');tmp.replace(output)
 print(json.dumps(dict(scenario=scenario,outcome=status,reason=reason,timeS=p.t,note=p.note,chassis=p.drive.state,energy=c.diagnostics(p),newPhaseResidualJ=c.diagnostics(p)['residualJ']-inherited,runtime=result['runtimeSeconds'])),flush=True)
if __name__=='__main__':run(pathlib.Path(sys.argv[1]),sys.argv[2] if len(sys.argv)>2 else 'nominal')

"""Bounded continuation of the same controller from its actual .862 s endpoint."""
import copy,json,pathlib,sys,time
import run_early_continuation as prev
prior=prev.prior;c=prev.c;R=c.R
SOURCE='sim/pressure/early-continued-200ms-evidence.json'
FILES=['sim/pressure/run_braking_boundary.py','tests/test_pressure_braking_boundary.py']

def replay(config,report):
 p,controller,pins,bus,ancestry=prev.load('nominal')
 if report['schema']!='pressure-early-policy-continuation/1' or report['scenario']!='nominal' or report['inputs']!=pins or report['outcome']!='HORIZON' or report['stepS']!=.001 or report['durationS']!=.2 or report['controllerPeriodS']!=.01:raise ValueError('unsupported continuation lineage')
 if report['startCheckpoint']!=c.export(p) or report['controllerStart']!=vars(controller):raise ValueError('continuation start mismatch')
 if len(report['events'])!=20 or len(report['trace'])!=200:raise ValueError('continuation count mismatch')
 for i,event in enumerate(report['events']):
  if abs(event['timeS']-(p.t+i*.01))>1e-12:raise ValueError('controller clock discontinuity')
  if controller.update(event['timeS'],event['observation'])!=event['command']:raise ValueError('controller replay mismatch')
 if vars(controller)!=report['controllerEnd']:raise ValueError('controller state mismatch')
 for i,row in enumerate(report['trace']):
  if abs(row['timeS']-(p.t+(i+1)*.001))>1e-12 or row['command']!=report['events'][i//10]['command']:raise ValueError('trace chronology mismatch')
 return controller,pins

def load(scenario='nominal'):
 if scenario!='nominal':raise ValueError('only predeclared nominal continuation')
 raw=(R/SOURCE).read_bytes();report=json.loads(raw)
 controller,pins=replay(None,report)
 p=c.restore(report['endCheckpoint']);end=report['trace'][-1]
 if abs(p.t-.862)>1e-12 or p.t!=end['timeS'] or p.note!=end['note'] or p.drive.state!=end['chassis']:raise ValueError('endpoint mismatch')
 if {k:end['energyJ'][k] for k in p.energy}!=p.energy:raise ValueError('energy mismatch')
 if raw!=(R/SOURCE).read_bytes():raise ValueError('receipt changed')
 pins=dict(pins);pins.update({f:c.factory.r.sha(R/f) for f in FILES+[SOURCE]})
 return p,controller,pins,end['busV'],dict(source=SOURCE,sha256=pins[SOURCE],priorEvents=30,priorSteps=300)

def run(output,scenario='nominal',dt=.001,duration=.05):
 if output.exists() or dt not in [.001,.0005] or duration not in [.002,.02,.05]:raise ValueError('fresh predeclared run required')
 p,controller,pins,bus,ancestry=load(scenario); initial=c.export(p)
 controller_start=copy.deepcopy(vars(controller)); rows=[];events=[];reason=None;status='HORIZON';last=None;rejected=None
 original=p.geometry
 def query(q):
  nonlocal last
  last=dict(relativeRobotXY=list(q),status='EVALUATING')
  try:result=original(q)
  except (ValueError,prior.DomainExit) as e:
   last['status']='UNKNOWN';last['reason']=str(e);raise
  last['status']=result['status'];return result
 p.geometry=query;start=time.perf_counter();inherited=initial['diagnostics']['residualJ']
 for i in range(round(duration/dt)):
  if i%round(.01/dt)==0:
   observation=prior.observe(p.t,p.drive.state,p.note,bus)
   if scenario=='stale':observation['timestamp']=p.t-.051
   command=controller.update(p.t,observation);events.append(dict(timeS=p.t,observation=observation,command=copy.deepcopy(command)))
  before=c.export(p)
  try:row=p.step(dt,command['driveDemands'],command['pinchDemands'],[p.drive.c['massKg']*9.80665/2]*2,modes=command['modes'])
  except (ValueError,prior.DomainExit) as e:
   rejected=copy.deepcopy(last)
   if c.export(p)!=before:raise AssertionError('non-atomic physical rejection')
   reason=str(e);status='UNKNOWN_DOMAIN';break
  row['command']=copy.deepcopy(command);row['newPhaseResidualJ']=row['energyJ']['residual']-inherited;row['contactSampleTimeS']=row['timeS']-dt/2
  rows.append(row);bus=row['busV']
  if (i+1)%50==0:print(json.dumps(dict(progressTimeS=p.t,relativeX=row['relativeRobotXY'][0],chassisVx=p.drive.state['vx'])),flush=True)
 if any(c.factory.r.sha(R/f)!=h for f,h in pins.items()):raise ValueError('changed source')
 result=dict(schema='pressure-braking-boundary/1',inputs=pins,scenario=scenario,stepS=dt,durationS=duration,controllerPeriodS=.01,ancestry=ancestry,startCheckpoint=initial,endCheckpoint=c.export(p),controllerStart=controller_start,controllerEnd=copy.deepcopy(vars(controller)),events=events,trace=rows,outcome=status,reason=reason,rejectedGeometryQuery=rejected,runtimeSeconds=time.perf_counter()-start,acceptance=False,limits=['Same exact pressure law, state, momentum, clock and cumulative energy; no legacy conversion.','Controller recovered by exact previous observation/command replay, retaining fault latch.','This is continued early braking, not a new controller initialization or tuned policy.','Finite horizon or first model-domain rejection; no capture, retention, lift, collision or stopping qualification.'])
 output.parent.mkdir(parents=True,exist_ok=True);tmp=output.with_suffix('.tmp');tmp.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');tmp.replace(output)
 print(json.dumps(dict(scenario=scenario,stepS=dt,outcome=status,reason=reason,timeS=p.t,note=p.note,chassis=p.drive.state,energy=c.diagnostics(p),newPhaseResidualJ=c.diagnostics(p)['residualJ']-inherited,runtime=result['runtimeSeconds'])),flush=True)
 return result
if __name__=='__main__':run(pathlib.Path(sys.argv[1]),sys.argv[2] if len(sys.argv)>2 else 'nominal',float(sys.argv[3]) if len(sys.argv)>3 else .001,float(sys.argv[4]) if len(sys.argv)>4 else .05)

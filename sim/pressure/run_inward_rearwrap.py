"""Predeclared200ms rear-wrap observation, exact same newdomain state and controller clock."""
import copy,json,pathlib,sys,time
import run_inward_extended as prev
prior=prev.prior;c=prev.c;R=prev.R
export_domain=prev.export_domain
SOURCE='sim/pressure/inward-extended-100ms-evidence.json'
FILES=['sim/pressure/run_inward_rearwrap.py','tests/test_pressure_inward_rearwrap.py']
restore_domain=prev.restore_domain

def load(scenario='nominal'):
 if scenario!='nominal':raise ValueError('nominal only')
 p,controller,pins,bus,ancestry=prev.load();raw=(R/SOURCE).read_bytes();report=json.loads(raw)
 if report['schema']!='pressure-inward-extended-braking/1' or report['inputs']!=pins or report['startCheckpoint']!=export_domain(p) or report['controllerStart']!=vars(controller) or report['outcome']!='HORIZON' or report['stepS']!=.001 or len(report['trace'])!=100:raise ValueError('newdomain continuation lineage')
 next_tick=ancestry['nextControllerTimeS']
 for event in report['events']:
  if abs(event['timeS']-next_tick)>1e-12 or controller.update(event['timeS'],event['observation'])!=event['command']:raise ValueError('controller clock/history')
  next_tick+=.01
 if vars(controller)!=report['controllerEnd']:raise ValueError('controller endpoint')
 p=restore_domain(report['endCheckpoint'])
 if p.note!=report['trace'][-1]['note'] or p.drive.state!=report['trace'][-1]['chassis']:raise ValueError('trace endpoint')
 pins=dict(pins);pins.update({f:c.factory.r.sha(R/f) for f in FILES+[SOURCE]})
 ancestry=dict(source=SOURCE,sha256=pins[SOURCE],lastCommand=copy.deepcopy(report['events'][-1]['command']),nextControllerTimeS=next_tick)
 if raw!=(R/SOURCE).read_bytes():raise ValueError('source drift')
 return p,controller,pins,report['trace'][-1]['busV'],ancestry

def controller_due(now,next_tick):
 return now>=next_tick-1e-12

def run(output,scenario='nominal',dt=.001,duration=.2):
 if output.exists() or dt not in [.001,.0005] or duration not in [.002,.02,.2]:raise ValueError('fresh predeclared run required')
 p,controller,pins,bus,ancestry=load(scenario); initial=export_domain(p)
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
 command=copy.deepcopy(ancestry['lastCommand']);next_tick=ancestry['nextControllerTimeS']
 for i in range(round(duration/dt)):
  if controller_due(p.t,next_tick):
   observation=prior.observe(p.t,p.drive.state,p.note,bus)
   if scenario=='stale':observation['timestamp']=p.t-.051
   command=controller.update(p.t,observation);next_tick+=.01;events.append(dict(timeS=p.t,observation=observation,command=copy.deepcopy(command)))
  before=export_domain(p)
  try:row=p.step(dt,command['driveDemands'],command['pinchDemands'],[p.drive.c['massKg']*9.80665/2]*2,modes=command['modes'])
  except (ValueError,prior.DomainExit) as e:
   rejected=copy.deepcopy(last)
   if export_domain(p)!=before:raise AssertionError('non-atomic physical rejection')
   reason=str(e);status='UNKNOWN_DOMAIN';break
  row['command']=copy.deepcopy(command);row['newPhaseResidualJ']=row['energyJ']['residual']-inherited;row['contactSampleTimeS']=row['timeS']-dt/2
  rows.append(row);bus=row['busV']
  if (i+1)%50==0:print(json.dumps(dict(progressTimeS=p.t,relativeX=row['relativeRobotXY'][0],chassisVx=p.drive.state['vx'])),flush=True)
 if any(c.factory.r.sha(R/f)!=h for f,h in pins.items()):raise ValueError('changed source')
 result=dict(schema='pressure-inward-rearwrap-observation/1',inputs=pins,scenario=scenario,stepS=dt,durationS=duration,controllerPeriodS=.01,ancestry=ancestry,startCheckpoint=initial,endCheckpoint=export_domain(p),controllerStart=controller_start,controllerEnd=copy.deepcopy(vars(controller)),events=events,trace=rows,outcome=status,reason=reason,rejectedGeometryQuery=rejected,runtimeSeconds=time.perf_counter()-start,acceptance=False,limits=['Same pressure law, physical state, momentum and cumulative energy; explicit new138mm experimental gate identity.','Controller recovered by exact previous observation/command replay, retaining fault latch.','This is continued early braking, not a new controller initialization or tuned policy.','Finite horizon or first model-domain rejection; no capture, retention, lift, collision or stopping qualification.'])
 output.parent.mkdir(parents=True,exist_ok=True);tmp=output.with_suffix('.tmp');tmp.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');tmp.replace(output)
 print(json.dumps(dict(scenario=scenario,stepS=dt,outcome=status,reason=reason,timeS=p.t,note=p.note,chassis=p.drive.state,energy=c.diagnostics(p),newPhaseResidualJ=c.diagnostics(p)['residualJ']-inherited,runtime=result['runtimeSeconds'])),flush=True)
 return result
if __name__=='__main__':run(pathlib.Path(sys.argv[1]),sys.argv[2] if len(sys.argv)>2 else 'nominal',float(sys.argv[3]) if len(sys.argv)>3 else .001,float(sys.argv[4]) if len(sys.argv)>4 else .2)

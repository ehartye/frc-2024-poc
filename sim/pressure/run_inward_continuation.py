"""Explicit domain-only transition at original accepted .910 endpoint; 50ms hypothesis."""
import copy,json,pathlib,sys,time
import run_braking_boundary as base
import inward_domain_gate as domain
prior=base.prior;c=base.c;R=base.R
SOURCE='sim/pressure/braking-boundary-1ms-evidence.json'
FILES=['sim/pressure/run_inward_continuation.py','sim/pressure/inward_domain_gate.py','tests/test_pressure_inward_continuation.py']
DOMAIN=dict(schema='flat-ground-experiment-domain/1',longitudinalMm=[138,658],lateralMm=[-20,20],source='sim/pressure/inward_domain_gate.py')
def export_domain(p):
 record=c.export(p)
 record['schema']='pressure-inward-checkpoint/1'
 record['identity']['experimentalGeometryDomain']=dict(DOMAIN,sha256=c.factory.r.sha(R/DOMAIN['source']))
 record['digest']=c.digest({k:v for k,v in record.items() if k!='digest'})
 return record

def load(scenario='nominal'):
 if scenario!='nominal':raise ValueError('predeclared nominal only')
 p,controller,pins,bus,ancestry=base.load()
 raw=(R/SOURCE).read_bytes();report=json.loads(raw)
 if report['inputs']!=pins or report['startCheckpoint']!=c.export(p) or report['controllerStart']!=vars(controller) or report['stepS']!=.001 or len(report['trace'])!=48 or report['outcome']!='UNKNOWN_DOMAIN':raise ValueError('boundary lineage')
 for event in report['events']:
  if controller.update(event['timeS'],event['observation'])!=event['command']:raise ValueError('controller history')
 if vars(controller)!=report['controllerEnd']:raise ValueError('controller endpoint')
 p=c.restore(report['endCheckpoint']);bus=report['trace'][-1]['busV']
 before=c.export(p);old=p.geometry
 obs=json.loads((R/'sim/acquisition/free-obstacles.json').read_text())
 gate=domain.make_gate(obs,p.cad['note']['majorRadius'],p.cad['note']['tubeRadius'],p.cad['floorCenterZ'])
 new=c.factory.r.free_geometry.make_geometry(p.cad,gate)
 q=report['trace'][-1]['relativeRobotXY']
 if old(q)!=new(q):raise ValueError('entry geometry mismatch')
 p.geometry=new
 if c.export(p)!=before:raise ValueError('domain transition changed physical state/energy')
 pins=dict(pins);pins.update({f:c.factory.r.sha(R/f) for f in FILES+[SOURCE]})
 ancestry=dict(source=SOURCE,sha256=pins[SOURCE],oldCheckpointDigest=before['digest'],transition='state-identical domain-only extension; NOT physical validation',newDomain=DOMAIN)
 if raw!=(R/SOURCE).read_bytes():raise ValueError('source changed')
 return p,controller,pins,bus,ancestry

def run(output,scenario='nominal',dt=.001,duration=.05):
 if output.exists() or dt not in [.001,.0005] or duration not in [.002,.02,.05]:raise ValueError('fresh predeclared run required')
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
 for i in range(round(duration/dt)):
  if i%round(.01/dt)==0:
   observation=prior.observe(p.t,p.drive.state,p.note,bus)
   if scenario=='stale':observation['timestamp']=p.t-.051
   command=controller.update(p.t,observation);events.append(dict(timeS=p.t,observation=observation,command=copy.deepcopy(command)))
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
 result=dict(schema='pressure-inward-domain-continuation/1',inputs=pins,scenario=scenario,stepS=dt,durationS=duration,controllerPeriodS=.01,ancestry=ancestry,startCheckpoint=initial,endCheckpoint=export_domain(p),controllerStart=controller_start,controllerEnd=copy.deepcopy(vars(controller)),events=events,trace=rows,outcome=status,reason=reason,rejectedGeometryQuery=rejected,runtimeSeconds=time.perf_counter()-start,acceptance=False,limits=['Same pressure law, physical state, momentum and cumulative energy; explicit new138mm experimental gate identity.','Controller recovered by exact previous observation/command replay, retaining fault latch.','This is continued early braking, not a new controller initialization or tuned policy.','Finite horizon or first model-domain rejection; no capture, retention, lift, collision or stopping qualification.'])
 output.parent.mkdir(parents=True,exist_ok=True);tmp=output.with_suffix('.tmp');tmp.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');tmp.replace(output)
 print(json.dumps(dict(scenario=scenario,stepS=dt,outcome=status,reason=reason,timeS=p.t,note=p.note,chassis=p.drive.state,energy=c.diagnostics(p),newPhaseResidualJ=c.diagnostics(p)['residualJ']-inherited,runtime=result['runtimeSeconds'])),flush=True)
 return result
if __name__=='__main__':run(pathlib.Path(sys.argv[1]),sys.argv[2] if len(sys.argv)>2 else 'nominal',float(sys.argv[3]) if len(sys.argv)>3 else .001,float(sys.argv[4]) if len(sys.argv)>4 else .05)

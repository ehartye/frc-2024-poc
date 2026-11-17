"""Predeclared1.400 to1.503s same-state local refinement around contact onset."""
import copy,json,pathlib,sys,time
import run_fresh_rest_controller as fresh
prior=fresh.prior;c=fresh.c;R=fresh.R;export_domain=fresh.export_domain
SOURCE='sim/pressure/fresh-rest-firstcontact-100ms-PARTIAL.json'
FILES=['sim/pressure/run_fresh_local_refinement.py','tests/test_pressure_fresh_local_refinement.py']
START=1.4

def load(scenario='nominal'):
 if scenario!='nominal':raise ValueError('nominal only')
 p,controller,pins,bus,ancestry=fresh.load();raw=(R/SOURCE).read_bytes();r=json.loads(raw)
 if r['schema']!='pressure-fresh-rest-PARTIAL/1' or r['partial'] is not True or r['inputs']!=pins or r['startCheckpoint']!=export_domain(p) or r['controllerStart']!=vars(controller):raise ValueError('frozen partial lineage')
 command=None;count=0
 for event in r['events']:
  if event['timeS']>=START-1e-12:break
  if abs(event['timeS']-count*.01)>1e-12 or controller.update(event['timeS'],event['observation'])!=event['command']:raise ValueError('controller history/clock')
  command=copy.deepcopy(event['command']);count+=1
 if count!=140:raise ValueError('predeclared controller prefix')
 selected=[(i,row) for i,row in enumerate(r['trace']) if abs(row['timeS']-START)<1e-12]
 if len(selected)!=1 or selected[0][0]!=1399:raise ValueError('predeclared accepted state')
 index,row=selected[0];s=dict(chassis=copy.deepcopy(row['chassis']),note=copy.deepcopy(row['note']),ledger={k:row['energyJ'][k] for k in p.energy},timeS=row['timeS'],initialEnergyJ=row['energyJ']['initial'])
 c.shape(s,export_domain(p)['state'])
 if s['initialEnergyJ']!=0 or any(v<0 for v in s['ledger'].values()):raise ValueError('energy continuity')
 p.drive.state=s['chassis'];p.note=s['note'];p.energy=s['ledger'];p.t=s['timeS'];p.initial_energy=s['initialEnergyJ']
 if c.diagnostics(p)!=dict(potentialJ=row['energyJ']['spring'],kineticJ=row['energyJ']['kinetic'],residualJ=row['energyJ']['residual']):raise ValueError('endpoint pressure/energy')
 pins=dict(pins);pins.update({f:c.factory.r.sha(R/f) for f in FILES+[SOURCE]})
 if raw!=(R/SOURCE).read_bytes():raise ValueError('partial changed')
 return p,controller,pins,row['busV'],dict(source=SOURCE,sha256=pins[SOURCE],rowIndex=index,lastCommand=command,nextControllerTimeS=count*.01,scope='local same-state1.400 to1.503 only; original startup remains1ms')

def controller_due(now,next_tick):
 return now>=next_tick-1e-12

def run(output,scenario='nominal',dt=.001,duration=.103):
 if output.exists() or dt not in [.001,.0005] or duration not in [.002,.103]:raise ValueError('fresh predeclared run required')
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
 result=dict(schema='pressure-fresh-local-refinement/1',inputs=pins,scenario=scenario,stepS=dt,durationS=duration,controllerPeriodS=.01,ancestry=ancestry,startCheckpoint=initial,endCheckpoint=export_domain(p),controllerStart=controller_start,controllerEnd=copy.deepcopy(vars(controller)),events=events,trace=rows,outcome=status,reason=reason,rejectedGeometryQuery=rejected,runtimeSeconds=time.perf_counter()-start,acceptance=False,limits=['Same pressure law, physical state, momentum and cumulative energy; explicit new138mm experimental gate identity.','Controller recovered by exact previous observation/command replay, retaining fault latch.','This is continued early braking, not a new controller initialization or tuned policy.','Finite horizon or first model-domain rejection; no capture, retention, lift, collision or stopping qualification.'])
 output.parent.mkdir(parents=True,exist_ok=True);tmp=output.with_suffix('.tmp');tmp.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');tmp.replace(output)
 print(json.dumps(dict(scenario=scenario,stepS=dt,outcome=status,reason=reason,timeS=p.t,note=p.note,chassis=p.drive.state,energy=c.diagnostics(p),newPhaseResidualJ=c.diagnostics(p)['residualJ']-inherited,runtime=result['runtimeSeconds'])),flush=True)
 return result
if __name__=='__main__':run(pathlib.Path(sys.argv[1]),sys.argv[2] if len(sys.argv)>2 else 'nominal',float(sys.argv[3]) if len(sys.argv)>3 else .001,float(sys.argv[4]) if len(sys.argv)>4 else .103)

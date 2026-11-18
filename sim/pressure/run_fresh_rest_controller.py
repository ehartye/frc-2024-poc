"""Existing controller from true fresh rest, predeclared3s or first UNKNOWN."""
import copy,json,pathlib,sys,time
import run_inward_continuation as prev
prior=prev.prior;c=prev.c;R=prev.R
export_domain=prev.export_domain
FILES=['sim/pressure/run_fresh_rest_controller.py','sim/pressure/inward_domain_gate.py','sim/pressure/run_inward_continuation.py','sim/integrated/speed_approach.py','tests/test_pressure_fresh_rest_controller.py']
BASELINE='sim/pressure/corrected-entry-680ms-evidence.json'
def load(scenario='nominal'):
 if scenario!='nominal':raise ValueError('predeclared nominal only')
 p,pins=c.factory.setup();obs=json.loads((R/'sim/acquisition/free-obstacles.json').read_text())
 g=prev.domain.make_gate(obs,p.cad['note']['majorRadius'],p.cad['note']['tubeRadius'],p.cad['floorCenterZ']);p.geometry=c.factory.r.free_geometry.make_geometry(p.cad,g)
 controller=prior.SpeedApproach(p.drive.c,goal=.185,max_speed=.25)
 if p.t!=0 or p.initial_energy!=0 or any(p.energy.values()) or c.diagnostics(p)!=dict(potentialJ=0.,kineticJ=0.,residualJ=0.):raise ValueError('fresh zero energy/state required')
 baseline=json.loads((R/BASELINE).read_text())
 if baseline['inputs']!=pins:raise ValueError('baseline exact law/source mismatch')
 first=next(row for row in baseline['trace'] if row['contacts'])
 pins=dict(pins);pins.update({f:c.factory.r.sha(R/f) for f in FILES+[BASELINE]})
 return p,controller,pins,12.,dict(source=BASELINE,sha256=pins[BASELINE],baselineFirstContact=dict(acceptedTimeS=first['timeS'],midpointTimeS=first['contactSampleTimeS'],relativeXM=first['relativeRobotXY'][0],chassisVxMps=first['chassis']['vx']),lastCommand=None,nextControllerTimeS=0.,change='engage existing unchanged controller at freshrest rather than firstcontact',domain=prev.DOMAIN)

def controller_due(now,next_tick):
 return now>=next_tick-1e-12

def run(output,scenario='nominal',dt=.001,duration=3.):
 if output.exists() or output.with_suffix('.partial.json').exists() or dt not in [.001,.0005] or duration not in [.002,.1,3.]:raise ValueError('fresh predeclared run required')
 p,controller,pins,bus,ancestry=load(scenario); initial=export_domain(p)
 controller_start=copy.deepcopy(vars(controller)); rows=[];events=[];first_contact=None;contact_milestone=False;reason=None;status='HORIZON';last=None;rejected=None
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
  if first_contact is None and row['contacts']:
   first_contact=dict(acceptedTimeS=p.t,midpointTimeS=row['contactSampleTimeS'],relativeXM=row['relativeRobotXY'][0],chassisVxMps=p.drive.state['vx'],noteVxMps=p.note['v'][0],distanceToControllerGoalM=row['relativeRobotXY'][0]-.185)
   print(json.dumps(dict(firstContact=first_contact)),flush=True)
  reached_contact_milestone=first_contact is not None and not contact_milestone and p.t>=first_contact['acceptedTimeS']+.1-1e-12
  if reached_contact_milestone or (i+1)%500==0:
   if any(c.factory.r.sha(R/f)!=h for f,h in pins.items()):raise ValueError('changed source during partial save')
   partial=dict(schema='pressure-fresh-rest-PARTIAL/1',partial=True,acceptance=False,inputs=pins,startCheckpoint=initial,endCheckpoint=export_domain(p),controllerStart=controller_start,controllerEnd=copy.deepcopy(vars(controller)),heldCommand=copy.deepcopy(command),nextControllerTimeS=next_tick,firstContact=first_contact,events=events,trace=rows,maxHorizonS=duration)
   path=output.with_suffix('.partial.json');tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(partial,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');tmp.replace(path)
   if reached_contact_milestone:
    contact_milestone=True
    print(json.dumps(dict(firstContactPlus100ms=dict(timeS=p.t,relativeXM=row['relativeRobotXY'][0],chassisVxMps=p.drive.state['vx'],noteVxMps=p.note['v'][0],contacts=row['contacts'],endFeatures=row['pressureFeatures'],residualJ=row['energyJ']['residual']))),flush=True)
  if (i+1)%50==0:print(json.dumps(dict(progressTimeS=p.t,relativeX=row['relativeRobotXY'][0],chassisVx=p.drive.state['vx'])),flush=True)
 if any(c.factory.r.sha(R/f)!=h for f,h in pins.items()):raise ValueError('changed source')
 result=dict(schema='pressure-fresh-rest-controller/1',inputs=pins,scenario=scenario,stepS=dt,durationS=duration,controllerPeriodS=.01,ancestry=ancestry,startCheckpoint=initial,endCheckpoint=export_domain(p),controllerStart=controller_start,controllerEnd=copy.deepcopy(vars(controller)),heldCommand=copy.deepcopy(command),nextControllerTimeS=next_tick,events=events,firstContact=first_contact,trace=rows,outcome=status,reason=reason,rejectedGeometryQuery=rejected,runtimeSeconds=time.perf_counter()-start,acceptance=False,limits=['Same pressure law, physical state, momentum and cumulative energy; explicit new138mm experimental gate identity.','Controller initializes at rest with exact virtual sensors; fault latch retained thereafter.','Existing goal185mm/cap.25 controller starts at actual fresh rest; no tuned parameter changes.','Finite horizon or first model-domain rejection; no capture, retention, lift, collision or stopping qualification.'])
 output.parent.mkdir(parents=True,exist_ok=True);tmp=output.with_suffix('.tmp');tmp.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');tmp.replace(output)
 print(json.dumps(dict(scenario=scenario,stepS=dt,outcome=status,reason=reason,timeS=p.t,note=p.note,chassis=p.drive.state,energy=c.diagnostics(p),newPhaseResidualJ=c.diagnostics(p)['residualJ']-inherited,runtime=result['runtimeSeconds'])),flush=True)
 return result
if __name__=='__main__':run(pathlib.Path(sys.argv[1]),sys.argv[2] if len(sys.argv)>2 else 'nominal',float(sys.argv[3]) if len(sys.argv)>3 else .001,float(sys.argv[4]) if len(sys.argv)>4 else 3.)

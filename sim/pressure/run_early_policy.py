"""Predeclared first-contact controller engagement under fresh corrected source."""
import json,pathlib,sys,copy,time
import checkpoint as c
from speed_approach import SpeedApproach,observe
from free_planar import DomainExit
R=c.R
SOURCE='sim/pressure/corrected-entry-680ms-evidence.json'
FILES=['sim/pressure/run_early_policy.py','sim/integrated/speed_approach.py','tests/test_pressure_early_policy.py']
def load():
 raw=(R/SOURCE).read_bytes();report=json.loads(raw);p,pins=c.factory.setup()
 if report['schema']!='fresh-pressure-acquisition/1' or report['inputs']!=pins or report['outcome']!='HORIZON':raise ValueError('corrected fresh lineage')
 # Earliest archived accepted sample with midpoint contact, selected before policy trials.
 index=next(i for i,row in enumerate(report['trace']) if row['contacts']);row=report['trace'][index]
 if row['timeS']!=report['firstContact']['acceptedTimeS'] or abs(row['timeS']-.562)>1e-12:raise ValueError('declared entry changed')
 payload=dict(schema=c.SCHEMA,identity=c.identity(p,pins),state=dict(chassis=row['chassis'],note=row['note'],ledger={k:row['energyJ'][k] for k in p.energy},timeS=row['timeS'],initialEnergyJ=row['energyJ']['initial']),diagnostics=dict(potentialJ=row['energyJ']['spring'],kineticJ=row['energyJ']['kinetic'],residualJ=row['energyJ']['residual']))
 p=c.restore(dict(payload,digest=c.digest(payload)))
 pins=dict(payload['identity']['inputs']);pins.update({f:c.factory.r.sha(R/f) for f in FILES+[SOURCE]})
 if raw!=(R/SOURCE).read_bytes():raise ValueError('source changed')
 return p,pins,row['busV'],dict(sourceReportSha256=pins[SOURCE],rowIndex=index,selection='first accepted sample with recorded midpoint contact',relativeX=row['relativeRobotXY'][0],distanceToDomainLowerBoundM=row['relativeRobotXY'][0]-.178)

def run(output,scenario='nominal',dt=.001):
 if output.exists() or scenario not in ['nominal','stale','no-pinch'] or dt not in [.001,.0005]:raise ValueError('declared fresh case')
 p,pins,bus,ancestry=load();initial=c.export(p);controller=SpeedApproach(p.drive.c,goal=.185,pinch_enabled=scenario!='no-pinch');rows=[];events=[];reason=None;status='HORIZON';last=None;rejected=None;original=p.geometry;start=time.perf_counter();inherited=initial['diagnostics']['residualJ']
 def query(q):
  nonlocal last
  result=original(q);last=dict(relativeRobotXY=list(q),status=result['status']);return result
 p.geometry=query
 for i in range(round(.1/dt)):
  if i%round(.01/dt)==0:
   o=observe(p.t,p.drive.state,p.note,bus)
   if scenario=='stale':o['timestamp']=p.t-.051
   cmd=controller.update(p.t,o);events.append(dict(timeS=p.t,observation=o,command=copy.deepcopy(cmd)))
  before=c.export(p)
  try:row=p.step(dt,cmd['driveDemands'],cmd['pinchDemands'],[p.drive.c['massKg']*9.80665/2]*2,modes=cmd['modes'])
  except (ValueError,DomainExit) as error:
   rejected=copy.deepcopy(last)
   if c.export(p)!=before:raise AssertionError('non-atomic rejection')
   reason=str(error);status='UNKNOWN_DOMAIN';break
  row['command']=copy.deepcopy(cmd);row['newPhaseResidualJ']=row['energyJ']['residual']-inherited;row['contactSampleTimeS']=row['timeS']-dt/2;rows.append(row);bus=row['busV']
 if any(c.factory.r.sha(R/f)!=h for f,h in pins.items()):raise ValueError('source changed')
 result=dict(schema='pressure-early-policy/1',inputs=pins,scenario=scenario,stepS=dt,durationS=.1,controllerPeriodS=.01,ancestry=ancestry,startCheckpoint=initial,endCheckpoint=c.export(p),events=events,trace=rows,outcome=status,reason=reason,rejectedGeometryQuery=rejected,runtimeSeconds=time.perf_counter()-start,acceptance=False,limits=['First recorded contact engagement selected before trial using available distance, not desired capture result.','Only controller initializes; plant momentum, time and cumulative energy retained.','Same corrected pressure law and unchanged SpeedApproach; virtual observations only.','100ms horizon, no capture/retention/lift claim.'])
 output.parent.mkdir(parents=True,exist_ok=True);tmp=output.with_suffix('.tmp');tmp.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');tmp.replace(output)
 print(json.dumps(dict(scenario=scenario,stepS=dt,outcome=status,reason=reason,timeS=p.t,note=p.note,chassis=p.drive.state,energy=c.diagnostics(p),newPhaseResidualJ=c.diagnostics(p)['residualJ']-inherited,runtime=result['runtimeSeconds'])),flush=True)
if __name__=='__main__':run(pathlib.Path(sys.argv[1]),sys.argv[2] if len(sys.argv)>2 else 'nominal',float(sys.argv[3]) if len(sys.argv)>3 else .001)

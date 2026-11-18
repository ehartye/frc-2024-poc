"""Predeclared finite command envelope; unchanged reviewed phase kernel."""
import copy,json,math,pathlib,sys
R=pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0,str(R/'sim/integrated'))
from coupled_lift_run import setup,sha
from coupled_lift_diagnostic import CoupledLift,diagnose
import coupled_lift_guide_adapter as guide_adapter
EXTRA=['sim/integrated/coupled_lift_sweep.py','tests/test_coupled_lift_sweep.py','sim/integrated/coupled_lift_separated_sweep.py','sim/integrated/coupled_lift_separated_phase.py','sim/integrated/coupled_lift_guide_adapter.py','sim/integrated/guide_separation.py','tests/test_coupled_lift_separated.py','tests/test_guide_separation.py','sim/integrated/coupled_lift_diagnostic.py','sim/integrated/coupled_lift_duration.py','tests/test_coupled_lift_diagnostic.py','sim/integrated/coupled_lift_event_refine.py']
PLAN=[('feedforward',0.,0.,'drive'),('lift-plus-001',.01,0.,'drive'),('pitch-plus-001',0.,.01,'drive'),('both-plus-001',.01,.01,'drive'),('both-plus-003',.03,.03,'drive'),('coast',0.,0.,'coast'),('brake',0.,0.,'brake')]
# Targeted refinement of the three observed domain stops; not a whole-matrix convergence claim.
PLAN=[p for p in PLAN if p[0] in ('both-plus-003','coast','brake')]
def command(p,case):
 name,lift,pitch,mode=case
 if mode not in ('drive','coast','brake') or any(type(x) not in (int,float) or not math.isfinite(x) or abs(x)>.03 for x in (lift,pitch)):raise ValueError('bounded command plan')
 c=copy.deepcopy(p.commands('gravity-feedforward'));c['actuator']=[c['actuator'][0]+lift,c['actuator'][1]+pitch];c['actuatorModes']=[mode,mode]
 if max(map(abs,c['actuator']))>1:raise ValueError('duty domain')
 return c

def summarize(initial,trace,final,inherited):
 return dict(acceptedTimeS=final['timeS']-initial['timeS'],cradleDzM=final['zPivot']-initial['zPivot'],cradlePitchRad=final['theta'],noteDzM=final['noteZ']-initial['noteZ'],notePitchRad=final['notePitch'],relativeNoteCradleDzM=(final['noteZ']-final['zPivot'])-(initial['noteZ']-initial['zPivot']),relativeNoteCradleDxM=(final['note']['p'][0]-final['drive']['x'])-(initial['note']['p'][0]-initial['drive']['x']),maxIncrementalEnergyErrorJ=max([abs(t['energyJ']['residual']-inherited) for t in trace],default=0.),minBusV=min([t['busV'] for t in trace],default=None),maxAbsSampledActuatorCurrentA=max([abs(m['currentA']) for t in trace for m in t['endSampleActuatorElectrical']['motors']],default=None))

def run(output,duration=.01,dt=.001):
 if output.exists():raise ValueError('fresh output required')
 if duration not in (.05,.1) or dt not in (.001,.0005) or abs(round(duration/dt)*dt-duration)>1e-12:raise ValueError('predeclared time envelope')
 source,patch,a,h,g,c,pins=setup();pins.update({f:sha(R/f) for f in EXTRA});cases=[]
 for case in PLAN:
  p=CoupledLift(source['checkpoint'],patch,a,h,g,c,source['inputs']);cmd=command(p,case);initial=copy.deepcopy(p.s);trace=[];reason=None;rejected=None
  inherited=p.sourceInitial+p.ledger['sourceWork']-sum(v for k,v in p.ledger.items() if k!='sourceWork')-p.initialMechanical
  for _ in range(round(duration/dt)):
   before=copy.deepcopy((p.s,p.ledger))
   try:row=p.step(dt,cmd)
   except ValueError as error:
    if before!=(p.s,p.ledger):raise AssertionError('non-atomic rejected proposal')
    reason=str(error);rejected=diagnose(p,getattr(error,'rejectedEvaluation',None));break
   s=p.s
   row['endSampleActuatorElectrical']=p.act.electrical(row['busV'],cmd['actuator'],cmd['actuatorModes'],[s['drive']['x'],s['zPivot'],s['theta']],[s['drive']['vx'],s['vPivot'],s['thetaRate']])
   dx=s['note']['p'][0]-s['drive']['x'];dy=s['note']['p'][1]-s['drive']['y'];hd=s['drive']['heading']
   row['endGuideQuery']=guide_adapter.query_round_xz([math.cos(hd)*dx+math.sin(hd)*dy,s['noteZ']],s['notePitch'],g,y_m=-math.sin(hd)*dx+math.cos(hd)*dy)
   trace.append(row)
  result=dict(name=case[0],commands=cmd,initialState=initial,initialResidualJ=inherited,trace=trace,finalState=copy.deepcopy(p.s),rejectedEvaluation=rejected,outcome='UNKNOWN_DOMAIN' if reason else 'HORIZON',reason=reason,summary=summarize(initial,trace,p.s,inherited),acceptance=False)
  cases.append(result);print(case[0],result['outcome'],reason,result['summary'],flush=True)
 if any(sha(R/f)!=h for f,h in pins.items()):raise ValueError('source changed during run')
 report=dict(schema='coupled-lift-targeted-event-refinement/1',inputs=pins,stepS=dt,requestedDurationS=duration,plan=PLAN,config=c,cases=cases,acceptance=False,limits=['Explicit phase experiment fork differs only in guide-query import; disjoint AABB fallback supplies zero guide force, never new contact acceptance.','Predeclared finite duty offsets, not a selected or enforced pose trajectory.','Original 1.755 N/side normal budget, no ideal attachment.','End-sample current is diagnostic, not the midpoint work integration current.','Domain failure stops atomically; horizon is not lift or retention acceptance.'])
 output.parent.mkdir(parents=True,exist_ok=True);temp=output.with_suffix('.tmp');temp.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');temp.replace(output)
 return report
if __name__=='__main__':run(pathlib.Path(sys.argv[1]),float(sys.argv[2]) if len(sys.argv)>2 else .1,float(sys.argv[3]) if len(sys.argv)>3 else .001)

"""One model and ledger from fresh acquisition through lift and raised indexing."""
import argparse,copy,hashlib,json,math,pathlib,sys,time
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def run(a):
 sys.path[:0]=[str(a.project/f'sim/{x}') for x in ['pressure','integrated','drivetrain','actuation','acquisition']]
 # Local candidate is additive; existing committed dependency sources are pinned.
 sys.path.insert(0,str(pathlib.Path(__file__).parent))
 import native_static_cycle as cycle,native_static_lift,native_dry_friction
 from speed_approach import SpeedApproach,observe
 load=lambda p:json.loads(p.read_bytes());c=load(a.contract);binding=load(a.binding);dc=load(a.project/'sim/drivetrain/config.json');nc=load(a.project/'sim/acquisition/free-planar-config.json')['model'];cad=load(a.project/'cad/assembly/intake_components.json');ac=load(a.project/'sim/actuation/config.json')
 m=cycle.NativeStaticCycle(dc,nc,cad,binding,ac,c,mu=a.mu);initial=copy.deepcopy(m.s);controller=SpeedApproach(dc,goal=.185,max_speed=.25)
 files={'runner':pathlib.Path(__file__),'model':pathlib.Path(cycle.__file__),'integrator':pathlib.Path(native_static_lift.__file__),'friction':pathlib.Path(native_dry_friction.__file__),'contract':a.contract,'binding':a.binding};pins={k:sha(p) for k,p in files.items()};assert not a.output.exists();a.output.mkdir(parents=True)
 rows=[];events=[];joins=[];phase='acquisition';phase_step=0;dwell_ticks=0;bus=12.;outcome='HORIZON';reason=None;started=time.perf_counter();tick_steps=round(.01/a.dt);command=None;index_x=None
 for step in range(round(a.duration/a.dt)):
  if step%tick_steps==0:
   s=m.s;d=s['chassis'];n=s['note'];o=observe(s['timeS'],d,n,bus);elapsed=(step-phase_step)*a.dt
   if phase=='acquisition':
    q=c['acquisitionGate'];eligible=q['relativeXRangeM'][0]<=o['relativeX']<=q['relativeXRangeM'][1] and abs(o['relativeVx'])<=q['maxRelativeSpeedMps'] and abs(o['forwardSpeed'])<=q['maxChassisSpeedMps']
    dwell_ticks=dwell_ticks+1 if eligible else 0
    if dwell_ticks>=round(q['dwellS']/.01)+1:joins.append(dict(fromPhase=phase,toPhase='lift',step=step,state=copy.deepcopy(s)));phase='lift';phase_step=step;elapsed=0.
    elif elapsed>=c['acquisitionTimeoutS']:outcome='ACQUISITION_TIMEOUT';break
   if phase=='lift' and elapsed>=c['liftDurationS']:
    g=c['liftGate'];ok=s['noteZ']-m.z0>=g['minimumNoteRiseM'] and abs(s['noteVz']-s['vPivot'])<=g['maximumRelativeVerticalSpeedMps'] and abs(s['noteZ']-s['zPivot'])<=g['maximumRelativeVerticalDriftM']
    if not ok:outcome='LIFT_GATE_FAILED';break
    joins.append(dict(fromPhase=phase,toPhase='index',step=step,state=copy.deepcopy(s)));phase='index';phase_step=step;elapsed=0.;index_x=o['relativeX']
   if phase=='index' and elapsed>=c['indexDurationS']:outcome='BOUNDED_CYCLE_COMPLETE';break
   drive=[0.,0.];modes=['brake','brake'];pinch=[0.,0.];zdes=m.z0;vdes=ades=0.
   if phase=='acquisition':
    approach=controller.update(s['timeS'],o);drive=approach['driveDemands'];modes=approach['modes'];pinch=approach['pinchDemands']
   elif phase=='lift':
    if elapsed<.2:zdes=m.z0+.05*elapsed**2;vdes=.1*elapsed;ades=.1
    elif elapsed<.5:zdes=m.z0+.002+.02*(elapsed-.2);vdes=.02
    else:zdes=m.z0+.008
   else:
    zdes=m.z0+.008;t=elapsed
    if t<.2:dx=-.025*t*t;vx=-.05*t;ax=-.05
    elif t<.4:dx=-.001-.01*(t-.2);vx=-.01;ax=0.
    elif t<.6:u=t-.4;dx=-.003-.01*u+.025*u*u;vx=-.01+.05*u;ax=.05
    else:dx=-.004;vx=ax=0.
    con=m.contact(s,{});target_w=-(vx+2*(index_x+dx-o['relativeX']))/nc['radiusM'];pinch=[]
    for w in n['w']:
     alpha=-ax/nc['radiusM']+50*(target_w-w);torque=(nc['rotorInertiaKgM2']+.5*nc['pieceMassKg']*nc['radiusM']**2)*alpha+.5*nc['radiusM']*(con['q']['noteForceWorldN'][0]+con['floorForce'][0]);amps=nc['kvRadpsPerV']*torque;voltage=w/nc['kvRadpsPerV']+amps*nc['motorResistanceOhm']
     if amps*voltage<0:voltage=-math.copysign(.001,w)
     pinch.append(float(max(-.025,min(.025,voltage/bus))))
   command=dict(drive=drive,driveModes=modes,pinch=pinch,actuator=cycle.lift_duties(m,bus,zdes,vdes,ades),actuatorModes=['drive','drive'],requestedPivotZM=zdes)
   events.append(dict(step=step,timeS=s['timeS'],phase=phase,command=copy.deepcopy(command)))
  before=copy.deepcopy(m.s)
  try:row=m.step(a.dt,command)
  except ValueError as e:assert m.s==before;outcome='MODEL_DOMAIN_EXIT';reason=str(e);break
  row['phase']=phase;rows.append(row);bus=row['busV']
  if abs(row['energyJ']['residual'])>c['energyGate']['absoluteJ']+c['energyGate']['fractionSourceWork']*row['energyJ']['sourceWork']:outcome='ENERGY_GATE_EXIT';break
  if (step+1)%round(.25/a.dt)==0:print(json.dumps(dict(timeS=m.s['timeS'],phase=phase,relativeXM=m.s['note']['p'][0]-m.s['chassis']['x'],noteRiseM=m.s['noteZ']-m.z0,residualJ=row['energyJ']['residual'])),flush=True)
 assert all(sha(p)==pins[k] for k,p in files.items())
 final=copy.deepcopy(m.s);g=c['indexGate'];index_pass=outcome=='BOUNDED_CYCLE_COMPLETE' and abs(final['note']['p'][0]-final['chassis']['x']-index_x-c['indexDisplacementM'])<=g['maximumPositionErrorM'] and abs(final['note']['v'][0]-final['chassis']['vx'])<=g['maximumRelativeSpeedMps'] and final['noteZ']-m.z0>=g['minimumNoteRiseM']
 result=dict(schema='native-static-cycle/1',inputs=pins,stepS=a.dt,mu=a.mu,initial=initial,final=final,outcome=outcome,reason=reason,indexGatePassed=index_pass,joins=joins,events=events,trace=rows,runtimeSeconds=time.perf_counter()-started,acceptance=False,fullLauncherFeed=False,limits=c['limits']);(a.output/'receipt.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:result[k] for k in ['outcome','reason','indexGatePassed','runtimeSeconds']}),flush=True)
if __name__=='__main__':
 p=argparse.ArgumentParser()
 for n in ['project','binding','contract','output']:p.add_argument('--'+n,type=pathlib.Path,required=True)
 p.add_argument('--dt',type=float,default=.0005);p.add_argument('--duration',type=float,default=9.);p.add_argument('--mu',type=float,default=.6);a=p.parse_args();assert a.dt in [.0005,.00025,.000125] and a.mu in [0.,.1,.6];run(a)

"""Finite-effort lift hypothesis from the attained native ground-stop endpoint."""
import argparse,copy,hashlib,json,math,pathlib,sys,time
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def run(a):
 sys.path[:0]=[str(a.project/f'sim/{x}') for x in ['pressure','integrated','drivetrain','actuation','acquisition']]
 import native_pressure_lift as lift
 from signed_pressure_acquisition import DomainExit
 load=lambda p:json.loads(p.read_bytes());source=load(a.source);contract=load(a.contract);binding=load(a.binding);dc=load(a.project/'sim/drivetrain/config.json');nc=load(a.project/'sim/acquisition/free-planar-config.json')['model'];cad=load(a.project/'cad/assembly/intake_components.json');ac=load(a.project/'sim/actuation/config.json')
 assert source['outcome']=='ACTIVE_STOP_HORIZON' and source['settledLast100ms'] and source['bindingSha256']==sha(a.binding)
 m=lift.NativePressureLift(source['final'],dc,nc,cad,binding,ac,contract);assert m.s['chassis']==source['final']['chassis'] and m.s['note']==source['final']['note'];initial=copy.deepcopy(m.s);initial_mechanical_difference=m.start_mechanical-source['trace'][-1]['energyJ']['kinetic']-source['trace'][-1]['energyJ']['spring'];assert abs(initial_mechanical_difference)<1e-12
 assert not a.output.exists();a.output.mkdir(parents=True);start=time.perf_counter();bus=source['trace'][-1]['busV'];next_tick=round(m.s['timeS']*100)/100;assert abs(next_tick-m.s['timeS'])<1e-9;rows=[];events=[];outcome='HORIZON';reason=None;command=None
 files=[pathlib.Path(__file__),pathlib.Path(lift.__file__),a.source,a.binding,a.contract];pins={p.name:sha(p) for p in files}
 def command_for(s,bus):
  tick=round((s['timeS']-initial['timeS'])/.01);elapsed=tick*.01;ramp=.2;speed=.02
  if tick<20:zdes=m.z0+.5*.1*elapsed**2;vdes=.1*elapsed;ades=.1
  elif tick<50:zdes=m.z0+.002+speed*(elapsed-ramp);vdes=speed;ades=0.
  else:zdes=m.z0+.008;vdes=0.;ades=0.
  con=m.contact(s,{});d=s['chassis'];n=s['note'];desired_note_acc=20*(d['vx']+max(-.008,min(.008,2*(.185-(n['p'][0]-d['x']))))-n['v'][0]);force=nc['pieceMassKg']*desired_note_acc-con['floorForce'][0]
  def probe(w):return m.contact(s,{},[w,w])['q']
  lo,hi=-10.,10.;ql,qh=probe(lo),probe(hi)
  if not qh['noteForceWorldN'][0]<=force<=ql['noteForceWorldN'][0]:raise lift.LiftDomainError('PINCH_FORCE_REQUEST_UNSUPPORTED')
  for _ in range(24):
   mid=(lo+hi)/2
   if probe(mid)['noteForceWorldN'][0]>force:lo=mid
   else:hi=mid
  target=(lo+hi)/2;qtarget=probe(target);pinch=[]
  for i,w in enumerate(n['w']):
   current=nc['kvRadpsPerV']*(nc['rotorInertiaKgM2']*100*(target-w)-qtarget['rotorContactTorqueNm'][i]);voltage=w/nc['kvRadpsPerV']+current*nc['motorResistanceOhm']
   if current*voltage<0:voltage=-math.copysign(.001,w)
   pinch.append(max(-.025,min(.025,voltage/bus)))
  q=[d['x'],s['zPivot'],s['theta']];qd=[d['vx'],s['vPivot'],s['thetaRate']];terms=m.act.terms(q,qd);desired=[0.,ades+100*(zdes-s['zPivot'])+20*(vdes-s['vPivot']),-100*s['theta']-20*s['thetaRate']];external=[con['q']['cradleReactionWorldN'][0],con['q']['cradleReactionWorldN'][2],-con['q']['cradleMomentWorldNm'][1]];required=[sum(terms['massMatrix'][i][j]*desired[j] for j in range(3))-terms['biasForce'][i]-external[i] for i in range(3)];act=[]
  for i in range(2):
   current=required[i+1]/terms['transmissionGains'][i]/ac['motorK'];voltage=terms['rotorOmegaRadps'][i]*ac['motorK']+current*ac['resistanceOhm'];act.append(max(-.35,min(.35,voltage/bus)))
  return dict(drive=[0.,0.],driveModes=['brake','brake'],pinch=pinch,actuator=act,actuatorModes=['coast','coast'] if a.scenario=='actuators-coast' else ['drive','drive'],requestedPivotZM=zdes,requestedPivotVelocityMps=vdes)
 for step in range(round(a.duration/a.dt)):
  if m.s['timeS']>=next_tick-1e-9:
   try:command=command_for(m.s,bus)
   except (ValueError,DomainExit) as e:outcome='CONTROLLER_DOMAIN_EXIT';reason=str(e);break
   events.append(dict(timeS=m.s['timeS'],command=copy.deepcopy(command)));next_tick+=.01
  before=copy.deepcopy(m.s)
  try:row=m.step(a.dt,command)
  except (ValueError,DomainExit) as e:assert m.s==before;outcome='MODEL_DOMAIN_EXIT';reason=str(e);break
  rows.append(row);bus=row['busV']
  if abs(row['energyJ']['residual']-source['trace'][-1]['energyJ']['residual'])>.002:outcome='ENERGY_GATE_EXIT';break
  if abs(row['portMidpointPowerDifferenceW'])>1e-8:outcome='POWER_PORT_MISMATCH';break
  if (step+1)%round(.05/a.dt)==0:print(json.dumps(dict(elapsedS=m.s['timeS']-initial['timeS'],pivotM=m.s['zPivot'],noteZM=m.s['noteZ'],floorN=row['floorForceN'][2],residualJ=row['energyJ']['residual'])),flush=True)
 assert all(sha(p)==pins[p.name] for p in files)
 result=dict(schema='native-pressure-lift-hypothesis/1',inputs=pins,stepS=a.dt,scenario=a.scenario,requestedDurationS=a.duration,initial=initial,final=copy.deepcopy(m.s),initialMechanicalDifferenceJ=initial_mechanical_difference,inheritedResidualJ=source['trace'][-1]['energyJ']['residual'],events=events,trace=rows,outcome=outcome,reason=reason,runtimeSeconds=time.perf_counter()-start,nativeContactGeometryBound=True,nativeMovingMassBound=False,fullAssemblyCollisionChecked=False,acceptance=False,limits=contract['limits'])
 (a.output/'receipt.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps({k:result[k] for k in ['outcome','reason','runtimeSeconds']}),flush=True)
if __name__=='__main__':
 p=argparse.ArgumentParser()
 for n in ['project','source','binding','contract','output']:p.add_argument('--'+n,type=pathlib.Path,required=True)
 p.add_argument('--dt',type=float,default=.0005);p.add_argument('--duration',type=float,default=.1);p.add_argument('--scenario',choices=['powered','actuators-coast'],default='powered');a=p.parse_args();assert a.dt in [.0005,.00025,.000125] and a.duration in [.002,.1,.8];run(a)

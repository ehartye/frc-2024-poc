"""Fresh native-floor acquisition and finite-effort signed transport experiment."""
import argparse,copy,hashlib,json,math,pathlib,sys,time
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def run(a):
 sys.path[:0]=[str(a.project/'sim/pressure'),str(a.project/'sim/integrated')]
 from native_signed_pressure import NativeSignedPressure
 from speed_approach import SpeedApproach,observe
 import native_signed_pressure,native_pressure_arrays,signed_pressure_acquisition
 load=lambda p:json.loads(p.read_bytes())
 contract=load(a.contract);binding=load(a.binding)
 files=[pathlib.Path(__file__),a.contract,a.binding]+[pathlib.Path(m.__file__) for m in [native_signed_pressure,native_pressure_arrays,signed_pressure_acquisition]]
 inputs={str(p.name):sha(p) for p in files}
 dc=load(a.project/'sim/drivetrain/config.json');nc=load(a.project/'sim/acquisition/free-planar-config.json')['model'];cad=load(a.project/'cad/assembly/intake_components.json')
 p=NativeSignedPressure(dc,nc,cad,binding);controller=SpeedApproach(dc,goal=.185,max_speed=.25)
 assert not a.output.exists();a.output.mkdir(parents=True)
 def state():return copy.deepcopy(dict(timeS=p.t,chassis=p.drive.state,note=p.note,ledger=p.energy,initialEnergyJ=p.initial_energy))
 initial=state();phase='acquisition';bus=12.;next_tick=0.;dwell=None;index=None;transport=None;events=[];rows=[];command=None;outcome='HORIZON';reason=None;started=time.perf_counter()
 def transport_command(o):
  elapsed=p.t-index['timeS'];desired=min(.1,index['chassis']['vx']+.1*elapsed);acc=.1 if desired<.1 else 0.
  voltage=dc['motorK']*dc['gearRatio']/dc['wheelRadiusM']*desired+2*(desired-o['forwardSpeed']);dd=[];modes=[]
  for wheel in p.drive.state['wheel']:
   if voltage<dc['motorK']*dc['gearRatio']*wheel+.015:dd.append(0.);modes.append('brake')
   else:dd.append(min(.2,max(0.,voltage/bus)));modes.append('drive')
  target_relative=max(-.008,min(.008,-2*(o['relativeX']-.185)))
  note_v=p.note['v'][0];force=nc['pieceMassKg']*(acc+20*(o['forwardSpeed']+target_relative-note_v))+nc['groundMu']*nc['pieceMassKg']*9.80665*math.tanh(note_v/nc['slipRegularizationMps'])
  prepared=p.prepared(p.drive.state,p.note,{});normal=sum(c['normalMagnitudeN'] for c in prepared['sideContacts']);ratio=force/(nc['beltMu']*normal)
  if abs(ratio)>=.95:raise ValueError('CONTROLLER_TRACTION_REQUEST_DOMAIN')
  slip=nc['slipRegularizationMps']*math.atanh(ratio);target_w=(o['forwardSpeed']-note_v-slip)/nc['radiusM'];duties=[];requests=[]
  for w in p.note['w']:
   current=nc['kvRadpsPerV']*(nc['rotorInertiaKgM2']*100*(target_w-w)-nc['radiusM']*force/2)
   terminal=w/nc['kvRadpsPerV']+current*nc['motorResistanceOhm'];requested=terminal
   if terminal*current<0:terminal=-math.copysign(.001,w)
   duties.append(max(-.025,min(.025,terminal/bus)));requests.append(dict(currentA=current,terminalV=requested,appliedTerminalV=terminal))
  return dict(driveDemands=dd,modes=modes,pinchDemands=duties,desiredSpeed=desired,desiredNoteForceN=force,rotorTargetRadps=target_w,requests=requests)
 for i in range(round(11/a.dt)):
  o=observe(p.t,p.drive.state,p.note,bus)
  if p.t>=next_tick-1e-10:
   try:
    command=controller.update(p.t,o) if phase=='acquisition' else transport_command(o) if phase=='transport' else dict(driveDemands=[0.,0.],pinchDemands=[0.,0.],modes=['brake','brake'])
   except ValueError as e:outcome='CONTROLLER_DOMAIN_EXIT';reason=str(e);break
   events.append(dict(timeS=p.t,phase=phase,command=copy.deepcopy(command)));next_tick+=.01
  before=state()
  try:row=p.step(a.dt,command['driveDemands'],command['pinchDemands'],[dc['massKg']*9.80665/2]*2,modes=command['modes'])
  except ValueError as e:
   assert state()==before;outcome='MODEL_DOMAIN_EXIT';reason=str(e);break
  row['phase']=phase;rows.append(row);bus=row['busV'];o=observe(p.t,p.drive.state,p.note,bus)
  if abs(row['energyJ']['residual'])>.01+.01*row['energyJ']['sourceWork']:outcome='ENERGY_GATE_EXIT';reason='cumulative residual exceeds declared gate';break
  if phase=='acquisition':
   qualifies=.180<=o['relativeX']<=.190 and math.hypot(o['relativeVx'],o['relativeVy'])<=.003 and abs(o['forwardSpeed'])<=.003
   dwell=(p.t if dwell is None else dwell) if qualifies else None
   if dwell is not None and p.t-dwell>=.1-1e-10 and abs(p.t-next_tick)<1e-9:
    index=state();phase='transport';print(json.dumps(dict(indexTime=p.t,relativeX=o['relativeX'])),flush=True)
   elif p.t>=6-1e-10:outcome='INDEX_TIMEOUT';break
  elif phase=='transport':
   if not .180<=o['relativeX']<=.190 or math.hypot(o['relativeVx'],o['relativeVy'])>.01:outcome='TRANSPORT_CORRIDOR_EXIT';break
   if p.t-index['timeS']>=3-1e-10:
    transport=state();phase='stop'
  else:
   if not .180<=o['relativeX']<=.190:outcome='STOP_CORRIDOR_EXIT';break
   if p.t-transport['timeS']>=2-1e-10:outcome='BOUNDED_GROUND_SEQUENCE_COMPLETE';break
  if (i+1)%round(.5/a.dt)==0:print(json.dumps(dict(timeS=p.t,phase=phase,x=o['relativeX'],v=o['relativeVx'],residualJ=row['energyJ']['residual'])),flush=True)
 assert all(sha(p)==inputs[p.name] for p in files)
 result=dict(schema='native-signed-fresh/1',inputs=inputs,stepS=a.dt,initial=initial,final=state(),index=index,transportEnd=transport,outcome=outcome,reason=reason,events=events,trace=rows,runtimeSeconds=time.perf_counter()-started,acceptance=False,limits=contract['limits'])
 (a.output/'receipt.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n')
 print(json.dumps({k:result[k] for k in ['outcome','reason','stepS','runtimeSeconds']}),flush=True)
if __name__=='__main__':
 parser=argparse.ArgumentParser()
 for name in ['project','binding','contract','output']:parser.add_argument('--'+name,type=pathlib.Path,required=True)
 parser.add_argument('--dt',type=float,default=.001);a=parser.parse_args();assert a.dt in [.001,.0005,.00025];run(a)

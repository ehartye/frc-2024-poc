"""Finite signed pinch control during actual chassis braking, without state reset."""
import argparse,copy,hashlib,json,math,pathlib,sys,time
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def run(a):
 sys.path[:0]=[str(a.project/'sim/pressure'),str(a.project/'sim/integrated')]
 from native_signed_pressure import NativeSignedPressure
 from speed_approach import observe
 load=lambda p:json.loads(p.read_bytes());source=load(a.trajectory);contract=load(a.contract);binding=load(a.binding);cad=load(a.project/'cad/assembly/intake_components.json');nc=load(a.project/'sim/acquisition/free-planar-config.json')['model'];dc=load(a.project/'sim/drivetrain/config.json')
 assert source['transportEnd'] and source['index'];initial=copy.deepcopy(source['transportEnd']);dt=a.dt
 model=NativeSignedPressure(dc,nc,cad,binding);model.drive.state=copy.deepcopy(initial['chassis']);model.note=copy.deepcopy(initial['note']);model.energy=copy.deepcopy(initial['ledger']);model.initial_energy=initial['initialEnergyJ'];model.t=initial['timeS']
 def state():return copy.deepcopy(dict(timeS=model.t,chassis=model.drive.state,note=model.note,ledger=model.energy,initialEnergyJ=model.initial_energy))
 assert state()==initial
 first=min(source['trace'],key=lambda x:abs(x['timeS']-model.t));assert abs(first['timeS']-model.t)<1e-9 and first['chassis']==model.drive.state and first['note']==model.note
 bus=first['busV'];next_tick=round(model.t*100)/100;assert abs(next_tick-model.t)<1e-9
 assert not a.output.exists();a.output.mkdir(parents=True);events=[];rows=[];reason=None;outcome='HORIZON';start=time.perf_counter();command=None
 for step in range(round(contract['durationS']/dt)):
  if model.t>=next_tick-1e-9:
   o=observe(model.t,model.drive.state,model.note,bus)
   e=model.drive.electrical(bus,[0.,0.],modes=['brake','brake']);torque=sum(x['electromagneticTorqueNm'] for x in e['perMotor'])*dc['gearRatio'];drag=sum(2*dc['motorViscousDragNmPerRadps']*w*dc['gearRatio']**2+dc['gearViscousDragNmPerRadps']*w for w in model.drive.state['wheel']);q=model.acquisition_rates(model.note,model.drive.state,bus,[0.,0.],{})
   acceleration=((torque-drag)/dc['wheelRadiusM']+q['reactionForceWorldN'][0])/(dc['massKg']+2*model.drive.axle_inertia/dc['wheelRadiusM']**2)
   relative_target=max(-.008,min(.008,-2*(o['relativeX']-.185)));note_v=model.note['v'][0];force=nc['pieceMassKg']*(acceleration+20*(o['forwardSpeed']+relative_target-note_v))+nc['groundMu']*nc['pieceMassKg']*9.80665*math.tanh(note_v/nc['slipRegularizationMps']);normal=sum(x['normalMagnitudeN'] for x in q['contacts']);ratio=force/(nc['beltMu']*normal)
   if abs(ratio)>=.95:outcome='CONTROLLER_TRACTION_REQUEST_DOMAIN';break
   slip=nc['slipRegularizationMps']*math.atanh(ratio);target_w=(o['forwardSpeed']-note_v-slip)/nc['radiusM'];duties=[]
   for w in model.note['w']:
    current=nc['kvRadpsPerV']*(nc['rotorInertiaKgM2']*100*(target_w-w)-nc['radiusM']*force/2);terminal=w/nc['kvRadpsPerV']+current*nc['motorResistanceOhm']
    if terminal*current<0:terminal=-math.copysign(.001,w)
    duties.append(max(-.025,min(.025,terminal/bus)))
   command=dict(driveDemands=[0.,0.],modes=['brake','brake'],pinchDemands=duties,estimatedChassisAccelerationMps2=acceleration,requestedNoteForceN=force);events.append(dict(timeS=model.t,command=copy.deepcopy(command)));next_tick+=.01
  before=state()
  try:row=model.step(dt,command['driveDemands'],command['pinchDemands'],[dc['massKg']*9.80665/2]*2,modes=command['modes'])
  except ValueError as e:assert state()==before;outcome='MODEL_DOMAIN_EXIT';reason=str(e);break
  rows.append(row);bus=row['busV'];o=observe(model.t,model.drive.state,model.note,bus)
  if not .180<=o['relativeX']<=.190 or math.hypot(o['relativeVx'],o['relativeVy'])>.01:outcome='STOP_CORRIDOR_EXIT';break
  if abs(row['energyJ']['residual'])>.01+.01*row['energyJ']['sourceWork']:outcome='ENERGY_GATE_EXIT';break
  if (step+1)%round(.5/dt)==0:print(json.dumps(dict(timeS=model.t,relativeX=o['relativeX'],relativeV=o['relativeVx'],chassisV=o['forwardSpeed'])),flush=True)
 if len(rows)==round(contract['durationS']/dt):outcome='ACTIVE_STOP_HORIZON'
 final=state();tail=[x for x in rows if x['timeS']>=final['timeS']-.1-1e-9];settled=outcome=='ACTIVE_STOP_HORIZON' and len(tail)>=round(.1/dt) and all(abs(x['chassis']['vx'])<=.003 and abs(x['note']['v'][0]-x['chassis']['vx'])<=.003 for x in tail)
 result=dict(schema='native-signed-continuous-stop/1',sourceTrajectorySha256=sha(a.trajectory),sourceTransportCheckpoint=initial,initial=initial,final=final,stepS=dt,contractSha256=sha(a.contract),bindingSha256=sha(a.binding),scriptSha256=sha(pathlib.Path(__file__)),events=events,trace=rows,outcome=outcome,reason=reason,settledLast100ms=settled,maximumResidualJ=max(abs(x['energyJ']['residual']) for x in rows),runtimeSeconds=time.perf_counter()-start,acceptance=False,limits=['Exact continuation from attained native transport endpoint; old failed coast-stop remains preserved','Brake acceleration is controller feedforward only; physical acceleration still comes from finite tire/motor/contact forces','Same uncalibrated planar-floor model, not lift, feed or hardware validation'])
 (a.output/'receipt.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps({k:result[k] for k in ['outcome','settledLast100ms','runtimeSeconds']}),flush=True)
if __name__=='__main__':
 p=argparse.ArgumentParser()
 for n in ['project','binding','trajectory','contract','output']:p.add_argument('--'+n,type=pathlib.Path,required=True)
 p.add_argument('--dt',type=float,required=True);a=p.parse_args();assert a.dt in [.0005,.00025];run(a)

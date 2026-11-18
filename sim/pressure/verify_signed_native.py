import argparse,copy,hashlib,json,math,pathlib,sys,time
import numpy as np
p=argparse.ArgumentParser()
for name in ['project','binding','contract','output']:p.add_argument('--'+name,type=pathlib.Path,required=True)
a=p.parse_args();assert not a.output.exists();sys.path[:0]=[str(a.project/'sim/pressure'),str(a.project/'sim/integrated')]
import signed_pressure_acquisition as signed,native_signed_pressure as native,native_pressure_arrays as arrays,pressure_tiled_geometry as scalar,finite_area,finite_transport
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
config=json.loads((a.project/'cad/assembly/intake_components.json').read_bytes());motor=json.loads((a.project/'sim/acquisition/free-planar-config.json').read_bytes())['model'];binding=json.loads(a.binding.read_bytes());contract=json.loads(a.contract.read_bytes());g=binding['effectiveGeometryMm'];original=finite_area.belt.geometry;finite_area.belt.geometry=lambda c:dict(g)
electrical=[];max_error=0.
for bus in [0.,6.,12.]:
 for duty in [-1.,-.025,-.0034,0.,.0034,.025,1.]:
  for omega in [-900.,-300.,-20.,-1.,0.,1.,20.,300.,900.]:
   try:r=signed.motor_electrical(duty,bus,omega,motor)
   except signed.DomainExit as e:electrical.append(dict(busV=bus,duty=duty,omega=omega,rejected=str(e)));continue
   mirror=signed.motor_electrical(-duty,bus,-omega,motor);assert r['currentA']==-mirror['currentA'] and r['terminalPowerW']==mirror['terminalPowerW'];assert abs(r['currentA'])<=40 and r['terminalPowerW']>=-1e-10;max_error=max(max_error,abs(r['powerResidualW']));assert abs(r['powerResidualW'])<1e-9
   if duty>=0:
    old=0. if duty==0 else max(0.,min(40.,(duty*bus-omega/motor['kvRadpsPerV'])/motor['motorResistanceOhm']));assert r['currentA']==old
   electrical.append(dict(busV=bus,duty=duty,omega=omega,**r))
rows=[];started=time.perf_counter()
poses=[(.658,0.,0.),(.4,0.,0.),(.324,0.,0.),(.185,0.,0.),(.178,0.,0.),(.185,.00005,0.),(.185,-.00005,0.),(.185,.0005,0.),(.185,.02,0.),(.185,0.,.012)]
for resolution in [128,1024]:
 for x,y,dz in poses:
  args=dict(note_center=[x,y,.0254+dz],cradle_origin=[.178,0,.0254],resolution=(resolution,resolution),k_area=1e7)
  values=[];timings=[]
  for impl in ['scalar','arrays']:
   start=time.perf_counter()
   try:
    q=scalar.prepare(config,**args) if impl=='scalar' else arrays.prepare(config,g,**args)
    if impl=='scalar' and any(c['penetrationM']>.0015+1e-12 for c in q['contacts']):raise finite_area.DomainError('UNSUPPORTED_SUBSTRATE_CONTACT')
    result=dict(outcome='RETURNED',count=len(q['contacts']) if impl=='scalar' else q['contacts'],potentialJ=q['potentialJ'],rates=[])
    for speed,rotors in [(0.,[0.,0.]),(.1,[-.5,-.5]),(.01,[2.,1.])]:
     rates_args=dict(note_velocity=[speed,0,0],note_omega=[0,0,.01],cradle_velocity=[.1,0,0],cradle_omega=[0,0,0],rotor_omega=rotors,pitch_radius_m=motor['radiusM'],mu=motor['beltMu'],slip_scale=motor['slipRegularizationMps'])
     result['rates'].append((finite_transport if impl=='scalar' else arrays).rates(q,**rates_args))
   except ValueError as e:result=dict(outcome='REJECTED',reason=str(e))
   timings.append(time.perf_counter()-start);values.append(result)
  one,two=values;assert one['outcome']==two['outcome']
  if one['outcome']=='REJECTED':assert one['reason']==two['reason'];errors={}
  else:
   assert one['count']==two['count'];errors={'potentialJ':abs(one['potentialJ']-two['potentialJ'])};assert errors['potentialJ']<=contract['arrayComparison']['potentialAbsoluteJ']
   for field in ['noteForceWorldN','noteMomentWorldNm','cradleReactionWorldN','cradleMomentWorldNm','rotorContactTorqueNm']:
    errors[field]=max(max(abs(u-v) for u,v in zip(r[field],s[field])) for r,s in zip(one['rates'],two['rates']));assert errors[field]<=contract['arrayComparison']['momentAbsoluteNm' if 'Moment' in field or 'Torque' in field else 'forceAbsoluteN'],(x,y,dz,field,errors[field])
  rows.append(dict(resolution=resolution,pose=[x,y,dz],outcome=one['outcome'],reason=one.get('reason'),contacts=one.get('count'),errors=errors,scalarSeconds=timings[0],arraySeconds=timings[1]));print(json.dumps(rows[-1]),flush=True)
finite_area.belt.geometry=original
drive=json.loads((a.project/'sim/drivetrain/config.json').read_bytes());model=native.NativeSignedPressure(drive,motor,config,binding);assert model.t==0 and model.initial_energy==0 and all(v==0 for v in model.energy.values()) and model.prepared(model.drive.state,model.note,{})['potentialJ']==0
result=dict(schema='signed-native-preflight/1',passed=True,acceptance=False,scriptSha256=sha(pathlib.Path(__file__)),contractSha256=sha(a.contract),bindingSha256=sha(a.binding),sourceHashes={pathlib.Path(m.__file__).name:sha(pathlib.Path(m.__file__)) for m in [signed,native,arrays,scalar]},electricalCases=len(electrical),maximumElectricalPowerResidualW=max_error,electrical=electrical,geometryComparisons=rows,freshZeroState=True,runtimeSeconds=time.perf_counter()-started,limits=['Prescribed-state and electrical preflight only; no acquisition, retention or hardware acceptance'])
a.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps({k:result[k] for k in ['passed','electricalCases','maximumElectricalPowerResidualW','freshZeroState','runtimeSeconds']}))

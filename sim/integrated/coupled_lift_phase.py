"""Narrow symmetric XZ continuation, finite loads and one bus; not accepted lift."""
import pathlib,sys,copy,math,json,hashlib
import numpy as np
R=pathlib.Path(__file__).resolve().parents[2]
sys.path[:0]=[str(R/'sim/integrated'),str(R/'sim/drivetrain'),str(R/'sim/actuation'),str(R/'sim/holding'),str(R/'cad/assembly')]
from plant import DrivePlant
from mechanics import ActuatorMechanics
from finite_motor_patch import FiniteMotorPatch
from shared_supply import solve_supply
from continuation_state import restore
import patch_registration as registration,guide_contact,finite_retention_guide as guide
class PhaseDomainError(ValueError):pass
def dot(a,b):return sum(x*y for x,y in zip(a,b))
def add(a,b):return [x+y for x,y in zip(a,b)]
def cross(a,b):return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
def rotate(v,m):return registration.matvec(m,v)
class CoupledLift:
 def __init__(self,saved,patch,act_config,hold_config,guide_contract,config,expected_identity):
  if not expected_identity or any(hashlib.sha256((R/f).read_bytes()).hexdigest()!=h for f,h in expected_identity.items()):raise PhaseDomainError('STALE_ENTRY_MODEL_IDENTITY')
  restore(saved,lambda q:None,expected_identity,json.loads((R/'sim/drivetrain/config.json').read_text()),json.loads((R/'sim/acquisition/free-planar-config.json').read_text())['model'])
  if patch['timeS']!=saved['timeS'] or patch['preservedState']!={'chassis':saved['drive'],'note':saved['note']}:raise PhaseDomainError('PATCH_CHECKPOINT_REGISTRATION_IDENTITY')
  self.c=copy.deepcopy(config);self.nc=copy.deepcopy(saved['noteConfig']);self.ac=copy.deepcopy(act_config);self.hc=copy.deepcopy(hold_config);self.patch=copy.deepcopy(patch['cradlePoints']);self.guide=guide_contract
  dc=copy.deepcopy(saved['driveConfig']);mechanism=act_config['carriageMassKg']+act_config['rotatingMassKg']
  if dc['massKg']!=45. or mechanism!=7.:raise PhaseDomainError('EXPLICIT_45KG_PARTITION_REQUIRED')
  dc['massKg']-=mechanism;self.drive=DrivePlant(dc);self.loads=[45.*9.80665/2]*2
  self.s=dict(timeS=saved['timeS'],drive=copy.deepcopy(saved['drive']),note=copy.deepcopy(saved['note']),zPivot=.0254,vPivot=0.,theta=0.,thetaRate=0.,noteZ=.0254,noteVz=0.,notePitch=0.,notePitchRate=0.,pinchW=[-w for w in saved['note']['w']])
  self.act=ActuatorMechanics(act_config,dict(timeS=saved['timeS'],q=[self.s['drive']['x'],.0254,0.],qd=[self.s['drive']['vx'],0.,0.]))
  self.hold=FiniteMotorPatch(hold_config,dict(timeS=saved['timeS'],frameAngleRad=0.,rotorOmegaRadps=self.s['pinchW']))
  self.ledger=copy.deepcopy(saved['energy']);self.ledger.update(actuatorWinding=0.,actuatorBrake=0.,floorNormalHeat=0.,floorFrictionHeat=0.,guideNormalHeat=0.,guideFrictionHeat=0.,pinchBrake=0.)
  self.sourceInitial=saved['initialEnergyJ'];self.inheritedSpring=config['inheritedNormalSpringJ'];self.initialZ=.0254
  self.potential0=self.mechanism_potential(self.s)
  initial=self.rates(self.s,12.,self.commands('gravity-feedforward'),{})
  if initial['guideElasticJ']!=0 or initial['floorElasticJ']!=0:raise PhaseDomainError('NONZERO_INITIAL_EXTENSION_ENERGY')
  self.initialMechanical=self.mechanical(self.s,initial)
 def commands(self,scenario):
  # Finite static gravity feedforward; source commands for drive/pinch supplied explicitly.
  c=self.ac;k=c['motorK'];res=c['resistanceOhm'];mass=c['carriageMassKg']+c['rotatingMassKg'];a=c['comLocalXZ_M'][0]
  currents=[mass*c['gravityMps2']/(c['liftGearRatio']/c['liftRadiusM']*k),c['rotatingMassKg']*c['gravityMps2']*a/(c['pitchGearRatio']*k)]
  if scenario not in ['gravity-feedforward','actuators-coast']:raise ValueError('scenario')
  return dict(drive=self.c['driveDemands'],pinch=self.c['pinchDemandsRetention'],actuator=[i*res/12. for i in currents],actuatorModes=['coast','coast'] if scenario=='actuators-coast' else ['drive','drive'])
 def mechanism_potential(self,s):
  t=self.act.terms([s['drive']['x'],s['zPivot'],s['theta']],[0,0,0]);return self.ac['gravityMps2']*(t['mechanismMassKg']*s['zPivot']+self.ac['rotatingMassKg']*t['comOffsetXZ'][1])
 def mechanical(self,s,r):
  d=s['drive'];n=s['note'];q=[d['x'],s['zPivot'],s['theta']];v=[d['vx'],s['vPivot'],s['thetaRate']]
  mech=self.act.energy(q,v)-self.potential0
  body=.5*self.nc['pieceMassKg']*(dot(n['v'],n['v'])+s['noteVz']**2)+.5*self.nc['yawInertiaKgM2']*n['spin']**2+.5*self.c['notePitchInertiaKgM2']*s['notePitchRate']**2+.5*self.nc['rotorInertiaKgM2']*dot(s['pinchW'],s['pinchW'])+self.nc['pieceMassKg']*9.80665*(s['noteZ']-self.initialZ)
  return self.drive.kinetic(d)+.5*7.*d['vy']**2+mech+body+self.inheritedSpring+r['floorElasticJ']+r['guideElasticJ']
 def rates(self,s,bus,cmd,cache):
  d=s['drive'];n=s['note']
  if abs(d['heading'])>self.c['planarAngleLimitRad'] or abs(n['yaw']-d['heading'])>self.c['planarAngleLimitRad']:raise PhaseDomainError('UNSUPPORTED_PLANAR_FRAME_REGISTRATION')
  if max(abs(d['vy']),abs(d['yaw_rate']),abs(n['v'][1]),abs(n['spin']))>self.c['nonplanarVelocityLimit']:raise PhaseDomainError('UNSUPPORTED_NONPLANAR_VELOCITY')
  heading=d['heading'];basis=registration.rotation(heading,s['theta']);nbasis=registration.rotation(n['yaw'],s['notePitch']);origin=[d['x']+.178*math.cos(heading),d['y']+.178*math.sin(heading),s['zPivot']];center=[*n['p'],s['noteZ']]
  key=tuple(origin+center+[heading,n['yaw'],s['theta'],s['notePitch']])
  if key not in cache:cache[key]=registration.evaluate(self.patch,origin,basis,center,nbasis,.1524,.0254)
  witnesses=cache[key]
  if not witnesses['allNominalWitnessesInside']:raise PhaseDomainError('UNKNOWN_FINITE_PATCH_PERSISTENCE')
  axis=rotate([1.,0.,0.],basis);up=rotate([0.,0.,1.],basis);pitchaxis=[math.sin(heading),-math.cos(heading),0.];noteaxis=[math.sin(n['yaw']),-math.cos(n['yaw']),0.]
  cw=add([0.,0.,d['yaw_rate']],[x*s['thetaRate'] for x in pitchaxis]);nw=add([0.,0.,n['spin']],[x*s['notePitchRate'] for x in noteaxis]);nv=[*n['v'],s['noteVz']]
  force=[0.,0.,0.];moment=[0.,0.,0.];cradleF=[0.,0.,0.];cradleM=[0.,0.,0.];beltloss=normalwork=0.;rotortorque=[0.,0.];rows=[]
  pe=[self.hold.electrical(bus,cmd['pinch'][i],'drive',s['pinchW'][i]) for i in [0,1]]
  for p in witnesses['points']:
   i=p['motor'];point=p['pointWorldM'];arm=[a-b for a,b in zip(point,origin)];lever=p['leverWorldM'];cv=add([d['vx'],d['vy'],s['vPivot']],cross(cw,arm));bv=add(nv,cross(nw,lever));rel=[a-b+self.hc['beltRadiusM']*s['pinchW'][i]*t for a,b,t in zip(cv,bv,axis)];slip=[dot(rel,axis),dot(rel,up)];speed=math.hypot(*slip);normal=self.hc['normalSideN'][i]/15;cap=self.hc['mu']*normal;scale=0. if speed==0 else cap*math.tanh(speed/self.hc['slipRegularizationMps'])/speed
   f=[normal*p['normalWorld'][j]+scale*(slip[0]*axis[j]+slip[1]*up[j]) for j in range(3)];force=add(force,f);moment=add(moment,cross(lever,f));reaction=[-x for x in f];cradleF=add(cradleF,reaction);cradleM=add(cradleM,cross(arm,reaction));rotortorque[i]-=self.hc['beltRadiusM']*dot(f,axis);beltloss+=scale*speed*speed;normalwork+=normal*dot(p['normalWorld'],[a-b for a,b in zip(cv,bv)])
   rows.append(dict(motor=i,forceWorldN=f,pointWorldM=point))
  if abs(normalwork)>self.c['nonplanarPowerLimitW']:raise PhaseDomainError('INHERITED_NORMAL_ENERGY_WORK_DOMAIN')
  floorE=floorHeat=floorFriction=0.
  for j in range(self.c['floorPoints']):
   a=2*math.pi*j/self.c['floorPoints'];ring=rotate([.1524*math.cos(a),.1524*math.sin(a),0.],nbasis);lever=[ring[0],ring[1],ring[2]-.0254];z=s['noteZ']+lever[2];delta=max(0.,-z)
   if delta==0:continue
   pv=add(nv,cross(nw,lever));k=self.c['floorStiffnessNpm']/self.c['floorPoints'];damper=self.c['floorDampingNspm']/self.c['floorPoints'];compression=max(-pv[2],0.);normal=k*delta+damper*compression;speed=math.hypot(pv[0],pv[1]);factor=0. if speed==0 else -self.nc['groundMu']*normal*math.tanh(speed/self.nc['slipRegularizationMps'])/speed;f=[factor*pv[0],factor*pv[1],normal];force=add(force,f);moment=add(moment,cross(lever,f));floorE+=.5*k*delta*delta;floorHeat+=damper*compression**2;floorFriction-=f[0]*pv[0]+f[1]*pv[1]
  rel=[n['p'][0]-d['x'],n['p'][1]-d['y']];rx=math.cos(heading)*rel[0]+math.sin(heading)*rel[1];ry=-math.sin(heading)*rel[0]+math.cos(heading)*rel[1]
  gkey=('guide',rx,ry,s['noteZ'],s['notePitch'])
  if gkey not in cache:cache[gkey]=guide.query_round_xz([rx,s['noteZ']],s['notePitch'],self.guide,y_m=ry)
  g=cache[gkey];gp=g.get('guidePointRobotXZm',[rx,s['noteZ']]);world=[d['x']+gp[0]*math.cos(heading),d['y']+gp[0]*math.sin(heading),gp[1]];gv=add(nv,cross(nw,[a-b for a,b in zip(world,center)]))
  gw=guide_contact.wrench(g,stiffness=self.c['guideStiffnessNpm'],damping=self.c['guideDampingNspm'],mu=self.c['guideMu'],slip_scale=.01,chassis_position=[d['x'],d['y'],0.],chassis_velocity=[d['vx'],d['vy'],0.],heading=heading,yaw_rate=d['yaw_rate'],piece_center=center,piece_point_velocity=gv,contact_y_robot=ry)
  force=add(force,gw['forceWorldN']);moment=add(moment,gw['pieceMomentWorldNm'])
  dr=self.drive._rates(d,bus,cmd['drive'],('drive','drive'),self.loads,(0.,0.),0.);fx,fy=dr['contactForceBodyN'];tire=[math.cos(heading)*fx-math.sin(heading)*fy,math.sin(heading)*fx+math.cos(heading)*fy]
  q=[d['x'],s['zPivot'],s['theta']];qd=[d['vx'],s['vPivot'],s['thetaRate']];external=[tire[0]+cradleF[0]+gw['chassisReactionWorldN'][0],cradleF[2],dot(cradleM,pitchaxis)]
  ag=self.act.generalized(bus,cmd['actuator'],cmd['actuatorModes'],q,qd,external);matrix=copy.deepcopy(ag['massMatrix']);matrix[0][0]+=38.;acc=np.linalg.solve(np.array(matrix),np.array(ag['rhs'])).tolist()
  discarded=dict(noteYForceN=force[1],noteRollTorqueNm=moment[0],noteYawTorqueNm=moment[2],baseYForceN=tire[1]+cradleF[1]+gw['chassisReactionWorldN'][1],baseRollTorqueNm=cradleM[0]+gw['chassisMomentWorldNm'][0],baseYawTorqueNm=dr['contactYawTorqueNm']+cradleM[2]+gw['chassisMomentWorldNm'][2])
  if max(map(abs,discarded.values()))>self.c['nonplanarWrenchLimit']:raise PhaseDomainError('UNSUPPORTED_NONPLANAR_WRENCH')
  motors=dr['electrical']['perMotor']+pe+ag['electrical']['motors'];bad=dr['electrical']['infeasibleCurrentLimitVoltage'] or ag['electrical']['infeasibleCurrentLimitVoltage'] or any(p['infeasible'] for p in pe)
  rates=dict(sourceWork=0.,sourceHeat=0.,driveWinding=dr['energyRates']['windingHeat'],driveBrake=dr['energyRates']['controllerBrakeHeat'],driveDrag=dr['energyRates']['motorDragHeat']+dr['energyRates']['gearDragHeat'],tireSlip=dr['energyRates']['tireSlipHeat'],pinchWinding=sum(p['windingHeatW'] for p in pe),pinchBrake=sum(p['brakeHeatW'] for p in pe),normalHeat=0.,beltSlip=beltloss,groundHeat=0.,actuatorWinding=sum(e['windingHeatW'] for e in ag['electrical']['motors']),actuatorBrake=sum(e['brakeHeatW'] for e in ag['electrical']['motors']),floorNormalHeat=floorHeat,floorFrictionHeat=floorFriction,guideNormalHeat=gw['normalHeatW'],guideFrictionHeat=gw['frictionHeatW'])
  return dict(acc=acc,noteAccel=[force[0]/self.nc['pieceMassKg'],force[2]/self.nc['pieceMassKg']-9.80665],notePitchAccel=dot(moment,noteaxis)/self.c['notePitchInertiaKgM2'],pinchAccel=[(p['torqueNm']+t)/self.hc['rotorInertiaKgM2'] for p,t in zip(pe,rotortorque)],wheelAccel=dr['dw'],busPowerW=sum(p['busPowerW'] for p in motors),infeasible=bad,energyRates=rates,floorElasticJ=floorE,guideElasticJ=gw['elasticJ'],discardedNonplanar=discarded,normalReservoirPowerW=normalwork,guideStatus=g['status'],guideMaterialStatus=g['nativeMaterialStatus'],patchForce=force)
 def shifted(self,s,r,dt):
  v=copy.deepcopy(s);v['timeS']+=dt;d=v['drive'];old=s['drive'];d['t']+=dt
  for key,rate in [('x',old['vx']),('y',old['vy']),('heading',old['yaw_rate']),('vx',r['acc'][0])]:d[key]+=dt*rate
  d['wheel']=[x+dt*a for x,a in zip(old['wheel'],r['wheelAccel'])];d['wheel_angle']=[x+dt*w for x,w in zip(old['wheel_angle'],old['wheel'])]
  n=v['note'];n['p']=[x+dt*a for x,a in zip(s['note']['p'],s['note']['v'])];n['v'][0]+=dt*r['noteAccel'][0];n['yaw']+=dt*s['note']['spin']
  for key,rate in [('zPivot',s['vPivot']),('vPivot',r['acc'][1]),('theta',s['thetaRate']),('thetaRate',r['acc'][2]),('noteZ',s['noteVz']),('noteVz',r['noteAccel'][1]),('notePitch',s['notePitchRate']),('notePitchRate',r['notePitchAccel'])]:v[key]+=dt*rate
  v['pinchW']=[w+dt*a for w,a in zip(s['pinchW'],r['pinchAccel'])];n['w']=[-w for w in v['pinchW']];return v
 def preview(self,dt,bus,cmd,cache):
  a=self.rates(self.s,bus,cmd,cache);mid=self.shifted(self.s,a,dt/2);b=self.rates(mid,bus,cmd,cache);end=self.shifted(self.s,b,dt)
  # Positions use midpoint velocities, not beginning-of-step velocities.
  for key,vel in [('zPivot','vPivot'),('theta','thetaRate'),('noteZ','noteVz'),('notePitch','notePitchRate')]:end[key]=self.s[key]+dt*mid[vel]
  end['drive']['x']=self.s['drive']['x']+dt*mid['drive']['vx'];end['drive']['wheel_angle']=[x+dt*w for x,w in zip(self.s['drive']['wheel_angle'],mid['drive']['wheel'])];end['note']['p'][0]=self.s['note']['p'][0]+dt*mid['note']['v'][0]
  return a,b,end
 def step(self,dt,cmd):
  if type(dt) not in (int,float) or not math.isfinite(dt) or not 0<dt<=.001:raise ValueError('positive dt <=1ms')
  cache={}
  def port(bus):return {'busPowerW':self.preview(dt,bus,cmd,cache)[1]['busPowerW']}
  supply=solve_supply({'eight-motor-phase':port},12.,.02);a,b,end=self.preview(dt,supply['busV'],cmd,cache)
  if a['infeasible'] or b['infeasible']:raise PhaseDomainError('CONTROLLER_VOLTAGE_DOMAIN')
  final=self.rates(end,supply['busV'],cmd,{})
  rates=dict(b['energyRates'],sourceWork=supply['sourcePowerW'],sourceHeat=supply['sourceHeatW']);ledger={k:self.ledger[k]+dt*rates[k] for k in self.ledger};energy=self.mechanical(end,final);res=self.sourceInitial+ledger['sourceWork']-sum(v for k,v in ledger.items() if k!='sourceWork')-energy
  self.s=end;self.ledger=ledger
  return dict(state=copy.deepcopy(end),busV=supply['busV'],energyJ=dict(ledger,mechanical=energy,residual=res,inheritedNormalSpring=self.inheritedSpring),discardedNonplanar=final['discardedNonplanar'],normalReservoirPowerW=final['normalReservoirPowerW'],guideStatus=final['guideStatus'],guideMaterialStatus=final['guideMaterialStatus'],acceptance=False)

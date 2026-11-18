"""Experimental XZ lift continuation with the native-bound finite-area field.

Moving mass/transmission and compliant-floor parameters remain explicit hypotheses.
No full native collision or calibrated lift claim is made.
"""
import copy,math
import numpy as np
from plant import DrivePlant
from mechanics import ActuatorMechanics
from shared_supply import solve_supply
from patch_registration import rotation
from signed_pressure_acquisition import motor_electrical,DomainExit
import native_pressure_arrays as arrays

class LiftDomainError(ValueError):pass

class NativePressureLift:
 def __init__(self,checkpoint,drive,note,cad,binding,actuator,contract):
  self.nc=copy.deepcopy(note);self.cad=copy.deepcopy(cad);self.binding=copy.deepcopy(binding);self.ac=copy.deepcopy(actuator);self.c=copy.deepcopy(contract);self.initial=copy.deepcopy(checkpoint);self.z0=cad['floorCenterZ']/1000
  mass=actuator['carriageMassKg']+actuator['rotatingMassKg'];assert drive['massKg']==45 and mass==7
  dc=copy.deepcopy(drive);dc['massKg']-=mass;self.drive=DrivePlant(dc);self.loads=[45*9.80665/2]*2
  self.s=copy.deepcopy(checkpoint);self.s.update(zPivot=self.z0,vPivot=0.,theta=0.,thetaRate=0.,noteZ=self.z0,noteVz=0.,notePitch=0.,notePitchRate=0.)
  self.act=ActuatorMechanics(actuator,dict(timeS=self.s['timeS'],q=[self.s['chassis']['x'],self.z0,0.],qd=[self.s['chassis']['vx'],0.,0.]))
  self.ledger=copy.deepcopy(checkpoint['ledger']);self.ledger.update(actuatorWinding=0.,actuatorBrake=0.,floorNormalHeat=0.,floorFrictionHeat=0.)
  self.s['ledger']=self.ledger;self.potential0=self.act.energy([0,self.z0,0],[0,0,0]);self.pitch_inertia=note['pieceMassKg']*((cad['note']['majorRadius']/1000)**2/2+5*(cad['note']['tubeRadius']/1000)**2/8)
  self.start_mechanical=self.mechanical(self.s,self.contact(self.s,{}))
 def geometry(self,s,cache):
  d=s['chassis'];n=s['note'];heading=d['heading']
  if max(abs(heading),abs(n['yaw']-heading))>1e-8 or max(abs(d['vy']),abs(d['yaw_rate']),abs(n['v'][1]),abs(n['spin']))>1e-8:raise LiftDomainError('UNSUPPORTED_NONPLANAR_STATE')
  if abs(s['zPivot']-self.z0)>.02 or abs(s['theta'])>.02 or abs(s['notePitch'])>.02 or not .178<=n['p'][0]-d['x']<=.20:raise LiftDomainError('EXPERIMENT_MOTION_ENVELOPE')
  key=(d['x'],d['y'],heading,*n['p'],n['yaw'],s['zPivot'],s['theta'],s['noteZ'],s['notePitch'])
  if key not in cache:
   origin=[d['x']+.178*math.cos(heading),d['y']+.178*math.sin(heading),s['zPivot']]
   cache[key]=arrays.prepare(self.cad,self.binding['effectiveGeometryMm'],note_center=[*n['p'],s['noteZ']],note_basis=rotation(n['yaw'],s['notePitch']),cradle_origin=origin,cradle_basis=rotation(heading,s['theta']),resolution=tuple(self.c['resolution']),k_area=1e7)
  return cache[key]
 def contact(self,s,cache,rotors=None):
  d=s['chassis'];n=s['note'];p=self.geometry(s,cache);nv=[*n['v'],s['noteVz']];nw=[0.,-s['notePitchRate'],n['spin']];cw=[0.,-s['thetaRate'],d['yaw_rate']]
  q=arrays.rates(p,note_velocity=nv,note_omega=nw,cradle_velocity=[d['vx'],d['vy'],s['vPivot']],cradle_omega=cw,rotor_omega=n['w'] if rotors is None else rotors,pitch_radius_m=self.nc['radiusM'],mu=self.nc['beltMu'],slip_scale=self.nc['slipRegularizationMps'])
  floor_force=np.zeros(3);floor_moment=np.zeros(3);floor_energy=floor_heat=floor_slip=0.;nb=np.array(rotation(n['yaw'],s['notePitch']));major=self.cad['note']['majorRadius']/1000;tube=self.cad['note']['tubeRadius']/1000
  for j in range(self.c['floorPoints']):
   angle=2*math.pi*j/self.c['floorPoints'];ring=nb@np.array([major*math.cos(angle),major*math.sin(angle),0.]);lever=ring+np.array([0,0,-tube]);depth=max(0.,-(s['noteZ']+lever[2]))
   if depth==0:continue
   velocity=np.array(nv)+np.cross(nw,lever);k=self.c['floorStiffnessNpm']/self.c['floorPoints'];b=self.c['floorDampingNspm']/self.c['floorPoints'];compression=max(0.,-velocity[2]);normal=k*depth+b*compression;speed=math.hypot(*velocity[:2]);factor=0. if speed==0 else -self.nc['groundMu']*normal*math.tanh(speed/self.nc['slipRegularizationMps'])/speed;force=np.array([factor*velocity[0],factor*velocity[1],normal]);floor_force+=force;floor_moment+=np.cross(lever,force);floor_energy+=.5*k*depth*depth;floor_heat+=b*compression**2;floor_slip-=force[0]*velocity[0]+force[1]*velocity[1]
  force=np.array(q['noteForceWorldN'])+floor_force;moment=np.array(q['noteMomentWorldNm'])+floor_moment
  discarded=[force[1],moment[0],moment[2],q['cradleReactionWorldN'][1],q['cradleMomentWorldNm'][0],q['cradleMomentWorldNm'][2]]
  if max(map(abs,discarded))>1e-8:raise LiftDomainError('UNSUPPORTED_NONPLANAR_WRENCH')
  return dict(pressure=p,q=q,force=force,moment=moment,floorForce=floor_force,floorEnergy=floor_energy,floorHeat=floor_heat,floorSlip=floor_slip,discarded=discarded)
 def electrical(self,s,bus,cmd):
  d=s['chassis'];n=s['note'];de=self.drive.electrical(bus,cmd['drive'],modes=cmd['driveModes'],state=d);pe=[motor_electrical(duty,bus,w,self.nc) for duty,w in zip(cmd['pinch'],n['w'])];ae=self.act.electrical(bus,cmd['actuator'],cmd['actuatorModes'],[d['x'],s['zPivot'],s['theta']],[d['vx'],s['vPivot'],s['thetaRate']])
  return de,pe,ae
 def rates(self,s,bus,cmd,cache,contact=None):
  d=s['chassis'];n=s['note'];con=self.contact(s,cache) if contact is None else contact;q=con['q'];de,pe,ae=self.electrical(s,bus,cmd)
  dr=self.drive._rates(d,bus,cmd['drive'],cmd['driveModes'],self.loads,[0.,0.],0.)
  terms=self.act.terms([d['x'],s['zPivot'],s['theta']],[d['vx'],s['vPivot'],s['thetaRate']]);matrix=np.array(terms['massMatrix']);matrix[0,0]+=self.drive.c['massKg'];external=np.array([dr['contactForceBodyN'][0]+q['cradleReactionWorldN'][0],q['cradleReactionWorldN'][2],-q['cradleMomentWorldNm'][1]]);acc=np.linalg.solve(matrix,np.array(terms['biasForce'])+np.array(ae['generalizedMotorForce'])+external)
  energy=dict(driveWinding=dr['energyRates']['windingHeat'],driveBrake=dr['energyRates']['controllerBrakeHeat'],driveDrag=dr['energyRates']['motorDragHeat']+dr['energyRates']['gearDragHeat'],tireSlip=dr['energyRates']['tireSlipHeat'],pinchWinding=sum(x['windingHeatW'] for x in pe),normalHeat=0.,beltSlip=q['slipHeatW'],groundHeat=0.,actuatorWinding=sum(x['windingHeatW'] for x in ae['motors']),actuatorBrake=sum(x['brakeHeatW'] for x in ae['motors']),floorNormalHeat=con['floorHeat'],floorFrictionHeat=con['floorSlip'])
  return dict(acc=acc.tolist(),noteAccel=[float(con['force'][0]/self.nc['pieceMassKg']),float(con['force'][2]/self.nc['pieceMassKg']-9.80665)],notePitchAccel=float(-con['moment'][1]/self.pitch_inertia),pinchAccel=[(e['torqueNm']+t)/self.nc['rotorInertiaKgM2'] for e,t in zip(pe,q['rotorContactTorqueNm'])],wheelAccel=dr['dw'],powerW=de['busPowerW']+sum(x['terminalPowerW'] for x in pe)+ae['busPowerW'],energyRates=energy,contact=con,electrical=dict(drive=de,pinch=pe,actuator=ae),infeasible=de['infeasibleCurrentLimitVoltage'] or ae['infeasibleCurrentLimitVoltage'])
 def shifted(self,s,r,dt):
  v=copy.deepcopy(s);v['timeS']+=dt;d=v['chassis'];d['t']+=dt;old=s['chassis'];n=v['note'];d['x']+=dt*old['vx'];d['y']+=dt*old['vy'];d['heading']+=dt*old['yaw_rate'];d['vx']+=dt*r['acc'][0];d['wheel']=[w+dt*a for w,a in zip(old['wheel'],r['wheelAccel'])];d['wheel_angle']=[x+dt*w for x,w in zip(old['wheel_angle'],old['wheel'])];n['p']=[x+dt*u for x,u in zip(s['note']['p'],s['note']['v'])];n['v'][0]+=dt*r['noteAccel'][0];n['yaw']+=dt*s['note']['spin'];n['w']=[w+dt*a for w,a in zip(s['note']['w'],r['pinchAccel'])]
  for key,rate in [('zPivot',s['vPivot']),('vPivot',r['acc'][1]),('theta',s['thetaRate']),('thetaRate',r['acc'][2]),('noteZ',s['noteVz']),('noteVz',r['noteAccel'][1]),('notePitch',s['notePitchRate']),('notePitchRate',r['notePitchAccel'])]:v[key]+=dt*rate
  return v
 def mechanical(self,s,con):
  d=s['chassis'];n=s['note'];mechanism=self.act.energy([d['x'],s['zPivot'],s['theta']],[d['vx'],s['vPivot'],s['thetaRate']])-self.potential0
  note=.5*self.nc['pieceMassKg']*(sum(x*x for x in n['v'])+s['noteVz']**2)+.5*self.nc['yawInertiaKgM2']*n['spin']**2+.5*self.pitch_inertia*s['notePitchRate']**2+.5*self.nc['rotorInertiaKgM2']*sum(x*x for x in n['w'])+self.nc['pieceMassKg']*9.80665*(s['noteZ']-self.z0)
  return self.drive.kinetic(d)+.5*7*d['vy']**2+mechanism+note+con['pressure']['potentialJ']+con['floorEnergy']
 def step(self,dt,cmd):
  if dt not in [.0005,.00025,.000125]:raise ValueError('declared timestep')
  cache={};start_contact=self.contact(self.s,cache)
  # The electrical port needs only start-state accelerations to obtain midpoint
  # rotor speeds. Contact forces at that start state are independent of bus V.
  def port(bus):
   r=self.rates(self.s,bus,cmd,cache,start_contact);mid=self.shifted(self.s,r,dt/2);de,pe,ae=self.electrical(mid,bus,cmd)
   return {'busPowerW':de['busPowerW']+sum(x['terminalPowerW'] for x in pe)+ae['busPowerW']}
  supply=solve_supply({'native-pressure-lift':port},12.,.02);first=self.rates(self.s,supply['busV'],cmd,cache,start_contact);mid=self.shifted(self.s,first,dt/2);second=self.rates(mid,supply['busV'],cmd,cache);end=self.shifted(self.s,second,dt)
  for key,vel in [('zPivot','vPivot'),('theta','thetaRate'),('noteZ','noteVz'),('notePitch','notePitchRate')]:end[key]=self.s[key]+dt*mid[vel]
  end['chassis']['x']=self.s['chassis']['x']+dt*mid['chassis']['vx'];end['chassis']['wheel_angle']=[x+dt*w for x,w in zip(self.s['chassis']['wheel_angle'],mid['chassis']['wheel'])];end['note']['p'][0]=self.s['note']['p'][0]+dt*mid['note']['v'][0]
  final=self.contact(end,cache)
  if first['infeasible'] or second['infeasible']:raise LiftDomainError('ACTUATOR_VOLTAGE_DOMAIN')
  rates=dict(second['energyRates'],sourceWork=supply['sourcePowerW'],sourceHeat=supply['sourceHeatW']);ledger={k:self.ledger[k]+dt*rates[k] for k in self.ledger};mechanical=self.mechanical(end,final);residual=self.initial['initialEnergyJ']+ledger['sourceWork']-sum(v for k,v in ledger.items() if k!='sourceWork')-mechanical
  end['ledger']=ledger;self.ledger=ledger;self.s=end
  return dict(timeS=end['timeS'],state=copy.deepcopy(end),busV=supply['busV'],sourcePowerResidualW=supply['powerResidualW'],portMidpointPowerDifferenceW=supply['busPowerW']-second['powerW'],pressureWorkResidualW=final['q']['workResidualW'],noteContactForceN=final['q']['noteForceWorldN'],floorForceN=final['floorForce'].tolist(),normalN=[x['normalMagnitudeN'] for x in final['pressure']['sideContacts']],contactCount=final['pressure']['contacts'],maximumPenetrationM=final['pressure']['maximumPenetrationM'],energyJ=dict(ledger,mechanical=mechanical,pressure=final['pressure']['potentialJ'],floor=final['floorEnergy'],residual=residual),acceptance=False)

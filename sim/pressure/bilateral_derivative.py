"""Draft actual acquisition derivative with reciprocal bilateral mechanism inertia.
No trajectory/checkpoint adoption: evaluate rates and energy only.
"""
import copy,math
from dataclasses import dataclass,fields
import cassette_coupling as components
import bilateral_geometry as geometry
import bilateral_transport as transport
import cassette_inertia as inertia
from pressure_acquisition import rotate,cross,DomainExit,rotation

@dataclass(frozen=True)
class Mechanism:
 up: float=0.
 udp: float=0.
 tp: float=0.
 tdp: float=0.
 un: float=0.
 udn: float=0.
 tn: float=0.
 tdn: float=0.
 def validate(self):
  for f in fields(self):components.cam.scalar(f.name,getattr(self,f.name))
  for u,ud,t in [(self.up,self.udp,self.tp),(self.un,self.udn,self.tn)]:
   if not 0<=u<=.0015 or not 0<=t<=math.pi:raise ValueError('mechanism domain')
   if (u==0 and ud<0) or (u==.0015 and ud>0):raise ValueError('moving end-stop impact unresolved')

def prepared(plant,d,n,m,cache):
 m.validate();key=tuple(n['p'])+(n['yaw'],d['x'],d['y'],d['heading'],m.up,m.un)
 if key not in cache:
  relative=rotate([n['p'][0]-d['x'],n['p'][1]-d['y']],-d['heading'])
  from pressure_acquisition import check_contacts
  check_contacts(plant.geometry,relative)
  stage=plant.cad['stageCenterX']/1000;z=plant.cad['floorCenterZ']/1000
  origin=[d['x']+stage*math.cos(d['heading']),d['y']+stage*math.sin(d['heading']),z]
  cache[key]=geometry.prepare(plant.cad,note_center=[*n['p'],z],note_basis=rotation(n['yaw'],0.),cradle_origin=origin,cradle_basis=rotation(d['heading'],0.),u_positive_m=m.up,u_negative_m=m.un,resolution=plant.resolution,k_area=plant.k_area)
 return cache[key]

def acquisition_rates(plant,n,d,bus,duties,mechanism,cache):
 c=plant.c;p=prepared(plant,d,n,mechanism,cache);stage=plant.cad['stageCenterX']/1000;offset=rotate([stage,0.],d['heading']);cv=[d['vx']-d['yaw_rate']*offset[1],d['vy']+d['yaw_rate']*offset[0],0.]
 q=transport.rates(p,note_velocity=[*n['v'],0.],note_omega=[0.,0.,n['spin']],cradle_velocity=cv,cradle_omega=[0.,0.,d['yaw_rate']],rotor_omega=n['w'],u_velocities=[mechanism.udp,mechanism.udn],pitch_radius_m=c['radiusM'],mu=c['beltMu'],slip_scale=c['slipRegularizationMps'])
 discarded=[q['noteForceWorldN'][2],*q['noteMomentWorldNm'][:2],q['cradleReactionWorldN'][2],*q['cradleMomentWorldNm'][:2]]
 if max(map(abs,discarded))>1e-8:raise DomainExit('UNSUPPORTED_NONPLANAR_PRESSURE_WRENCH')
 kv=c['kvRadpsPerV'];res=c['motorResistanceOhm'];amps=[0. if duty==0 else max(0.,min(c['currentLimitA'],(duty*bus-w/kv)/res)) for duty,w in zip(duties,n['w'])];terminal=[(w/kv+i*res)*i for w,i in zip(n['w'],amps)]
 if min(terminal)<-1e-10:raise DomainExit('Unmodeled pinch regeneration')
 force=q['noteForceWorldN'][:2];torque=q['noteMomentWorldNm'][2];ground_heat=0.
 for rx,ry in [(c['groundRingRadiusM'],0),(-c['groundRingRadiusM'],0),(0,c['groundRingRadiusM']),(0,-c['groundRingRadiusM'])]:
  vx=n['v'][0]-n['spin']*ry;vy=n['v'][1]+n['spin']*rx;speed=math.hypot(vx,vy);factor=0. if speed==0 else -c['groundMu']*c['pieceMassKg']*9.80665/4*math.tanh(speed/c['slipRegularizationMps'])/speed;f=[factor*vx,factor*vy];force=[x+y for x,y in zip(force,f)];torque+=rx*f[1]-ry*f[0];ground_heat-=f[0]*vx+f[1]*vy
 rows=[]
 for index in [0,1]:
  cells=[cell for cell in p['contacts'] if cell['motor']==index]
  if cells:rows.append(dict(motor=index,normalMagnitudeN=sum(cell['normalN'] for cell in cells),tangentialN=q['sideBeltAxisTractionN'][index],activeSamples=len(cells),activeAreaM2=sum(cell['areaM2'] for cell in cells)))
 reaction=q['cradleReactionWorldN'][:2];relative=rotate([n['p'][0]-d['x'],n['p'][1]-d['y']],-d['heading'])
 return dict(cassetteContact=q['cassetteGeneralizedForceN'],accel=[f/c['pieceMassKg'] for f in force],spinAccel=torque/c['yawInertiaKgM2'],rotorAccel=[(i/kv+t)/c['rotorInertiaKgM2'] for i,t in zip(amps,q['rotorContactTorqueNm'])],reactionForceWorldN=reaction,reactionYawNm=q['cradleMomentWorldNm'][2]+cross(offset,reaction),contacts=rows,contactTorqueNm=q['rotorContactTorqueNm'],springJ=p['potentialJ'],busPowerW=sum(terminal),windingHeatW=sum(i*i*res for i in amps),normalHeatW=0.,slipHeatW=q['slipHeatW'],groundHeatW=ground_heat,relativeRobotXY=relative,pressureFeatures=p['features'],pressureWorkResidualW=q['workResidualW'],discardedNonplanar=discarded)

def derivative(plant,drive,note,mechanism,bus,drive_demands,pinch_demands,loads,*,modes=('drive','drive'),torques=(0.,0.),partition=None,component_parameters=None,locked=()):
 mechanism.validate();partition=partition or inertia.default_partition();cp=component_parameters or (components.Parameters(),components.Parameters())
 if len(cp)!=2 or len(torques)!=2:raise ValueError('two independent component hypotheses/torques')
 if partition.total_mass!=plant.drive.c['massKg'] or partition.total_yaw_inertia!=plant.drive.c['yawInertiaKgM2']:raise ValueError('existing aggregate identity mismatch')
 a=acquisition_rates(plant,note,drive,bus,pinch_demands,mechanism,{})
 b=plant.drive._rates(drive,bus,drive_demands,modes,loads,a['reactionForceWorldN'],a['reactionYawNm'])
 generalized=[b['dvx']*partition.total_mass,b['dvy']*partition.total_mass,b['dyaw']*partition.total_yaw_inertia];uf=[];tf=[];stored=0.;heat=0.;power=0.;details=[]
 for i,(u,ud,t,td,p,requested) in enumerate([(mechanism.up,mechanism.udp,mechanism.tp,mechanism.tdp,cp[0],torques[0]),(mechanism.un,mechanism.udn,mechanism.tn,mechanism.tdn,cp[1],torques[1])]):
  p.validate();torque=components.cam.scalar('mechanical torque',requested);torque=max(-p.torque_limit,min(p.torque_limit,torque))
  spring=components.spring_state(u,free_length_m=p.spring_free_length);cam=components.cam.evaluate(u,ud,t,td,stiffness_n_m=p.cam_stiffness,damping_n_s_m=p.cam_damping)
  force=spring['pairInwardForceN']+cam['cassette_generalized_force_n']+a['cassetteContact'][i]
  # Stop complementarity is not solved here; reaching a loaded hard stop requires a coupled constraint solve.
  if ud==0 and ((u==0 and force<0) or (u==.0015 and force>0)):raise ValueError('coupled end-stop reaction unresolved')
  uf.append(force);tf.append(torque+cam['cam_generalized_torque_nm']);stored+=spring['storedEnergyJ']+cam['potential_j'];heat+=cam['dissipation_w'];power+=torque*td;details.append(dict(spring=spring,cam=cam,appliedTorque=torque))
 q=[drive['x'],drive['y'],drive['heading'],mechanism.up,mechanism.un,mechanism.tp,mechanism.tn]
 v=[drive['vx'],drive['vy'],drive['yaw_rate'],mechanism.udp,mechanism.udn,mechanism.tdp,mechanism.tdn]
 coupled=inertia.acceleration(partition,q,v,generalized+uf+tf,locked)
 changed=dict(b,dvx=coupled['acceleration'][0],dvy=coupled['acceleration'][1],dyaw=coupled['acceleration'][2])
 wheel=.5*plant.drive.axle_inertia*sum(w*w for w in drive['wheel'])
 kinetic=coupled['kineticJ']+wheel+plant.note_kinetic(note)
 return dict(noteRates=a,driveRates=changed,originalDriveRates=b,mechanismAcceleration=coupled['acceleration'][3:],inertia=coupled,componentDetails=details,kineticJ=kinetic,pressureJ=a['springJ'],componentPotentialJ=stored,totalMechanicalJ=kinetic+a['springJ']+stored,mechanicalActuatorPowerW=power,camHeatW=heat,acceptance=False)

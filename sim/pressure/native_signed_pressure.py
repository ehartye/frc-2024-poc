"""Native-bound flat-floor pressure model with signed finite pinch actuation."""
import copy,math
from signed_pressure_acquisition import SignedPressureAcquisition,signed_current,motor_electrical,rotate,cross,DomainExit,rotation
import native_pressure_arrays as arrays

class NativeSignedPressure(SignedPressureAcquisition):
 def __init__(self,drive,note,cad,binding,resolution=(1024,1024)):
  self.binding=copy.deepcopy(binding)
  super().__init__(drive,note,None,dict(p=[.658,0.],v=[0.,0.],yaw=0.,spin=0.,w=[0.,0.]),cad,resolution=resolution,k_area=1e7)
 def prepared(self,d,n,cache):
  key=tuple(n['p'])+(n['yaw'],d['x'],d['y'],d['heading'])
  if key not in cache:
   relative=rotate([n['p'][0]-d['x'],n['p'][1]-d['y']],-d['heading']);domain=self.binding['corridor']
   if not domain['relativeXM'][0]-1e-12<=relative[0]<=domain['relativeXM'][1]+1e-12 or abs(relative[1])>domain['relativeYAbsM'] or abs(n['yaw']-d['heading'])>domain['relativeYawAbsRad']:raise DomainExit('NATIVE_SAMPLED_FLOOR_CORRIDOR_EXIT')
   stage=self.cad['stageCenterX']/1000;z=self.cad['floorCenterZ']/1000;origin=[d['x']+stage*math.cos(d['heading']),d['y']+stage*math.sin(d['heading']),z]
   cache[key]=arrays.prepare(self.cad,self.binding['effectiveGeometryMm'],note_center=[*n['p'],z],note_basis=rotation(n['yaw'],0.),cradle_origin=origin,cradle_basis=rotation(d['heading'],0.),resolution=self.resolution,k_area=self.k_area)
  return cache[key]
 def acquisition_rates(self,n,d,bus,duties,cache):
  c=self.c;p=self.prepared(d,n,cache);stage=self.cad['stageCenterX']/1000;offset=rotate([stage,0.],d['heading']);cv=[d['vx']-d['yaw_rate']*offset[1],d['vy']+d['yaw_rate']*offset[0],0.]
  q=arrays.rates(p,note_velocity=[*n['v'],0.],note_omega=[0.,0.,n['spin']],cradle_velocity=cv,cradle_omega=[0.,0.,d['yaw_rate']],rotor_omega=n['w'],pitch_radius_m=c['radiusM'],mu=c['beltMu'],slip_scale=c['slipRegularizationMps'])
  discarded=[q['noteForceWorldN'][2],*q['noteMomentWorldNm'][:2],q['cradleReactionWorldN'][2],*q['cradleMomentWorldNm'][:2]]
  if max(map(abs,discarded))>1e-8:raise DomainExit('UNSUPPORTED_NONPLANAR_PRESSURE_WRENCH')
  kv=c['kvRadpsPerV'];res=c['motorResistanceOhm'];amps=[signed_current(duty,bus,w,kv,res,c['currentLimitA']) for duty,w in zip(duties,n['w'])];terminal=[(w/kv+i*res)*i for w,i in zip(n['w'],amps)]
  if min(terminal)<-1e-10:raise DomainExit('Unmodeled pinch regeneration')
  force=q['noteForceWorldN'][:2];torque=q['noteMomentWorldNm'][2];ground_heat=0.
  for rx,ry in [(c['groundRingRadiusM'],0),(-c['groundRingRadiusM'],0),(0,c['groundRingRadiusM']),(0,-c['groundRingRadiusM'])]:
   vx=n['v'][0]-n['spin']*ry;vy=n['v'][1]+n['spin']*rx;speed=math.hypot(vx,vy);factor=0. if speed==0 else -c['groundMu']*c['pieceMassKg']*9.80665/4*math.tanh(speed/c['slipRegularizationMps'])/speed;f=[factor*vx,factor*vy];force=[x+y for x,y in zip(force,f)];torque+=rx*f[1]-ry*f[0];ground_heat-=f[0]*vx+f[1]*vy
  rows=[dict(row,tangentialN=q['sideBeltAxisTractionN'][row['motor']]) for row in p['sideContacts']]
  reaction=q['cradleReactionWorldN'][:2];relative=rotate([n['p'][0]-d['x'],n['p'][1]-d['y']],-d['heading'])
  return dict(accel=[f/c['pieceMassKg'] for f in force],spinAccel=torque/c['yawInertiaKgM2'],rotorAccel=[(i/kv+t)/c['rotorInertiaKgM2'] for i,t in zip(amps,q['rotorContactTorqueNm'])],reactionForceWorldN=reaction,reactionYawNm=q['cradleMomentWorldNm'][2]+cross(offset,reaction),contacts=rows,contactTorqueNm=q['rotorContactTorqueNm'],springJ=p['potentialJ'],busPowerW=sum(terminal),windingHeatW=sum(i*i*res for i in amps),normalHeatW=0.,slipHeatW=q['slipHeatW'],groundHeatW=ground_heat,relativeRobotXY=relative,pressureFeatures=p['features'],pressureWorkResidualW=q['workResidualW'],discardedNonplanar=discarded,pinchElectrical=[motor_electrical(duty,bus,w,c) for duty,w in zip(duties,n['w'])])

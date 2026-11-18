"""Signed nonregenerative pinch extension of the preserved pressure acquisition kernel."""
import pathlib,sys,copy,math
R=pathlib.Path(__file__).resolve().parents[2];sys.path[:0]=[str(R/'sim/integrated'),str(R/'sim/pressure')]
from coupled_acquisition import CoupledAcquisition,rotate,dot,cross,note_shift,check_contacts,DomainExit
from shared_supply import solve_supply
from patch_registration import rotation
import fast_geometry,finite_transport

def signed_current(duty,bus,omega,kv,resistance,limit):
 if duty==0:return 0.
 direction=1. if duty>0 else -1.
 current=direction*max(0.,min(limit,(abs(duty)*bus-direction*omega/kv)/resistance))
 if current!=0 and abs(omega/kv+current*resistance)>bus+1e-9:raise DomainExit('Pinch controller voltage domain')
 return current

def motor_electrical(duty,bus,omega,config):
 if not all(math.isfinite(v) for v in [duty,bus,omega]) or not -1<=duty<=1 or bus<0:raise ValueError('signed motor domain')
 kv=config['kvRadpsPerV'];res=config['motorResistanceOhm'];current=signed_current(duty,bus,omega,kv,res,config['currentLimitA']);emf=omega/kv;terminal=emf+current*res;power=terminal*current
 if power< -1e-10:raise DomainExit('Unmodeled pinch regeneration')
 if current!=0 and abs(terminal)>bus+1e-9:raise DomainExit('Pinch controller voltage domain')
 heat=current*current*res;shaft=current/kv*omega
 return dict(currentA=current,terminalV=terminal,terminalPowerW=power,windingHeatW=heat,shaftPowerW=shaft,torqueNm=current/kv,powerResidualW=power-heat-shaft)

class SignedPressureAcquisition(CoupledAcquisition):
 def __init__(self,drive_config,note_config,geometry,initial_note,cad,*,resolution=(1024,1024),k_area=1e7):
  super().__init__(drive_config,note_config,geometry,initial_note);self.cad=copy.deepcopy(cad);self.resolution=tuple(resolution);self.k_area=k_area
  if initial_note['v']!=[0.,0.] or initial_note['w']!=[0.,0.] or initial_note['spin']!=0 or initial_note['yaw']!=0:raise ValueError('FRESH_REST_START_REQUIRED; no legacy checkpoint import')
  p=self.prepared(self.drive.state,self.note,{})
  if p['potentialJ']!=0 or p['contacts']:raise ValueError('FRESH_NO_CONTACT_START_REQUIRED')
  self.initial_energy=self.drive.kinetic()+self.note_kinetic(self.note)
 def prepared(self,d,n,cache):
  key=tuple(n['p'])+(n['yaw'],d['x'],d['y'],d['heading'])
  if key not in cache:
   relative=rotate([n['p'][0]-d['x'],n['p'][1]-d['y']],-d['heading']);check_contacts(self.geometry,relative)
   stage=self.cad['stageCenterX']/1000;z=self.cad['floorCenterZ']/1000;origin=[d['x']+stage*math.cos(d['heading']),d['y']+stage*math.sin(d['heading']),z]
   cache[key]=fast_geometry.prepare(self.cad,note_center=[*n['p'],z],note_basis=rotation(n['yaw'],0.),cradle_origin=origin,cradle_basis=rotation(d['heading'],0.),resolution=self.resolution,k_area=self.k_area)
  return cache[key]
 def acquisition_rates(self,n,d,bus,duties,cache):
  c=self.c;p=self.prepared(d,n,cache);stage=self.cad['stageCenterX']/1000;offset=rotate([stage,0.],d['heading']);cv=[d['vx']-d['yaw_rate']*offset[1],d['vy']+d['yaw_rate']*offset[0],0.]
  q=finite_transport.rates(p,note_velocity=[*n['v'],0.],note_omega=[0.,0.,n['spin']],cradle_velocity=cv,cradle_omega=[0.,0.,d['yaw_rate']],rotor_omega=n['w'],pitch_radius_m=c['radiusM'],mu=c['beltMu'],slip_scale=c['slipRegularizationMps'])
  discarded=[q['noteForceWorldN'][2],*q['noteMomentWorldNm'][:2],q['cradleReactionWorldN'][2],*q['cradleMomentWorldNm'][:2]]
  if max(map(abs,discarded))>1e-8:raise DomainExit('UNSUPPORTED_NONPLANAR_PRESSURE_WRENCH')
  kv=c['kvRadpsPerV'];res=c['motorResistanceOhm'];amps=[signed_current(duty,bus,w,kv,res,c['currentLimitA']) for duty,w in zip(duties,n['w'])];terminal=[(w/kv+i*res)*i for w,i in zip(n['w'],amps)]
  if min(terminal)<-1e-10:raise DomainExit('Unmodeled pinch regeneration')
  force=q['noteForceWorldN'][:2];torque=q['noteMomentWorldNm'][2];ground_heat=0.
  for rx,ry in [(c['groundRingRadiusM'],0),(-c['groundRingRadiusM'],0),(0,c['groundRingRadiusM']),(0,-c['groundRingRadiusM'])]:
   vx=n['v'][0]-n['spin']*ry;vy=n['v'][1]+n['spin']*rx;speed=math.hypot(vx,vy);factor=0. if speed==0 else -c['groundMu']*c['pieceMassKg']*9.80665/4*math.tanh(speed/c['slipRegularizationMps'])/speed;f=[factor*vx,factor*vy];force=[x+y for x,y in zip(force,f)];torque+=rx*f[1]-ry*f[0];ground_heat-=f[0]*vx+f[1]*vy
  rows=[]
  for index in [0,1]:
   cells=[cell for cell in p['contacts'] if cell['motor']==index]
   if cells:rows.append(dict(motor=index,normalMagnitudeN=sum(cell['normalN'] for cell in cells),tangentialN=q['sideBeltAxisTractionN'][index],activeSamples=len(cells),activeAreaM2=sum(cell['areaM2'] for cell in cells)))
  reaction=q['cradleReactionWorldN'][:2];relative=rotate([n['p'][0]-d['x'],n['p'][1]-d['y']],-d['heading'])
  return dict(accel=[f/c['pieceMassKg'] for f in force],spinAccel=torque/c['yawInertiaKgM2'],rotorAccel=[(i/kv+t)/c['rotorInertiaKgM2'] for i,t in zip(amps,q['rotorContactTorqueNm'])],reactionForceWorldN=reaction,reactionYawNm=q['cradleMomentWorldNm'][2]+cross(offset,reaction),contacts=rows,contactTorqueNm=q['rotorContactTorqueNm'],springJ=p['potentialJ'],busPowerW=sum(terminal),windingHeatW=sum(i*i*res for i in amps),normalHeatW=0.,slipHeatW=q['slipHeatW'],groundHeatW=ground_heat,relativeRobotXY=relative,pressureFeatures=p['features'],pressureWorkResidualW=q['workResidualW'],discardedNonplanar=discarded,pinchElectrical=[motor_electrical(duty,bus,w,c) for duty,w in zip(duties,n['w'])])
 def preview(self,dt,drive_demands,pinch_demands,bus,loads,modes,cache):
  def rates(d,n):
   a=self.acquisition_rates(n,d,bus,pinch_demands,cache);b=self.drive._rates(d,bus,drive_demands,modes,loads,a['reactionForceWorldN'],a['reactionYawNm']);return a,b
  a,b=rates(self.drive.state,self.note);dm=self.drive._shift(self.drive.state,b,dt/2);nm=note_shift(self.note,a,dt/2);am,bm=rates(dm,nm);dn=self.drive._shift(self.drive.state,bm,dt);dn['wheel_angle']=[x+dt*w for x,w in zip(self.drive.state['wheel_angle'],dm['wheel'])];nn=note_shift(self.note,am,dt);nn['p']=[x+dt*v for x,v in zip(self.note['p'],nm['v'])];nn['yaw']=self.note['yaw']+dt*nm['spin']
  return dict(note=nn,drive=dn,a=am,b=bm,startA=a,startB=b)
 def make_port(self,dt,drive_demands,pinch_demands,loads,modes,cache):
  d=copy.deepcopy(self.drive.state);n=copy.deepcopy(self.note);c=self.c;dc=self.drive.c;drive_demands=tuple(drive_demands);pinch_demands=tuple(pinch_demands);modes=tuple(modes)
  a=self.acquisition_rates(n,d,0.,pinch_demands,cache);b=self.drive._rates(d,0.,drive_demands,modes,loads,a['reactionForceWorldN'],a['reactionYawNm']);dm=self.drive._shift(d,b,dt/2);nm=note_shift(n,a,dt/2);self.prepared(dm,nm,cache)
  reactions=[0.,0.]
  for g in b['contacts']:reactions[g['side']]-=dc['wheelRadiusM']*g['forceBodyN'][0]
  def port(bus):
   e=self.drive.electrical(bus,drive_demands,modes=modes,state=d);wheels=[]
   for side in range(2):
    wheel=d['wheel'][side];motor=wheel*dc['gearRatio'];motor_drag=2*dc['motorViscousDragNmPerRadps']*motor;gear_drag=dc['gearViscousDragNmPerRadps']*wheel;torque=sum(r['electromagneticTorqueNm'] for r in e['perMotor'][side*2:side*2+2])*dc['gearRatio'];dw=(torque-motor_drag*dc['gearRatio']-gear_drag+reactions[side])/self.drive.axle_inertia;wheels.append(wheel+dt/2*dw)
   drive=self.drive.electrical(bus,drive_demands,modes=modes,state=dict(dm,wheel=wheels));kv=c['kvRadpsPerV'];res=c['motorResistanceOhm'];amps=[signed_current(duty,bus,w,kv,res,c['currentLimitA']) for duty,w in zip(pinch_demands,n['w'])];wmid=[w+dt/2*((i/kv+t)/c['rotorInertiaKgM2']) for w,i,t in zip(n['w'],amps,a['contactTorqueNm'])];ia=[signed_current(duty,bus,w,kv,res,c['currentLimitA']) for duty,w in zip(pinch_demands,wmid)];terminal=[(w/kv+i*res)*i for w,i in zip(wmid,ia)]
   if min(terminal)<-1e-10:raise DomainExit('Unmodeled pinch regeneration')
   return dict(busPowerW=sum(terminal)+drive['busPowerW'])
  return port
 def step(self,dt,drive_demands,pinch_demands,loads,source_v=12.,source_r=.02,modes=('drive','drive')):
  self.drive._validate_step(dt,loads,(0.,0.),0.)
  if len(pinch_demands)!=2 or any(type(d) not in (int,float) or not math.isfinite(d) or not -1<=d<=1 for d in pinch_demands):raise ValueError('pinch duties')
  cache={};port=self.make_port(dt,drive_demands,pinch_demands,loads,modes,cache);supply=solve_supply({'robot':port},source_v,source_r);p=self.preview(dt,drive_demands,pinch_demands,supply['busV'],loads,modes,cache)
  if p['b']['electrical']['infeasibleCurrentLimitVoltage'] or p['startB']['electrical']['infeasibleCurrentLimitVoltage']:raise DomainExit('Drive controller voltage domain')
  end=self.acquisition_rates(p['note'],p['drive'],supply['busV'],pinch_demands,cache);b=p['b']['energyRates'];a=p['a'];rates=dict(sourceWork=supply['sourcePowerW'],sourceHeat=supply['sourceHeatW'],driveWinding=b['windingHeat'],driveBrake=b['controllerBrakeHeat'],driveDrag=b['motorDragHeat']+b['gearDragHeat'],tireSlip=b['tireSlipHeat'],pinchWinding=a['windingHeatW'],normalHeat=a['normalHeatW'],beltSlip=a['slipHeatW'],groundHeat=a['groundHeatW']);ledger={k:self.energy[k]+dt*v for k,v in rates.items()}
  kinetic=self.drive.kinetic(p['drive'])+self.note_kinetic(p['note']);residual=self.initial_energy+ledger['sourceWork']-sum(v for k,v in ledger.items() if k!='sourceWork')-kinetic-end['springJ'];self.energy=ledger;self.drive.state=p['drive'];self.note=p['note'];self.t+=dt
  return dict(timeS=self.t,chassis=copy.deepcopy(self.drive.state),note=copy.deepcopy(self.note),relativeRobotXY=end['relativeRobotXY'],busV=supply['busV'],busCurrentA=supply['busCurrentA'],contacts=a['contacts'],reactionForceWorldN=a['reactionForceWorldN'],reactionYawNm=a['reactionYawNm'],pressureFeatures=end['pressureFeatures'],pressureWorkResidualW=end['pressureWorkResidualW'],discardedNonplanar=end['discardedNonplanar'],pinchElectrical=a['pinchElectrical'],sourcePowerResidualW=supply['powerResidualW'],sourceVoltageResidualV=supply['voltageResidualV'],energyJ=dict(self.energy,kinetic=kinetic,spring=end['springJ'],initial=self.initial_energy,residual=residual),acceptance=False)

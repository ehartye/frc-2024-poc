"""Persistent planar four-CIM drivetrain, with caller-owned supply and explicit tire slip."""
import math,copy

def finite(v):return type(v) in [int,float] and math.isfinite(v)

def vector(v,n,label):
 if len(v)!=n or any(not finite(x) for x in v):raise ValueError(label)

class DrivePlant:
 def __init__(self,config):
  self.c=copy.deepcopy(config);c=self.c
  positive=['trackM','outerAxleXM','wheelRadiusM','gearRatio','motorK','motorResistanceOhm','phaseCurrentLimitA','massKg','yawInertiaKgM2','motorRotorInertiaKgM2','wheelSideInertiaKgM2','slipRegularizationMps']
  nonnegative=['motorViscousDragNmPerRadps','gearViscousDragNmPerRadps','tireMu']
  if any(not finite(c[k]) or c[k]<=0 for k in positive) or any(not finite(c[k]) or c[k]<0 for k in nonnegative):raise ValueError('finite physical parameter domain')
  if type(c['motorsPerSide']) is not int or c['motorsPerSide']!=2:raise ValueError('four CIM source topology required')
  vector(c['loadFractions'],3,'three normal fractions')
  if min(c['loadFractions'])<0 or abs(sum(c['loadFractions'])-1)>1e-12:raise ValueError('normal fractions sum to one')
  if len(c['contactStationsM'])!=2 or any(len(side)!=3 for side in c['contactStationsM']):raise ValueError('six source contact stations')
  for side in c['contactStationsM']:
   for xy in side:vector(xy,2,'finite source contact station')
  self.axle_inertia=c['wheelSideInertiaKgM2']+2*c['motorRotorInertiaKgM2']*c['gearRatio']**2
  self.state=dict(t=0.,x=0.,y=0.,heading=0.,vx=0.,vy=0.,yaw_rate=0.,wheel=[0.,0.],wheel_angle=[0.,0.])
  self.energy=dict(terminalWork=0.,windingHeat=0.,controllerBrakeHeat=0.,motorDragHeat=0.,gearDragHeat=0.,tireSlipHeat=0.,externalWork=0.)
  self.initial_kinetic=0.
 def kinetic(self,s=None):
  s=self.state if s is None else s;c=self.c
  return .5*c['massKg']*(s['vx']**2+s['vy']**2)+.5*c['yawInertiaKgM2']*s['yaw_rate']**2+.5*self.axle_inertia*sum(w*w for w in s['wheel'])
 def electrical(self,bus_v,demands,*,modes,state=None):
  if not finite(bus_v) or bus_v<0:raise ValueError('nonnegative finite bus voltage')
  vector(demands,2,'two finite signed demands')
  if any(abs(d)>1 for d in demands) or len(modes)!=2 or any(m not in ['drive','coast','brake'] for m in modes):raise ValueError('explicit motor modes/demands')
  s=self.state if state is None else state;c=self.c;rows=[]
  for side in range(2):
   omega=s['wheel'][side]*c['gearRatio'];emf=c['motorK']*omega;mode=modes[side];requested=demands[side]*bus_v
   if mode=='coast':amps=0.;terminal=None;brake_heat=0.;bus_power=0.;infeasible=False
   else:
    target=0. if mode=='brake' else requested
    amps=max(-c['phaseCurrentLimitA'],min(c['phaseCurrentLimitA'],(target-emf)/c['motorResistanceOhm']))
    terminal=emf+amps*c['motorResistanceOhm'];terminal_power=terminal*amps
    brake_heat=max(0.,-terminal_power) if mode=='brake' else 0.;bus_power=0. if mode=='brake' else terminal_power
    infeasible=mode=='drive' and abs(terminal)>bus_v+1e-9
   for index in range(2):rows.append(dict(side='left' if side==0 else 'right',motor=index,mode=mode,currentA=amps,actualTerminalV=terminal,backEmfV=emf,omegaRadps=omega,electromagneticTorqueNm=c['motorK']*amps,busPowerW=bus_power,windingHeatW=amps*amps*c['motorResistanceOhm'],controllerBrakeHeatW=brake_heat,infeasibleCurrentLimitVoltage=infeasible))
  power=sum(x['busPowerW'] for x in rows)
  return dict(busPowerW=power,busCurrentA=power/bus_v if bus_v>0 else None,perMotor=rows,windingHeatW=sum(x['windingHeatW'] for x in rows),controllerBrakeHeatW=sum(x['controllerBrakeHeatW'] for x in rows),infeasibleCurrentLimitVoltage=any(x['infeasibleCurrentLimitVoltage'] for x in rows),stateTimeS=s['t'])
 def _rates(self,s,bus_v,demands,modes,normal_loads,external_force_world,external_yaw_torque):
  c=self.c;e=self.electrical(bus_v,demands,modes=modes,state=s);co,si=math.cos(s['heading']),math.sin(s['heading']);vx=co*s['vx']+si*s['vy'];vy=-si*s['vx']+co*s['vy'];fx=fy=torque=slip_heat=0.;wheel_reaction=[0.,0.];contacts=[]
  for side in range(2):
   for (x,y),weight in zip(c['contactStationsM'][side],c['loadFractions']):
    long=c['wheelRadiusM']*s['wheel'][side]-(vx-s['yaw_rate']*y);lateral=-(vy+s['yaw_rate']*x);speed=math.hypot(long,lateral);normal=normal_loads[side]*weight
    factor=0. if speed==0 else c['tireMu']*normal*math.tanh(speed/c['slipRegularizationMps'])/speed
    tx,ty=factor*long,factor*lateral;fx+=tx;fy+=ty;torque+=x*ty-y*tx;wheel_reaction[side]-=c['wheelRadiusM']*tx;slip_heat+=tx*long+ty*lateral
    contacts.append(dict(side=side,xM=x,yM=y,normalN=normal,forceBodyN=[tx,ty],slipMps=[long,lateral]))
  motor_heat=gear_heat=0.;dw=[]
  for side in range(2):
   wheel=s['wheel'][side];motor=wheel*c['gearRatio'];motor_drag=2*c['motorViscousDragNmPerRadps']*motor;gear_drag=c['gearViscousDragNmPerRadps']*wheel
   drive_torque=sum(r['electromagneticTorqueNm'] for r in e['perMotor'][side*2:side*2+2])*c['gearRatio']
   dw.append((drive_torque-motor_drag*c['gearRatio']-gear_drag+wheel_reaction[side])/self.axle_inertia);motor_heat+=motor_drag*motor;gear_heat+=gear_drag*wheel
  wx=co*fx-si*fy;wy=si*fx+co*fy
  rates=dict(terminalWork=e['busPowerW'],windingHeat=e['windingHeatW'],controllerBrakeHeat=e['controllerBrakeHeatW'],motorDragHeat=motor_heat,gearDragHeat=gear_heat,tireSlipHeat=slip_heat,externalWork=external_force_world[0]*s['vx']+external_force_world[1]*s['vy']+external_yaw_torque*s['yaw_rate'])
  return dict(dx=s['vx'],dy=s['vy'],dh=s['yaw_rate'],dvx=(wx+external_force_world[0])/c['massKg'],dvy=(wy+external_force_world[1])/c['massKg'],dyaw=(torque+external_yaw_torque)/c['yawInertiaKgM2'],dw=dw,contactForceBodyN=[fx,fy],contactYawTorqueNm=torque,contacts=contacts,electrical=e,energyRates=rates)
 def _validate_step(self,dt,normal_loads,external_force_world,external_yaw_torque):
  if not finite(dt) or not 0<dt<=.0025:raise ValueError('step must be positive and <=2.5ms')
  vector(normal_loads,2,'two normal loads');vector(external_force_world,2,'world force')
  if min(normal_loads)<0 or not finite(external_yaw_torque):raise ValueError('load domain')
 def _shift(self,s,r,dt):
  n=copy.deepcopy(s)
  for key,rate in [('x','dx'),('y','dy'),('heading','dh'),('vx','dvx'),('vy','dvy'),('yaw_rate','dyaw')]:n[key]+=dt*r[rate]
  n['t']+=dt;n['wheel']=[w+dt*a for w,a in zip(s['wheel'],r['dw'])];n['wheel_angle']=[a+dt*w for a,w in zip(s['wheel_angle'],s['wheel'])];return n
 def preview(self,dt,demands,bus_v,normal_loads,*,modes,external_force_world=(0.,0.),external_yaw_torque=0.,state=None):
  self._validate_step(dt,normal_loads,external_force_world,external_yaw_torque);s=self.state if state is None else state
  first=self._rates(s,bus_v,demands,modes,normal_loads,external_force_world,external_yaw_torque);mid=self._shift(s,first,dt/2)
  rates=self._rates(mid,bus_v,demands,modes,normal_loads,external_force_world,external_yaw_torque)
  final=self._shift(s,rates,dt);final['wheel_angle']=[a+dt*w for a,w in zip(s['wheel_angle'],mid['wheel'])]
  return dict(nextState=final,midpoint=rates,startElectrical=first['electrical'])
 def power_port(self,dt,demands,normal_loads,*,modes,external_force_world=(0.,0.),external_yaw_torque=0.):
  snapshot=copy.deepcopy(self.state)
  def port(bus_v):
   p=self.preview(dt,demands,bus_v,normal_loads,modes=modes,external_force_world=external_force_world,external_yaw_torque=external_yaw_torque,state=snapshot);e=p['midpoint']['electrical'];e['infeasibleCurrentLimitVoltage']=e['infeasibleCurrentLimitVoltage'] or p['startElectrical']['infeasibleCurrentLimitVoltage'];return e
  return port
 def step(self,dt,demands,bus_v,normal_loads,*,modes,external_force_world=(0.,0.),external_yaw_torque=0.):
  p=self.preview(dt,demands,bus_v,normal_loads,modes=modes,external_force_world=external_force_world,external_yaw_torque=external_yaw_torque)
  if p['startElectrical']['infeasibleCurrentLimitVoltage'] or p['midpoint']['electrical']['infeasibleCurrentLimitVoltage']:raise ValueError('unsupported current-limit terminal voltage')
  self.state=p['nextState']
  for k,v in p['midpoint']['energyRates'].items():self.energy[k]+=v*dt
  kinetic=self.kinetic();res=self.initial_kinetic+self.energy['terminalWork']+self.energy['externalWork']-sum(v for k,v in self.energy.items() if k not in ['terminalWork','externalWork'])-kinetic
  return dict(state=copy.deepcopy(self.state),rotorOmegaRadps=[w*self.c['gearRatio'] for w in self.state['wheel']],electrical=p['midpoint']['electrical'],contacts=p['midpoint']['contacts'],energyJ=dict(self.energy,kinetic=kinetic,residual=res),contactForceBodyN=p['midpoint']['contactForceBodyN'],contactYawTorqueNm=p['midpoint']['contactYawTorqueNm'],acceptance=False)

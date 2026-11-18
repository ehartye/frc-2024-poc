"""Planar force-coupled chassis and loose NOTE; one bus, no prescribed robot motion."""
import pathlib,sys,math,copy
R=pathlib.Path(__file__).resolve().parents[2]
sys.path[:0]=[str(R/'sim/drivetrain'),str(R/'sim/acquisition'),str(R/'sim/integrated')]
from plant import DrivePlant
from free_planar import check_contacts,DomainExit
from shared_supply import solve_supply

def rotate(v,h):
 c,s=math.cos(h),math.sin(h);return [c*v[0]-s*v[1],s*v[0]+c*v[1]]
def cross(a,b):return a[0]*b[1]-a[1]*b[0]
def dot(a,b):return sum(x*y for x,y in zip(a,b))

def acquisition_rates(note,drive,c,geometry,bus,duties):
 relative=rotate([note['p'][0]-drive['x'],note['p'][1]-drive['y']],-drive['heading']);contacts=check_contacts(geometry,relative)
 kv=c['kvRadpsPerV'];res=c['motorResistanceOhm'];amps=[0. if duty==0 else max(0.,min(c['currentLimitA'],(duty*bus-w/kv)/res)) for duty,w in zip(duties,note['w'])]
 terminal=[(w/kv+i*res)*i for w,i in zip(note['w'],amps)]
 if min(terminal)<-1e-10:raise DomainExit('Unmodeled pinch regeneration')
 force=[0.,0.];torque=0.;reaction=[0.,0.];reaction_torque=0.;rotor_torque=[i/kv for i in amps];spring=normal_heat=slip_heat=ground_heat=0.;rows=[]
 for g in contacts:
  n=rotate(g['normal'],drive['heading']);t=rotate(g['tangent'],drive['heading']);lever=rotate(g['leverM'],drive['heading']);arm=[note['p'][0]+lever[0]-drive['x'],note['p'][1]+lever[1]-drive['y']]
  body_v=[drive['vx']-drive['yaw_rate']*arm[1],drive['vy']+drive['yaw_rate']*arm[0]]
  piece_v=[note['v'][0]-note['spin']*lever[1],note['v'][1]+note['spin']*lever[0]];relative_v=[a-b for a,b in zip(piece_v,body_v)]
  vn=dot(relative_v,n);vt=dot(relative_v,t);compression=max(-vn,0.);delta=g['penetrationM'];idx=g['motor']
  normal=c['normalStiffnessNpm']*delta*(1+c['compressionAlphaSpm']*compression);slip=c['radiusM']*note['w'][idx]-vt;tangent=c['beltMu']*normal*math.tanh(slip/c['slipRegularizationMps']);f=[normal*a+tangent*b for a,b in zip(n,t)]
  force=[a+b for a,b in zip(force,f)];torque+=cross(lever,f);reaction=[a-b for a,b in zip(reaction,f)];reaction_torque-=cross(arm,f);rotor_torque[idx]-=c['radiusM']*tangent
  spring+=.5*c['normalStiffnessNpm']*delta*delta;normal_heat+=c['normalStiffnessNpm']*delta*c['compressionAlphaSpm']*compression**2;slip_heat+=tangent*slip
  rows.append(dict(motor=idx,normalN=normal,tangentialN=tangent,forceWorldN=f,leverWorldM=lever,chassisLeverWorldM=arm,penetrationM=delta))
 # Distributed floor drag; the NOTE is flat and remains ground-supported in this phase.
 for rx,ry in [(c['groundRingRadiusM'],0),(-c['groundRingRadiusM'],0),(0,c['groundRingRadiusM']),(0,-c['groundRingRadiusM'])]:
  vx=note['v'][0]-note['spin']*ry;vy=note['v'][1]+note['spin']*rx;speed=math.hypot(vx,vy);factor=0. if speed==0 else -c['groundMu']*c['pieceMassKg']*9.80665/4*math.tanh(speed/c['slipRegularizationMps'])/speed
  f=[factor*vx,factor*vy];force=[a+b for a,b in zip(force,f)];torque+=rx*f[1]-ry*f[0];ground_heat-=f[0]*vx+f[1]*vy
 return dict(accel=[f/c['pieceMassKg'] for f in force],spinAccel=torque/c['yawInertiaKgM2'],rotorAccel=[v/c['rotorInertiaKgM2'] for v in rotor_torque],reactionForceWorldN=reaction,reactionYawNm=reaction_torque,contacts=rows,springJ=spring,busPowerW=sum(terminal),windingHeatW=sum(i*i*res for i in amps),normalHeatW=normal_heat,slipHeatW=slip_heat,groundHeatW=ground_heat,relativeRobotXY=relative)

def note_shift(s,r,dt):return dict(p=[x+dt*v for x,v in zip(s['p'],s['v'])],v=[v+dt*a for v,a in zip(s['v'],r['accel'])],yaw=s['yaw']+dt*s['spin'],spin=s['spin']+dt*r['spinAccel'],w=[w+dt*a for w,a in zip(s['w'],r['rotorAccel'])])

class CoupledAcquisition:
 def __init__(self,drive_config,note_config,geometry,initial_note):
  self.drive=DrivePlant(drive_config);self.c=copy.deepcopy(note_config);self.geometry=geometry;self.note=copy.deepcopy(initial_note);self.energy={k:0. for k in ['sourceWork','sourceHeat','driveWinding','driveBrake','driveDrag','tireSlip','pinchWinding','normalHeat','beltSlip','groundHeat']};self.t=0.;self.initial_energy=None
 def note_kinetic(self,s):return .5*self.c['pieceMassKg']*dot(s['v'],s['v'])+.5*self.c['yawInertiaKgM2']*s['spin']**2+.5*self.c['rotorInertiaKgM2']*dot(s['w'],s['w'])
 def preview(self,dt,drive_demands,pinch_demands,bus,normal_loads,modes,geometry=None):
  geometry=self.geometry if geometry is None else geometry
  def rates(d,n):
   a=acquisition_rates(n,d,self.c,geometry,bus,pinch_demands)
   b=self.drive._rates(d,bus,drive_demands,modes,normal_loads,a['reactionForceWorldN'],a['reactionYawNm']);return a,b
  a,b=rates(self.drive.state,self.note);dm=self.drive._shift(self.drive.state,b,dt/2);nm=note_shift(self.note,a,dt/2);am,bm=rates(dm,nm)
  dn=self.drive._shift(self.drive.state,bm,dt);dn['wheel_angle']=[x+dt*w for x,w in zip(self.drive.state['wheel_angle'],dm['wheel'])]
  nn=note_shift(self.note,am,dt);nn['p']=[x+dt*v for x,v in zip(self.note['p'],nm['v'])];nn['yaw']=self.note['yaw']+dt*nm['spin']
  return dict(note=nn,drive=dn,a=am,b=bm,startA=a,startB=b)
 def step(self,dt,drive_demands,pinch_demands,normal_loads,source_v=12.,source_r=.02,modes=('drive','drive')):
  self.drive._validate_step(dt,normal_loads,(0.,0.),0.)
  if len(pinch_demands)!=2 or any(type(d) not in (int,float) or not math.isfinite(d) or not 0<=d<=1 for d in pinch_demands):raise ValueError('pinch duties')
  cache={}
  def cached_geometry(q):
   key=tuple(q)
   if key not in cache:cache[key]=self.geometry(q)
   return cache[key]
  def port(v):
   p=self.preview(dt,drive_demands,pinch_demands,v,normal_loads,modes,geometry=cached_geometry);return dict(busPowerW=p['a']['busPowerW']+p['b']['electrical']['busPowerW'])
  supply=solve_supply({'robot':port},source_v,source_r);p=self.preview(dt,drive_demands,pinch_demands,supply['busV'],normal_loads,modes,geometry=cached_geometry)
  if p['b']['electrical']['infeasibleCurrentLimitVoltage'] or p['startB']['electrical']['infeasibleCurrentLimitVoltage']:raise DomainExit('Drive controller voltage domain')
  # Reject a domain exit before mutating ANY persistent state.
  end=acquisition_rates(p['note'],p['drive'],self.c,self.geometry,supply['busV'],pinch_demands)
  if self.initial_energy is None:self.initial_energy=self.drive.kinetic()+self.note_kinetic(self.note)+p['startA']['springJ']
  b=p['b']['energyRates'];a=p['a'];rates=dict(sourceWork=supply['sourcePowerW'],sourceHeat=supply['sourceHeatW'],driveWinding=b['windingHeat'],driveBrake=b['controllerBrakeHeat'],driveDrag=b['motorDragHeat']+b['gearDragHeat'],tireSlip=b['tireSlipHeat'],pinchWinding=a['windingHeatW'],normalHeat=a['normalHeatW'],beltSlip=a['slipHeatW'],groundHeat=a['groundHeatW'])
  for k,v in rates.items():self.energy[k]+=dt*v
  self.drive.state=p['drive'];self.note=p['note'];self.t+=dt;kinetic=self.drive.kinetic()+self.note_kinetic(self.note);residual=self.initial_energy+self.energy['sourceWork']-sum(v for k,v in self.energy.items() if k!='sourceWork')-kinetic-end['springJ']
  return dict(timeS=self.t,chassis=copy.deepcopy(self.drive.state),note=copy.deepcopy(self.note),relativeRobotXY=end['relativeRobotXY'],busV=supply['busV'],busCurrentA=supply['busCurrentA'],contacts=a['contacts'],reactionForceWorldN=a['reactionForceWorldN'],reactionYawNm=a['reactionYawNm'],energyJ=dict(self.energy,kinetic=kinetic,spring=end['springJ'],initial=self.initial_energy,residual=residual),acceptance=False)

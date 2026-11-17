"""Bounded XYZ/yaw native six-wheel ground-contact audit, not whole CAD.
Roll/pitch constrained. Six wheel hinges have four soft 1:1 equalities.
Only reviewed electrical/drag law is reused; no custom tire force or normal loads.
"""
import copy,hashlib,json,math,pathlib
import numpy as np
import mujoco
from drivetrain_bridge import load_source

def xml(c,axle_inertia,wheels_source,dt=.00025,mu=.8):
 mass=.5;r=c['wheelRadiusM'];width=wheels_source[0][0]['sizeMm'][1]/1000
 spin=.5*mass*r*r;trans=mass*(3*r*r+width*width)/12
 armature=axle_inertia/3-spin
 if armature<=0:raise ValueError('physical wheel inertia exceeds side budget')
 stations=[[[p['worldCenterMm'][0]/1000,p['worldCenterMm'][1]/1000] for p in row] for row in wheels_source];base_mass=c['massKg']-6*mass
 bx=-mass*sum(p[0] for row in stations for p in row)/base_mass;by=-mass*sum(p[1] for row in stations for p in row)/base_mass
 yaw=c['yawInertiaKgM2']-sum(trans+mass*(x*x+y*y) for row in stations for x,y in row)-base_mass*(bx*bx+by*by)
 wheels=[];equalities=[]
 for side,row in enumerate(stations):
  for i,(x,y) in enumerate(row):
   name=f'wheel_{side}_{i}';z=wheels_source[side][i]['worldCenterMm'][2]/1000
   wheels.append(f'<body name="{name}" pos="{x} {y} {z}"><joint name="{name}" type="hinge" axis="0 1 0" armature="{armature}"/><inertial pos="0 0 0" mass="{mass}" diaginertia="{trans} {spin} {trans}"/><geom name="{name}" type="cylinder" size="{r} {width/2}" quat=".7071067811865476 .7071067811865476 0 0"/></body>')
   if i!=1:equalities.append(f'<joint joint1="{name}" joint2="wheel_{side}_1" polycoef="0 1 0 0 0" solref=".002 1"/>')
 return f'''<mujoco model="native sixwheel XYZyaw"><compiler angle="radian"/><option timestep="{dt}" gravity="0 0 -9.80665" integrator="implicitfast" solver="Newton" iterations="100" tolerance="1e-10"><flag energy="enable"/></option><default><geom friction="{mu} 0 0" condim="3" solref=".005 1" solimp=".9 .95 .001"/></default><worldbody><geom name="floor" type="plane" size="2 2 .1"/><body name="chassis" pos="0 0 .001"><joint name="x" type="slide" axis="1 0 0"/><joint name="y" type="slide" axis="0 1 0"/><joint name="z" type="slide" axis="0 0 1"/><joint name="yaw" type="hinge" axis="0 0 1"/><inertial pos="{bx} {by} .1762" mass="{base_mass}" diaginertia="3 4 {yaw}"/>{''.join(wheels)}</body></worldbody><equality>{''.join(equalities)}</equality></mujoco>'''

class NativeDrive:
 def __init__(self,root,dt=.00025,mu=.8):
  if not 0<dt<=.001 or not math.isfinite(dt) or not 0<=mu<=2:raise ValueError('bounded dt/friction')
  self.source,self.pins=load_source(root);self.c=self.source.c
  path=pathlib.Path(root)/'cad/drivetrain/candidate.json';raw=path.read_bytes();self.pins['cad/drivetrain/candidate.json']=hashlib.sha256(raw).hexdigest();candidate=json.loads(raw);parts=[p for p in candidate['parts'] if p['id'].startswith('am-0940b-')]
  if len(parts)!=6:raise ValueError('six source wheels required')
  self.wheels_source=[sorted([p for p in parts if p['worldCenterMm'][1]*side>0],key=lambda p:p['worldCenterMm'][0]) for side in [1,-1]]
  self.xml=xml(self.c,self.source.axle_inertia,self.wheels_source,dt,mu);self.m=mujoco.MjModel.from_xml_string(self.xml);self.d=mujoco.MjData(self.m)
  self.q={name:int(self.m.joint(name).qposadr[0]) for name in ['x','y','z','yaw']+[f'wheel_{s}_{i}' for s in range(2) for i in range(3)]};self.v={name:int(self.m.joint(name).dofadr[0]) for name in self.q}
  self.work=0.;self.constraint_work=0.;mujoco.mj_forward(self.m,self.d);self.initial_energy=float(sum(self.d.energy))
 def motor_query(self,demands=(0,0),modes=('coast','coast'),bus=12.):
  omega=[float(self.d.qvel[self.v[f'wheel_{s}_1']]) for s in range(2)];state=dict(wheel=omega,t=float(self.d.time));e=self.source.electrical(bus,demands,modes=modes,state=state)
  if e['infeasibleCurrentLimitVoltage']:raise ValueError('unsupported controller voltage')
  torque=[]
  for s,w in enumerate(omega):
   drive=sum(x['electromagneticTorqueNm'] for x in e['perMotor'][2*s:2*s+2])*self.c['gearRatio'];drag=2*self.c['motorViscousDragNmPerRadps']*w*self.c['gearRatio']**2+self.c['gearViscousDragNmPerRadps']*w;torque.append(drive-drag)
  return e,torque,omega
 def step(self,demands=(0,0),modes=('coast','coast')):
  e,t,w=self.motor_query(demands,modes);self.d.qfrc_applied[:]=0
  for s in range(2):self.d.qfrc_applied[self.v[f'wheel_{s}_1']]=t[s]
  mujoco.mj_forward(self.m,self.d)
  self.work+=self.m.opt.timestep*sum(a*b for a,b in zip(t,w));self.constraint_work+=self.m.opt.timestep*float(self.d.qfrc_constraint@self.d.qvel)
  mujoco.mj_step(self.m,self.d);mujoco.mj_forward(self.m,self.d)
  return e
 def observe(self):
  loads={f'wheel_{s}_{i}':0. for s in range(2) for i in range(3)};penetrations=[]
  for j,c in enumerate(self.d.contact):
   names=[mujoco.mj_id2name(self.m,mujoco.mjtObj.mjOBJ_GEOM,int(g)) for g in [c.geom1,c.geom2]]
   if 'floor' not in names:continue
   f=np.zeros(6);mujoco.mj_contactForce(self.m,self.d,j,f);F=c.frame.reshape(3,3).T@f[:3];wheel=next(n for n in names if n.startswith('wheel_'));sign=1 if names[1]==wheel else -1;loads[wheel]+=sign*F[2];penetrations.append(float(c.dist))
  errors=[float(self.d.qpos[self.q[f'wheel_{s}_{i}']]-self.d.qpos[self.q[f'wheel_{s}_1']]) for s in range(2) for i in [0,2]]
  return dict(timeS=float(self.d.time),qpos=self.d.qpos.tolist(),qvel=self.d.qvel.tolist(),wheelVerticalLoadsN=loads,contactDistancesM=penetrations,equalityAngleErrorsRad=errors,engineEnergyJ=self.d.energy.tolist(),motorAxleWorkJ=self.work,constraintWorkJ=self.constraint_work,discreteWorkResidualJ=float(sum(self.d.energy))-self.initial_energy-self.work-self.constraint_work)

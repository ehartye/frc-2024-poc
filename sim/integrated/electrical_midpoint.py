"""Experimental pure electrical midpoint port; unchanged full-scan solver and reference."""
import copy,math,inspect,textwrap
from coupled_acquisition import CoupledAcquisition,acquisition_rates,rotate,check_contacts
from shared_supply import solve_supply
from free_planar import DomainExit

def make_port(plant,dt,drive_demands,pinch_demands,loads,modes,geometry=None):
 # Full start force evaluations once. They depend on start velocity/contact, not voltage.
 geometry=plant.geometry if geometry is None else geometry
 d=copy.deepcopy(plant.drive.state);n=copy.deepcopy(plant.note);c=copy.deepcopy(plant.c);dc=plant.drive.c
 drive_demands=tuple(drive_demands);pinch_demands=tuple(pinch_demands);modes=tuple(modes)
 a=acquisition_rates(n,d,c,geometry,0.,pinch_demands)
 b=plant.drive._rates(d,0.,drive_demands,modes,loads,a['reactionForceWorldN'],a['reactionYawNm'])
 dm=plant.drive._shift(d,b,dt/2)
 # Midpoint position and heading use start velocity, so are bus-independent.
 np=[x+dt/2*v for x,v in zip(n['p'],n['v'])]
 relative=rotate([np[0]-dm['x'],np[1]-dm['y']],-dm['heading'])
 check_contacts(geometry,relative) # retain finite-geometry/unknown-domain gate
 reactions=[0.,0.]
 for g in b['contacts']:reactions[g['side']]-=dc['wheelRadiusM']*g['forceBodyN'][0]
 def port(bus):
  e=plant.drive.electrical(bus,drive_demands,modes=modes,state=d)
  wheels=[]
  for side in range(2):
   wheel=d['wheel'][side];motor=wheel*dc['gearRatio'];motor_drag=2*dc['motorViscousDragNmPerRadps']*motor;gear_drag=dc['gearViscousDragNmPerRadps']*wheel
   torque=sum(r['electromagneticTorqueNm'] for r in e['perMotor'][side*2:side*2+2])*dc['gearRatio']
   dw=(torque-motor_drag*dc['gearRatio']-gear_drag+reactions[side])/plant.drive.axle_inertia
   wheels.append(wheel+dt/2*dw)
  mid=dict(dm,wheel=wheels)
  drive=plant.drive.electrical(bus,drive_demands,modes=modes,state=mid)
  kv=c['kvRadpsPerV'];res=c['motorResistanceOhm']
  amps=[0. if duty==0 else max(0.,min(c['currentLimitA'],(duty*bus-w/kv)/res)) for duty,w in zip(pinch_demands,n['w'])]
  torques=[i/kv for i in amps]
  for g in a['contacts']:torques[g['motor']]-=c['radiusM']*g['tangentialN']
  wmid=[w+dt/2*(torque/c['rotorInertiaKgM2']) for w,torque in zip(n['w'],torques)]
  ia=[0. if duty==0 else max(0.,min(c['currentLimitA'],(duty*bus-w/kv)/res)) for duty,w in zip(pinch_demands,wmid)]
  terminal=[(w/kv+i*res)*i for w,i in zip(wmid,ia)]
  if min(terminal)<-1e-10:raise DomainExit('Unmodeled pinch regeneration')
  return dict(busPowerW=sum(terminal)+drive['busPowerW'])
 return port

class ElectricalOnlyCoupled(CoupledAcquisition):
 pass

# Experimental derivation: preserve the entire reference step except its voltage-query
# body. This makes any reference change explicit rather than maintaining a drifting copy.
_original=inspect.getsource(CoupledAcquisition.step)
_old="  def port(v):\n   p=self.preview(dt,drive_demands,pinch_demands,v,normal_loads,modes,geometry=cached_geometry);return dict(busPowerW=p['a']['busPowerW']+p['b']['electrical']['busPowerW'])"
_new="  port=make_port(self,dt,drive_demands,pinch_demands,normal_loads,modes,geometry=cached_geometry)"
if _original.count(_old)!=1:raise RuntimeError('Reference step changed; electrical port requires revalidation')
_namespace={}
exec(compile(textwrap.dedent(_original.replace(_old,_new)),__file__,'exec'),globals(),_namespace)
ElectricalOnlyCoupled.step=_namespace['step']

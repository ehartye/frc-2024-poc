"""Virtual rigid-frame observations and bounded longitudinal speed experiment."""
import math

def observe(now, drive, note, bus_v=12.):
 """Derivative of R(-heading)(p_note-p_robot), including rotating-frame transport."""
 c,s=math.cos(drive['heading']),math.sin(drive['heading'])
 dx,dy=note['p'][0]-drive['x'],note['p'][1]-drive['y']
 ux,uy=note['v'][0]-drive['vx'],note['v'][1]-drive['vy']
 x,y=c*dx+s*dy,-s*dx+c*dy
 return dict(timestamp=now,relativeX=x,relativeY=y,
  relativeVx=c*ux+s*uy+drive['yaw_rate']*y,
  relativeVy=-s*ux+c*uy-drive['yaw_rate']*x,
  forwardSpeed=c*drive['vx']+s*drive['vy'],yawRate=drive['yaw_rate'],
  wheelLeft=drive['wheel'][0],wheelRight=drive['wheel'][1],busV=bus_v)

class SpeedApproach:
 """Straight-only, exact virtual feedback. No capture latch or ideal holding."""
 def __init__(self,config,goal=.215,max_speed=.25,max_age=.05,pinch_enabled=True):
  self.c=config;self.goal=goal;self.max_speed=max_speed;self.max_age=max_age
  self.pinch_enabled=pinch_enabled;self.fault=None;self.state='REGULATE'
 def update(self,now,o):
  keys=['timestamp','relativeX','relativeY','relativeVx','relativeVy','forwardSpeed','yawRate','wheelLeft','wheelRight','busV']
  valid=type(now) in (int,float) and math.isfinite(now) and all(type(o.get(k)) in (int,float) and math.isfinite(o[k]) for k in keys)
  if not valid or o['timestamp']>now+1e-9 or now-o['timestamp']>self.max_age or o['busV']<=0:
   self.fault='stale-or-invalid-observation'
  elif abs(o['relativeY'])>.001 or abs(o['yawRate'])>.01:
   self.fault='outside-straight-controller-domain'
  if self.fault:
   self.state='OBSERVATION_FAULT'
   return dict(state=self.state,fault=self.fault,driveDemands=[0.,0.],pinchDemands=[0.,0.],modes=['brake','brake'],desiredSpeed=0.)
  distance=max(0.,o['relativeX']-self.goal)
  # Approach is deliberately slow before contact; close-rate feedback uses the full frame derivative.
  desired=min(self.max_speed,1.5*distance)
  close=-o['relativeVx']; correction=2.*(desired-close)
  # Feedforward wheel back-EMF, with explicit dissipative brake when deceleration is required.
  voltage=self.c['motorK']*self.c['gearRatio']/self.c['wheelRadiusM']*desired+correction
  demands=[];modes=[]
  for wheel in [o['wheelLeft'],o['wheelRight']]:
   emf=self.c['motorK']*self.c['gearRatio']*wheel
   if desired<.003 or voltage<emf+.015:
    demands.append(0.);modes.append('brake')
   else:
    demands.append(min(.2,max(0.,voltage/o['busV'])));modes.append('drive')
  pinch=min(.025,max(0.,distance*.15)) if self.pinch_enabled else 0.
  self.state='REGULATE'
  return dict(state=self.state,fault=None,driveDemands=demands,pinchDemands=[pinch,pinch],modes=modes,desiredSpeed=desired,relativeClosingSpeed=close)

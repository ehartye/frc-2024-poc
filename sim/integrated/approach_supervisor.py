"""Virtual-observation approach/brake supervisor; no capture or hardware safety claim."""
import math
class ApproachSupervisor:
 def __init__(self,trigger_x=.25,max_age=.05):
  self.trigger_x=trigger_x;self.max_age=max_age;self.state='APPROACH';self.fault=None
 def update(self,now,observation):
  values=[now,observation.get('timestamp'),observation.get('relativeX'),observation.get('relativeVx')]
  valid=all(type(x) in (int,float) and math.isfinite(x) for x in values)
  if not valid or observation['timestamp']>now+1e-9 or now-observation['timestamp']>self.max_age:
   self.state='OBSERVATION_FAULT';self.fault='stale-or-invalid'
  elif self.state=='APPROACH' and observation['relativeX']<=self.trigger_x:self.state='BRAKE'
  braking=self.state!='APPROACH'
  return dict(state=self.state,fault=self.fault,driveDemands=[0.,0.] if braking else [.2,.2],pinchDemands=[0.,0.] if braking else [.12,.12],modes=['brake','brake'] if braking else ['drive','drive'])

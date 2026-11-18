"""Voltage thresholds sourced from 2024 WPILib; policy timing is hypothetical.
No motor-controller electrical response or physical safety is established here.
"""
import math
class PowerState:
 def __init__(self,trigger_v=6.75,recovery_v=7.5,fresh_dwell_s=.05):
  if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in [trigger_v,recovery_v,fresh_dwell_s]) or not 4.5<trigger_v<recovery_v or fresh_dwell_s<0:raise ValueError('Power policy domain')
  self.trigger=trigger_v;self.recovery=recovery_v;self.dwell=fresh_dwell_s
  self.stage='NORMAL';self.armed=True;self.fresh_since=None;self.last=None;self.trips=0
 def update(self,t,volts,fresh,rearm=False,boot_complete=False):
  if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in [t,volts]) or t<0 or volts<0 or (self.last is not None and t<=self.last):raise ValueError('Monotonic finite power sample')
  if any(type(v)is not bool for v in [fresh,rearm,boot_complete]):raise ValueError('Boolean observation')
  old=self.stage
  if volts<4.5:self.stage='BLACKOUT'
  elif self.stage=='BLACKOUT':
   if volts>4.65:self.stage='BOOT_UNVERIFIED'
  elif self.stage=='BOOT_UNVERIFIED':
   if boot_complete:self.stage='BROWNOUT' if volts<=self.recovery else 'NORMAL'
  elif self.stage=='BROWNOUT':
   if volts>self.recovery:self.stage='NORMAL'
  elif volts<self.trigger:self.stage='BROWNOUT'
  if old=='NORMAL' and self.stage!='NORMAL':self.trips+=1
  rails=self.stage=='NORMAL'
  if not rails or not fresh:self.armed=False;self.fresh_since=None
  elif not self.armed:
   if self.fresh_since is None:self.fresh_since=t
   if rearm and t-self.fresh_since+1e-12>=self.dwell:self.armed=True
  self.last=t
  return dict(timeS=t,stage=self.stage,outputsEnabled=rails and fresh and self.armed,railsValid=rails,sensorFresh=fresh and rails,rearmRequired=not self.armed,pwm6VAtRisk=volts<6.8,trips=self.trips)

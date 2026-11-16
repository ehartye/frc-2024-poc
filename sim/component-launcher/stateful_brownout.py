"""Persistent four-drive mechanical state plus hypothetical power/feedback policy.
Old geometry and force law retained unchanged and conditional. Not real hardware.
"""
import json,pathlib,math,hashlib,argparse,copy
import plant
from power_state import PowerState
R=pathlib.Path(__file__).resolve().parents[2]
class StatefulPlant:
 def __init__(self,c,geometry):
  self.c=copy.deepcopy(c);self.rows=geometry['contactSampling']['two lane candidate'];self.t=0.;self.x=c['startMm']/1000;self.feed0=self.x;self.v=0.;self.omega=[0.]*4;self.feeding=False
  self.energy=dict(batterySource=0.,prescribedFeedKinetic=0.,windingHeat=0.,sourceResistanceLoss=0.,slipLoss=0.,otherRobotLoadWork=0.)
  self.feed_time=None;self.last_bus=c['busOpenCircuitV'];self.terminal=None
 def feed(self):
  if self.feeding or self.terminal:raise ValueError('Single feed only')
  self.feeding=True;self.v=self.c['feedEntrySpeedMps'];self.feed_time=self.t;self.energy['prescribedFeedKinetic']=.5*self.c['pieceMassKg']*self.v**2
 def step(self,enabled,other_current=0.):
  if type(enabled)is not bool or not math.isfinite(other_current) or other_current<0:raise ValueError('Command/load')
  if self.terminal:raise ValueError('Terminal spatial domain')
  c=dict(self.c,otherRobotCurrentA=other_current);dt=c['stepS'];j=c['rotatingInertiaEachKgM2'];r=c['radiusM'];kt=1/c['backEmfRadPerSV']
  if enabled:b,amps,ib=plant.bus(c,self.omega,c['duty'])
  else:
   b=c['busOpenCircuitV']-c['sourceResistanceOhm']*other_current
   if b<0:raise ValueError('Other load outside passive source domain')
   amps=[0.]*4;ib=other_current
  wgt=plant.engagement(self.x,self.rows) if self.feeding else 0.
  forces=[c['friction']*c['normalEachN']*wgt*math.tanh((r*w-self.v)/.05) for w in self.omega]
  e=self.energy;e['batterySource']+=c['busOpenCircuitV']*ib*dt;e['sourceResistanceLoss']+=c['sourceResistanceOhm']*ib*ib*dt;e['windingHeat']+=sum(a*a*c['windingResistanceOhm'] for a in amps)*dt;e['otherRobotLoadWork']+=b*other_current*dt;e['slipLoss']+=sum(f*(r*w-self.v) for f,w in zip(forces,self.omega))*dt
  self.omega=[w+dt*(kt*a-f*r)/j for w,a,f in zip(self.omega,amps,forces)]
  if self.feeding:
   nv=self.v+dt*(sum(forces)/c['pieceMassKg']-9.80665*math.sin(math.radians(c['pitchDegrees'])));self.x+=dt*(self.v+nv)/2;self.v=nv
  self.t+=dt;self.last_bus=b
  if self.feeding and self.v<0:self.terminal='REVERSED_BEFORE_EXIT'
  elif self.feeding and self.x>=c['releaseBoundaryMm']/1000:self.terminal='PASSED_CONTACT_TABLE_BOUNDARY'
  return dict(timeS=self.t,noteXFromStationM=self.x,pieceSpeedMps=self.v,omegaRadps=self.omega[:],busV=b,busCurrentA=ib,phaseAmps=amps,appliedEnabled=enabled,engagementWeight=wgt,feeding=self.feeding,terminal=self.terminal,energyJ=self.ledger())
 def ledger(self):
  c=self.c;e=dict(self.energy);e['finalKinetic']=.5*c['rotatingInertiaEachKgM2']*sum(w*w for w in self.omega)+.5*c['pieceMassKg']*self.v*self.v;e['piecePotentialChange']=c['pieceMassKg']*9.80665*math.sin(math.radians(c['pitchDegrees']))*(self.x-self.feed0)
  e['balanceResidual']=e['batterySource']+e['prescribedFeedKinetic']-sum(e[k] for k in ['finalKinetic','piecePotentialChange','windingHeat','sourceResistanceLoss','slipLoss','otherRobotLoadWork']);return e

def simulate(c,g,scenario='nominal',trigger=6.75,dwell=.05):
 if scenario not in ['nominal','startup-load','midfeed-load','sensor-loss','blackout','persistent-load']:raise ValueError('Scenario')
 p=StatefulPlant(c,g);power=PowerState(trigger_v=trigger,fresh_dwell_s=dwell);history=[];events=[];latched=False;last=None;minbus=c['busOpenCircuitV'];maxres=0.;tick=0
 while p.t<c['maxSpinupS']+c['maxFeedS'] and p.terminal is None:
  since=None if p.feed_time is None else p.t-p.feed_time
  load=0.
  if scenario=='startup-load' and p.t<.1:load=220.
  if scenario=='persistent-load':load=220.
  if scenario=='midfeed-load' and since is not None and .03<=since<.08:load=300.
  if scenario=='blackout' and .05<=p.t<.1:load=400.
  fresh=not(scenario=='sensor-loss' and since is not None and .03<=since<.08)
  # Last-step bus measurement: one integration-step sensing/disable latency hypothesis.
  state=power.update(p.t,p.last_bus,fresh,rearm=not latched,boot_complete=False)
  if p.feeding and not state['outputsEnabled']:latched=True
  enabled=state['outputsEnabled'] and not latched
  if not p.feeding and enabled and min(p.omega)>=c['readyRpm']*2*math.pi/60:p.feed()
  if (state['stage'],enabled,latched)!=last:
   events.append(dict(timeS=p.t,**{k:v for k,v in state.items() if k!='timeS'},appliedEnabled=enabled,feedFaultLatched=latched,noteXFromStationM=p.x,pieceSpeedMps=p.v,omegaRadps=p.omega[:]));last=(state['stage'],enabled,latched)
  row=p.step(enabled,load);row['controller']=state;row['feedFaultLatched']=latched
  minbus=min(minbus,row['busV']);maxres=max(maxres,abs(row['energyJ']['balanceResidual']))
  if tick%100==0 or p.terminal is not None:history.append(row)
  tick+=1
 return dict(scenario=scenario,triggerV=trigger,freshDwellHypothesisS=dwell,outcome=p.terminal or 'HORIZON',elapsedS=p.t,feedTimeS=p.feed_time,minimumBusV=minbus,maxAbsEnergyResidualJ=maxres,finalEnergyJ=p.ledger(),events=events,history=history,feedFaultLatched=latched,acceptance=False)

def run(output):
 out=pathlib.Path(output)
 if out.exists():raise ValueError('Fresh output required')
 c=json.loads((R/'sim/fourdrive/config.json').read_text());gp=R/c['sourceGeometry'];g=json.loads(gp.read_text());files=[pathlib.Path(__file__),R/'sim/fourdrive/power_state.py',R/'sim/fourdrive/plant.py',R/'sim/fourdrive/config.json',gp]
 sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();pins={p.relative_to(R).as_posix():sha(p) for p in files}
 cases=[simulate(c,g,name,threshold) for threshold in [6.3,6.75] for name in ['nominal','startup-load','midfeed-load','sensor-loss','blackout','persistent-load']]
 if any(sha(R/n)!=h for n,h in pins.items()):raise ValueError('Input changed')
 report=dict(schema='frc-fourdrive-brownout/1',baseCommit='2e012cbe1248b7f85f6e6605b34f429c37c1c6a2',sourceSha256=pins,cases=cases,limits=['49.53mm radius and contact weighting preserved as conditional old geometry, not seat acceptance','One-step voltage observation latency and50ms fresh/rearm dwell are explicit hypotheses, not WPILib timing specifications','Output disable modeled as open-circuit motors; no motor-controller braking/reset certainty','Rail loss invalidates observations; mid-feed fault latches inhibit and never clears mechanical momentum','Blackout/boot remains unverified until external boot-complete evidence; no invented boot duration','Other loads are prescribed shared-bus stresses, not measured robot current or operability','No target score, physical capture, safety or hardware acceptance'])
 out.parent.mkdir(parents=True,exist_ok=True)
 with out.open('x') as f:json.dump(report,f,indent=2,allow_nan=False);f.write('\n')
 print([(r['scenario'],r['triggerV'],r['outcome'],r['minimumBusV']) for r in cases]);return report
if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('--output',required=True);v=a.parse_args();run(v.output)

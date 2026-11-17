"""Finite lift/pitch mechanics; full base-coupling terms, explicit fixed-base test step."""
import math,copy
class ActuatorDomainError(ValueError):pass
def finite(x):return type(x) in (int,float) and math.isfinite(x)
def vector(v,n):
 if len(v)!=n or any(not finite(x) for x in v):raise ValueError('finite vector')
def dot(a,b):return sum(x*y for x,y in zip(a,b))
class ActuatorMechanics:
 def __init__(self,config,initial):
  self.c=copy.deepcopy(config);c=self.c
  for k in ['carriageMassKg','rotatingMassKg','comPitchInertiaKgM2','liftRotorInertiaKgM2','pitchRotorInertiaKgM2','liftGearRatio','liftRadiusM','pitchGearRatio','motorK','resistanceOhm','currentLimitA','gravityMps2']:
   if not finite(c[k]) or c[k]<=0:raise ValueError('positive '+k)
  vector(c['comLocalXZ_M'],2);vector(initial['q'],3);vector(initial['qd'],3)
  if not finite(initial['timeS']) or initial['timeS']<0:raise ValueError('clock')
  self.state=copy.deepcopy(initial);self.initialEnergy=self.energy(initial['q'],initial['qd']);self.ledger=dict(terminalWork=0.,windingHeat=0.,brakeHeat=0.,externalWork=0.,fixedConstraintWork=0.)
 def terms(self,q,qd):
  vector(q,3);vector(qd,3);c=self.c;mr=c['rotatingMassKg'];mass=mr+c['carriageMassKg'];a,b=c['comLocalXZ_M'];theta=q[2];w=qd[2];x=a*math.cos(theta)-b*math.sin(theta);h=a*math.sin(theta)+b*math.cos(theta);gl=c['liftGearRatio']/c['liftRadiusM'];gp=c['pitchGearRatio']
  matrix=[[mass,0.,-mr*h],[0.,mass+c['liftRotorInertiaKgM2']*gl*gl,mr*x],[-mr*h,mr*x,c['comPitchInertiaKgM2']+mr*(a*a+b*b)+c['pitchRotorInertiaKgM2']*gp*gp]]
  bias=[mr*x*w*w,mr*h*w*w-mass*c['gravityMps2'],-mr*c['gravityMps2']*x]
  return dict(massMatrix=matrix,biasForce=bias,rotorOmegaRadps=[gl*qd[1],gp*qd[2]],transmissionGains=[gl,gp],mechanismMassKg=mass,comOffsetXZ=[x,h])
 def energy(self,q,qd):
  t=self.terms(q,qd);kinetic=.5*sum(qd[i]*t['massMatrix'][i][j]*qd[j] for i in range(3) for j in range(3));potential=self.c['gravityMps2']*(t['mechanismMassKg']*q[1]+self.c['rotatingMassKg']*t['comOffsetXZ'][1]);return kinetic+potential
 def electrical(self,bus,duties,modes,q,qd):
  if not finite(bus) or bus<0:raise ValueError('bus')
  vector(duties,2)
  if max(map(abs,duties))>1 or len(modes)!=2 or any(m not in ['drive','coast','brake'] for m in modes):raise ValueError('commands')
  t=self.terms(q,qd);c=self.c;rows=[]
  for w,d,mode in zip(t['rotorOmegaRadps'],duties,modes):
   emf=c['motorK']*w
   if mode=='coast':i=0.;v=None;p=bh=0.;bad=False
   else:
    i=max(-c['currentLimitA'],min(c['currentLimitA'],((0. if mode=='brake' else d*bus)-emf)/c['resistanceOhm']));v=emf+i*c['resistanceOhm'];p=v*i;bh=max(0.,-p) if mode=='brake' else 0.;p=0. if mode=='brake' else p;bad=mode=='drive' and abs(v)>bus+1e-9
   rows.append(dict(omegaRadps=w,currentA=i,actualTerminalV=v,busPowerW=p,windingHeatW=i*i*c['resistanceOhm'],brakeHeatW=bh,torqueNm=c['motorK']*i,infeasible=bad))
  return dict(busPowerW=sum(r['busPowerW'] for r in rows),motors=rows,generalizedMotorForce=[0.,rows[0]['torqueNm']*t['transmissionGains'][0],rows[1]['torqueNm']*t['transmissionGains'][1]],infeasibleCurrentLimitVoltage=any(r['infeasible'] for r in rows))
 def generalized(self,bus,duties,modes,q,qd,external=(0.,0.,0.)):
  vector(external,3);t=self.terms(q,qd);e=self.electrical(bus,duties,modes,q,qd);return dict(t,electrical=e,rhs=[a+b+c for a,b,c in zip(t['biasForce'],e['generalizedMotorForce'],external)],externalPowerW=dot(external,qd))
 def _fixed_rates(self,s,bus,duties,modes,external):
  if s['qd'][0]!=0.:raise ActuatorDomainError('FIXED_BASE_REQUIRES_ZERO_BASE_VELOCITY; use full generalized terms')
  g=self.generalized(bus,duties,modes,s['q'],s['qd'],external);m=g['massMatrix'];r=g['rhs'];det=m[1][1]*m[2][2]-m[1][2]**2
  if det<=0:raise ActuatorDomainError('NONPOSITIVE_MASS_MATRIX')
  az=(r[1]*m[2][2]-m[1][2]*r[2])/det;at=(m[1][1]*r[2]-m[1][2]*r[1])/det
  return dict(acceleration=[0.,az,at],electrical=g['electrical'],constraintForceX_N=m[0][1]*az+m[0][2]*at-r[0],externalPowerW=g['externalPowerW'])
 def preview_fixed(self,dt,bus,duties,modes,external=(0.,0.,0.),state=None):
  if not finite(dt) or not 0<dt<=.001:raise ValueError('dt <=1ms')
  s=self.state if state is None else state;a=self._fixed_rates(s,bus,duties,modes,external)
  mid=dict(timeS=s['timeS']+dt/2,q=[q+dt/2*v for q,v in zip(s['q'],s['qd'])],qd=[v+dt/2*f for v,f in zip(s['qd'],a['acceleration'])]);b=self._fixed_rates(mid,bus,duties,modes,external)
  nxt=dict(timeS=s['timeS']+dt,q=[q+dt*v for q,v in zip(s['q'],mid['qd'])],qd=[v+dt*f for v,f in zip(s['qd'],b['acceleration'])]);return dict(nextState=nxt,midpoint=b,start=a)
 def power_port_fixed(self,dt,duties,modes,external=(0.,0.,0.)):
  snapshot=copy.deepcopy(self.state);duties,modes,external=copy.deepcopy((duties,modes,external))
  def port(bus):
   p=self.preview_fixed(dt,bus,duties,modes,external,snapshot);e=p['midpoint']['electrical'];e['infeasibleCurrentLimitVoltage']|=p['start']['electrical']['infeasibleCurrentLimitVoltage'];return e
  return port
 def step_fixed(self,dt,bus,duties,modes,external=(0.,0.,0.)):
  p=self.preview_fixed(dt,bus,duties,modes,external)
  if any(p[k]['electrical']['infeasibleCurrentLimitVoltage'] for k in ['start','midpoint']):raise ActuatorDomainError('UNSUPPORTED_CONTROLLER_VOLTAGE')
  m=p['midpoint'];e=m['electrical'];self.state=p['nextState'];rates=dict(terminalWork=e['busPowerW'],windingHeat=sum(x['windingHeatW'] for x in e['motors']),brakeHeat=sum(x['brakeHeatW'] for x in e['motors']),externalWork=m['externalPowerW'],fixedConstraintWork=0.)
  for k,v in rates.items():self.ledger[k]+=dt*v
  energy=self.energy(self.state['q'],self.state['qd']);l=self.ledger;res=self.initialEnergy+l['terminalWork']+l['externalWork']-l['windingHeat']-l['brakeHeat']-energy
  return dict(state=copy.deepcopy(self.state),electrical=e,constraintForceX_N=m['constraintForceX_N'],energyJ=dict(l,mechanical=energy,initial=self.initialEnergy,residual=res),fixedBase=True,acceptance=False)

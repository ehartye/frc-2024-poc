"""Six-drive reciprocal-DC kernel for future powered handoff.
Unit-tested hypotheses only. Current blocked transfer geometry is NOT accepted.
"""
import math

def require_clear_transfer(report):
 if report.get('invalidOrNonSolidInputs'):raise ValueError('Invalid material source')
 rows=report['rows']
 if [r['advanceMm'] for r in rows]!=list(range(0,481,10)):raise ValueError('Expected full transfer domain')
 if any(h['contactGroup']=='unintended' for r in rows for h in r['hits']):raise ValueError('Unresolved rigid transfer collision')
 return rows

class SixDrivePlant:
 """Explicit virtual hold, two pinch drives, four launcher drives, one DC bus."""
 def __init__(self,c,rows):
  positive=['stepS','backEmfRadPerSV','windingResistanceOhm','busOpenCircuitV','phaseCurrentLimitA','pieceMassKg','pinchRadiusM','radiusM','pinchInertiaKgM2','rotatingInertiaEachKgM2']
  nonnegative=['sourceResistanceOhm','pinchNormalEachN','normalEachN','pinchFriction','friction']
  for k in positive+nonnegative+['pitchDegrees']:
   if type(c[k]) not in [int,float] or not math.isfinite(c[k]):raise ValueError('Finite numeric configuration:'+k)
  if any(c[k]<=0 for k in positive) or any(c[k]<0 for k in nonnegative) or not -90<=c['pitchDegrees']<=90:raise ValueError('Configuration domain')
  if len(rows)<2:raise ValueError('Contact table needs intervals')
  for index,row in enumerate(rows):
   if any(type(row[k]) not in [int,float] or not math.isfinite(row[k]) for k in ['advanceMm','pinchOverlapMm3','launcherOverlapMm3']):raise ValueError('Finite contact table')
   if min(row['pinchOverlapMm3'],row['launcherOverlapMm3'])<0 or index and row['advanceMm']<=rows[index-1]['advanceMm']:raise ValueError('Ordered nonnegative contact table')
  self.c=dict(c);self.rows=rows;self.x=0.;self.v=0.;self.w=[0.]*6;self.t=0.;self.held=True
  self.r=[c['pinchRadiusM']]*2+[c['radiusM']]*4;self.j=[c['pinchInertiaKgM2']]*2+[c['rotatingInertiaEachKgM2']]*4
  self.norm=[c['pinchNormalEachN']]*2+[c['normalEachN']]*4;self.mu=[c['pinchFriction']]*2+[c['friction']]*4
  self.peaks={k:max(r[k+'OverlapMm3'] for r in rows) for k in ['pinch','launcher']}
  if any(x<=0 or not math.isfinite(x) for x in self.r+self.j+list(self.peaks.values())):raise ValueError('Positive geometry and inertia required')
  self.e={'batterySource':0.,'windingHeat':0.,'sourceResistanceLoss':0.,'slipLoss':0.,'otherLoadWork':0.}
 def weights(self):
  for a,b in zip(self.rows,self.rows[1:]):
   xa=a['advanceMm']/1000;xb=b['advanceMm']/1000
   if xa<=self.x<=xb:
    q=(self.x-xa)/(xb-xa);return {k:((1-q)*a[k+'OverlapMm3']+q*b[k+'OverlapMm3'])/self.peaks[k] for k in self.peaks}
  return dict(pinch=0.,launcher=0.)
 def step(self,duties,release=False,other_current=0.):
  c=self.c;dt=c['stepS'];kv=c['backEmfRadPerSV'];res=c['windingResistanceOhm'];source=c['busOpenCircuitV'];sr=c['sourceResistanceOhm']
  if len(duties)!=6 or any(type(x) not in [int,float] or not math.isfinite(x) or not 0<=x<=1 for x in duties):raise ValueError('Six finite duty commands')
  if type(release)is not bool or type(other_current) not in [int,float] or not math.isfinite(other_current) or other_current<0 or not other_current*sr<source:raise ValueError('Release/load domain')
  if release:self.held=False
  def amps(v):return [0. if d==0 else max(0.,min(c['phaseCurrentLimitA'],(d*v-w/kv)/res)) for d,w in zip(duties,self.w)]
  lo=0.;hi=source
  for _ in range(40):
   bus=(lo+hi)/2;aa=amps(bus);ib=sum((w/kv+i*res)*i for w,i in zip(self.w,aa))/max(bus,1e-12)+other_current
   if bus+sr*ib>source:hi=bus
   else:lo=bus
  bus=(lo+hi)/2;aa=amps(bus);ib=sum((w/kv+i*res)*i for w,i in zip(self.w,aa))/max(bus,1e-12)+other_current
  weights=self.weights();body_v=0. if self.held else self.v
  forces=[mu*n*weights['pinch' if i<2 else 'launcher']*math.tanh((radius*w-body_v)/.05) for i,(mu,n,radius,w) in enumerate(zip(self.mu,self.norm,self.r,self.w))]
  self.e['batterySource']+=source*ib*dt;self.e['windingHeat']+=sum(i*i*res for i in aa)*dt;self.e['sourceResistanceLoss']+=sr*ib*ib*dt;self.e['otherLoadWork']+=bus*other_current*dt;self.e['slipLoss']+=sum(f*(r*w-body_v) for f,r,w in zip(forces,self.r,self.w))*dt
  self.w=[w+dt*(i/kv-f*r)/j for w,i,f,r,j in zip(self.w,aa,forces,self.r,self.j)]
  if not self.held:
   nv=self.v+dt*(sum(forces)/c['pieceMassKg']-9.80665*math.sin(math.radians(c['pitchDegrees'])));self.x+=dt*(self.v+nv)/2;self.v=nv
  self.t+=dt
  kinetic=.5*sum(j*w*w for j,w in zip(self.j,self.w))+.5*c['pieceMassKg']*self.v*self.v;potential=c['pieceMassKg']*9.80665*math.sin(math.radians(c['pitchDegrees']))*self.x
  return {'timeS':self.t,'advanceM':self.x,'pieceSpeedMps':self.v,'omegaRadps':self.w[:],'busV':bus,'busCurrentA':ib,'phaseCurrentA':aa,'held':self.held,'forcesN':forces,'energyJ':dict(self.e,kinetic=kinetic,potential=potential,residual=self.e['batterySource']-sum(v for k,v in self.e.items() if k!='batterySource')-kinetic-potential),'acceptance':False}

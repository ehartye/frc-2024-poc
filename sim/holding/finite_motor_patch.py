"""Finite shear patches + two finite-inertia motors, not a lift or contact finder."""
import copy,math
class HoldingDomainError(ValueError):pass

def finite(x):return type(x) in (int,float) and math.isfinite(x)
def vec(v,n):
 if not isinstance(v,(list,tuple)) or len(v)!=n or any(not finite(x) for x in v):raise ValueError('finite vector')
def rotate(v,a):
 c,s=math.cos(a),math.sin(a);return [c*v[0]-s*v[1],s*v[0]+c*v[1]]
def dot(a,b):return sum(x*y for x,y in zip(a,b))

class FiniteMotorPatch:
 def __init__(self,config,initial):
  self.c=copy.deepcopy(config);c=self.c
  for k in ['motorK','resistanceOhm','currentLimitA','rotorInertiaKgM2','beltRadiusM','shearStiffnessPerSide']:
   if not finite(c[k]) or c[k]<=0:raise ValueError('positive '+k)
  if not finite(c['mu']) or c['mu']<0:raise ValueError('nonnegative friction')
  vec(c['normalSideN'],2)
  if min(c['normalSideN'])<0:raise ValueError('normal budget')
  self.elastic=c['law']=='elastic-plastic'
  if c['law'] not in ['elastic-plastic','memoryless-tanh']:raise ValueError('explicit contact law')
  if not finite(c['slipRegularizationMps']) or c['slipRegularizationMps']<=0:raise ValueError('slip regularization')
  self.sides=list(c['patchMotor'])
  if not self.sides or any(type(s)is not int or s not in (0,1) for s in self.sides) or set(self.sides)!={0,1}:raise ValueError('two finite patch sets')
  self.counts=[self.sides.count(i) for i in range(2)];self.n=len(self.sides)
  if not finite(initial['timeS']) or initial['timeS']<0 or not finite(initial['frameAngleRad']):raise ValueError('initial clock/frame')
  vec(initial['rotorOmegaRadps'],2)
  if self.elastic:
   if len(initial['shearWorldM'])!=self.n:raise ValueError('explicit shear history required')
   for v in initial['shearWorldM']:vec(v,2)
  elif 'shearWorldM' in initial:raise ValueError('memoryless model has no elastic history')
  self.state=copy.deepcopy(initial)
  for i,s in enumerate(self.sides) if self.elastic else []:
   if math.hypot(*self.state['shearWorldM'][i])*c['shearStiffnessPerSide']>c['mu']*c['normalSideN'][s]+1e-12:raise ValueError('initial shear beyond capacity')
  self.energy={k:0. for k in ['terminalWork','windingHeat','brakeHeat','cradleWork','bodyWork','plasticHeat','returnMapNumericalLoss']};self.initial_stored=self.stored(self.state)
 def stored(self,s):
  c=self.c
  return .5*c['rotorInertiaKgM2']*sum(w*w for w in s['rotorOmegaRadps'])+(sum(.5*c['shearStiffnessPerSide']/self.counts[m]*dot(v,v) for m,v in zip(self.sides,s['shearWorldM'])) if self.elastic else 0.)
 def electrical(self,bus,duty,mode,w):
  c=self.c;emf=c['motorK']*w
  if mode=='coast':return dict(currentA=0.,actualTerminalV=None,busPowerW=0.,windingHeatW=0.,brakeHeatW=0.,torqueNm=0.,infeasible=False)
  target=0. if mode=='brake' else duty*bus;i=max(-c['currentLimitA'],min(c['currentLimitA'],(target-emf)/c['resistanceOhm']));v=emf+i*c['resistanceOhm'];p=v*i
  return dict(currentA=i,actualTerminalV=v,busPowerW=0. if mode=='brake' else p,windingHeatW=i*i*c['resistanceOhm'],brakeHeatW=max(0.,-p) if mode=='brake' else 0.,torqueNm=c['motorK']*i,infeasible=mode=='drive' and abs(v)>bus+1e-9)
 def query(self,dt,bus,duties,modes,frame_angle,body_velocities,cradle_velocities,lever_world,valid):
  if not finite(dt) or not 0<dt<=.0025 or not finite(bus) or bus<0 or not finite(frame_angle):raise ValueError('interval/bus/frame')
  vec(duties,2)
  if max(map(abs,duties))>1 or len(modes)!=2 or any(m not in ('drive','coast','brake') for m in modes):raise ValueError('motor commands')
  if len(valid)!=self.n or any(type(v)is not bool for v in valid):raise ValueError('explicit witness validity')
  if not all(valid):raise HoldingDomainError('UNKNOWN_FINITE_PATCH_PERSISTENCE')
  for collection in [body_velocities,cradle_velocities,lever_world]:
   if len(collection)!=self.n:raise ValueError('patch vector inventory')
   for v in collection:vec(v,2)
  c=self.c;s=self.state;axis=[math.cos(frame_angle),math.sin(frame_angle)];history=[rotate(v,frame_angle-s['frameAngleRad']) for v in s['shearWorldM']] if self.elastic else [[0.,0.] for _ in self.sides]
  def contact(i,mid):
   motor=self.sides[i];k=c['shearStiffnessPerSide']/self.counts[motor];cap=c['mu']*c['normalSideN'][motor]/self.counts[motor]
   slip=[a-b+c['beltRadiusM']*mid*t for a,b,t in zip(cradle_velocities[i],body_velocities[i],axis)]
   if not self.elastic:
    speed=math.hypot(*slip);scale=0. if speed==0 else cap*math.tanh(speed/c['slipRegularizationMps'])/speed;f=[scale*v for v in slip]
    return dict(force=f,plasticHeat=dot(f,slip)*dt,numericalLoss=0.,slip=slip)
   trial=[x+dt*v for x,v in zip(history[i],slip)];mag=math.hypot(*trial);factor=1. if mag==0 else min(1.,cap/(k*mag));new=[x*factor for x in trial];f=[k*x for x in new];plastic=[a-b for a,b in zip(trial,new)]
   return dict(shear=new,force=f,plasticHeat=dot(f,plastic),numericalLoss=.5*k*sum((a-b)**2 for a,b in zip(new,history[i])),slip=slip)
  nextw=[];motors=[];rows=[None]*self.n
  for motor in range(2):
   old=s['rotorOmegaRadps'][motor];indices=[i for i,m in enumerate(self.sides) if m==motor]
   def evaluate(new):
    mid=(old+new)/2;e=self.electrical(bus,duties[motor],modes[motor],mid);patches=[contact(i,mid) for i in indices];torque=-c['beltRadiusM']*sum(dot(p['force'],axis) for p in patches);residual=new-old-dt*(e['torqueNm']+torque)/c['rotorInertiaKgM2'];return residual,e,patches
   width=1.;lo=old-width;hi=old+width
   for _ in range(40):
    if evaluate(lo)[0]<=0<=evaluate(hi)[0]:break
    width*=2;lo=old-width;hi=old+width
   else:raise HoldingDomainError('UNRESOLVED_IMPLICIT_ROTOR_ROOT')
   if evaluate(old)[0]==0.:lo=hi=old
   for _ in range(60):
    mid=(lo+hi)/2
    if evaluate(mid)[0]>0:hi=mid
    else:lo=mid
   new=(lo+hi)/2;res,e,patches=evaluate(new)
   if abs(res)>1e-8:raise HoldingDomainError('IMPLICIT_ROTOR_RESIDUAL')
   nextw.append(new);motors.append(dict(e,midOmegaRadps=(old+new)/2,rootResidualRadps=res))
   for i,p in zip(indices,patches):rows[i]=p
  nextstate=dict(timeS=s['timeS']+dt,frameAngleRad=frame_angle,rotorOmegaRadps=nextw)
  if self.elastic:nextstate['shearWorldM']=[p['shear'] for p in rows]
  delta=dict(terminalWork=sum(e['busPowerW'] for e in motors)*dt,windingHeat=sum(e['windingHeatW'] for e in motors)*dt,brakeHeat=sum(e['brakeHeatW'] for e in motors)*dt,cradleWork=sum(dot(p['force'],v) for p,v in zip(rows,cradle_velocities))*dt,bodyWork=sum(dot(p['force'],v) for p,v in zip(rows,body_velocities))*dt,plasticHeat=sum(p['plasticHeat'] for p in rows),returnMapNumericalLoss=sum(p['numericalLoss'] for p in rows))
  change=self.stored(nextstate)-self.stored(s);residual=delta['terminalWork']+delta['cradleWork']-delta['bodyWork']-sum(delta[k] for k in ['windingHeat','brakeHeat','plasticHeat','returnMapNumericalLoss'])-change
  force=[sum(p['force'][j] for p in rows) for j in [0,1]];torque=sum(l[0]*p['force'][1]-l[1]*p['force'][0] for l,p in zip(lever_world,rows))
  return dict(busPowerW=sum(e['busPowerW'] for e in motors),motors=motors,patches=rows,bodyForceXZ_N=force,bodyPitchTorqueNm=torque,nextState=nextstate,deltaEnergyJ=delta,intervalResidualJ=residual,infeasibleCurrentLimitVoltage=any(e['infeasible'] for e in motors),acceptance=False)
 def step(self,*args,**kwargs):
  q=self.query(*args,**kwargs)
  if q['infeasibleCurrentLimitVoltage']:raise HoldingDomainError('UNSUPPORTED_CONTROLLER_VOLTAGE')
  self.state=q['nextState']
  for k,v in q['deltaEnergyJ'].items():self.energy[k]+=v
  stored=self.stored(self.state);e=self.energy;res=self.initial_stored+e['terminalWork']+e['cradleWork']-e['bodyWork']-sum(e[k] for k in ['windingHeat','brakeHeat','plasticHeat','returnMapNumericalLoss'])-stored
  return dict(q,energyJ=dict(e,stored=stored,initialStored=self.initial_stored,residual=res))
 def power_port(self,dt,duties,modes,frame_angle,body_velocities,cradle_velocities,lever_world,valid):
  frozen=copy.deepcopy(self);inputs=copy.deepcopy((duties,modes,frame_angle,body_velocities,cradle_velocities,lever_world,valid))
  return lambda bus:frozen.query(dt,bus,*inputs)

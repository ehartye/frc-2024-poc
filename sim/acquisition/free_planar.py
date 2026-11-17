"""Planar NOTE/motor dynamics with caller-supplied finite contact geometry.
No CAD acceptance, capture criterion, or physical material calibration.
"""
import math,copy

class DomainExit(Exception):pass

def check_contacts(geometry,q):
 result=geometry(q)
 if result['status']!='SUPPORTED':raise DomainExit(result['status'])
 seen=set()
 for c in result['contacts']:
  if type(c['motor']) is not int or c['motor'] not in [0,1] or c['motor'] in seen:raise ValueError('one contact per side/motor')
  seen.add(c['motor'])
  if any(len(c[k])!=2 for k in ['normal','tangent','leverM']):raise ValueError('planar contact vectors')
  vals=[c['penetrationM'],*c['normal'],*c['tangent'],*c['leverM']]
  if any(type(x) not in [int,float] or not math.isfinite(x) for x in vals) or c['penetrationM']<0:raise ValueError('finite contact geometry')
  n,t=c['normal'],c['tangent']
  if abs(sum(x*x for x in n)-1)>1e-9 or abs(sum(x*x for x in t)-1)>1e-9 or abs(sum(a*b for a,b in zip(n,t)))>1e-9:raise ValueError('orthonormal contact frame')
 return result['contacts']

def evaluate(s,c,geometry,base_velocity,duties):
 q=[s['p'][0]-s['baseX'],s['p'][1]];contacts=check_contacts(geometry,q)
 kv=c['kvRadpsPerV'];res=c['motorResistanceOhm'];source=c['sourceV'];sr=c['sourceResistanceOhm'];w=s['w']
 def currents(bus):return [0. if d==0 else max(0.,min(c['currentLimitA'],(d*bus-o/kv)/res)) for d,o in zip(duties,w)]
 lo,hi=0.,source
 for _ in range(32):
  bus=(lo+hi)/2;amps=currents(bus);ib=sum((o/kv+i*res)*i for o,i in zip(w,amps))/max(bus,1e-12)
  if bus+sr*ib>source:hi=bus
  else:lo=bus
 bus=(lo+hi)/2;amps=currents(bus);ib=sum((o/kv+i*res)*i for o,i in zip(w,amps))/max(bus,1e-12)
 if any((o/kv+i*res)*i < -1e-10 for o,i in zip(w,amps)):raise DomainExit('UNMODELED_MOTOR_REGENERATION')
 force=[0.,0.];torque=0.;motor_torque=[i/kv for i in amps];normal_loss=slip_loss=ground_loss=base_work=spring=0.;contact_rows=[]
 for g in contacts:
  delta=g['penetrationM'];n=g['normal'];t=g['tangent'];rx,ry=g['leverM'];idx=g['motor'];vx=s['v'][0]-s['spin']*ry-base_velocity;vy=s['v'][1]+s['spin']*rx
  vn=vx*n[0]+vy*n[1];vt=vx*t[0]+vy*t[1];compression=max(-vn,0.)
  normal=c['normalStiffnessNpm']*delta*(1+c['compressionAlphaSpm']*compression)
  slip=c['radiusM']*w[idx]-vt;tangent=c['beltMu']*normal*math.tanh(slip/c['slipRegularizationMps'])
  fx=normal*n[0]+tangent*t[0];fy=normal*n[1]+tangent*t[1]
  force[0]+=fx;force[1]+=fy;torque+=rx*fy-ry*fx;motor_torque[idx]-=c['radiusM']*tangent
  spring+=.5*c['normalStiffnessNpm']*delta*delta;normal_loss+=c['compressionAlphaSpm']*c['normalStiffnessNpm']*delta*compression*compression;slip_loss+=tangent*slip;base_work+=fx*base_velocity
  contact_rows.append(dict(motor=idx,normalN=normal,tangentialN=tangent,penetrationM=delta))
 # Four equal ground-normal quadrature loads on the nominal torus centerline.
 for rx,ry in [(c['groundRingRadiusM'],0),(-c['groundRingRadiusM'],0),(0,c['groundRingRadiusM']),(0,-c['groundRingRadiusM'])]:
  vx=s['v'][0]-s['spin']*ry;vy=s['v'][1]+s['spin']*rx;speed=math.hypot(vx,vy)
  factor=0 if speed==0 else -c['groundMu']*c['pieceMassKg']*9.80665/4*math.tanh(speed/c['slipRegularizationMps'])/speed
  fx,fy=factor*vx,factor*vy;force[0]+=fx;force[1]+=fy;torque+=rx*fy-ry*fx;ground_loss-=fx*vx+fy*vy
 rates=dict(electricSource=source*ib,windingHeat=sum(i*i*res for i in amps),sourceHeat=sr*ib*ib,normalHeat=normal_loss,slipHeat=slip_loss,groundHeat=ground_loss,prescribedBaseWork=base_work)
 return dict(accel=[f/c['pieceMassKg'] for f in force],spinAccel=torque/c['yawInertiaKgM2'],rotorAccel=[v/c['rotorInertiaKgM2'] for v in motor_torque],rates=rates,spring=spring,busV=bus,contacts=contact_rows)

def shifted(s,d,dt,base_velocity):
 return dict(t=s['t']+dt,p=[x+dt*v for x,v in zip(s['p'],s['v'])],v=[v+dt*a for v,a in zip(s['v'],d['accel'])],yaw=s['yaw']+dt*s['spin'],spin=s['spin']+dt*d['spinAccel'],w=[w+dt*a for w,a in zip(s['w'],d['rotorAccel'])],baseX=s['baseX']+dt*base_velocity)

def kinetic(s,c):return .5*c['pieceMassKg']*sum(x*x for x in s['v'])+.5*c['yawInertiaKgM2']*s['spin']**2+.5*c['rotorInertiaKgM2']*sum(x*x for x in s['w'])

def simulate(c,geometry,initial,base_velocity=0.,duties=(0.,0.)):
 for key,value in c.items():
  if type(value) not in [int,float] or not math.isfinite(value) or value<0:raise ValueError('finite nonnegative configuration '+key)
 for key in ['stepS','horizonS','kvRadpsPerV','motorResistanceOhm','sourceV','currentLimitA','radiusM','slipRegularizationMps','pieceMassKg','yawInertiaKgM2','rotorInertiaKgM2']:
  if c[key]<=0:raise ValueError('positive '+key)
 if len(duties)!=2 or any(type(d) not in [int,float] or not math.isfinite(d) or not 0<=d<=1 for d in duties):raise ValueError('two duties')
 if type(base_velocity) not in [int,float] or not math.isfinite(base_velocity):raise ValueError('finite prescribed base speed')
 s=dict(t=0.,p=list(initial['p']),v=list(initial.get('v',[0.,0.])),yaw=initial.get('yaw',0.),spin=initial.get('spin',0.),w=list(initial.get('w',[0.,0.])),baseX=0.)
 if any(len(s[k])!=2 for k in ['p','v','w']):raise ValueError('two planar coordinates and two motors')
 if any(type(v) not in [int,float] or not math.isfinite(v) for v in [*s['p'],*s['v'],s['yaw'],s['spin'],*s['w']]):raise ValueError('finite initial state')
 d=evaluate(s,c,geometry,base_velocity,duties);energy={k:0. for k in d['rates']};initial_energy=kinetic(s,c)+d['spring'];trace=[];max_res=0.;outcome='HORIZON';tick=0;stopped_attempt=None;peak_normal=0.;peak_penetration=0.
 def row(d):
  nonlocal max_res,peak_normal,peak_penetration
  peak_normal=max([peak_normal]+[x['normalN'] for x in d['contacts']]);peak_penetration=max([peak_penetration]+[x['penetrationM'] for x in d['contacts']])
  ke=kinetic(s,c);res=initial_energy+energy['electricSource']+energy['prescribedBaseWork']-sum(v for k,v in energy.items() if k not in ['electricSource','prescribedBaseWork'])-ke-d['spring'];max_res=max(max_res,abs(res))
  return dict(**copy.deepcopy(s),relativeRobotXY=[s['p'][0]-s['baseX'],s['p'][1]],busV=d['busV'],duties=list(duties),contacts=d['contacts'],energyJ=dict(energy,kinetic=ke,spring=d['spring'],initial=initial_energy,residual=res))
 trace.append(row(d))
 while s['t']<c['horizonS']-1e-12:
  dt=min(c['stepS'],c['horizonS']-s['t'])
  try:
   mid=shifted(s,d,dt/2,base_velocity);attempt=mid;stage='midpoint';dm=evaluate(mid,c,geometry,base_velocity,duties)
   nxt=copy.deepcopy(s);nxt.update(t=s['t']+dt,p=[p+dt*v for p,v in zip(s['p'],mid['v'])],v=[v+dt*a for v,a in zip(s['v'],dm['accel'])],yaw=s['yaw']+dt*mid['spin'],spin=s['spin']+dt*dm['spinAccel'],w=[w+dt*a for w,a in zip(s['w'],dm['rotorAccel'])],baseX=s['baseX']+dt*base_velocity)
   attempt=nxt;stage='endpoint';dn=evaluate(nxt,c,geometry,base_velocity,duties)
  except DomainExit as e:
   outcome='UNKNOWN_DOMAIN: '+str(e);stopped_attempt=dict(stage=stage,timeS=attempt['t'],relativeRobotXY=[attempt['p'][0]-attempt['baseX'],attempt['p'][1]],reason=str(e));break
  s=nxt;d=dn
  for k,v in dm['rates'].items():energy[k]+=v*dt
  r=row(d)
  if tick%20==0:trace.append(r)
  tick+=1
 if trace[-1]['t']!=s['t']:trace.append(row(d))
 return dict(outcome=outcome,trace=trace,final=row(d),maxAbsEnergyResidualJ=max_res,stoppedAttempt=stopped_attempt,peakStepNormalN=peak_normal,peakStepPenetrationM=peak_penetration,acceptance=False)

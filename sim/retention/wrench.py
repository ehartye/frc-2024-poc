"""Conditional finite-patch friction-wrench construction, not attached-body dynamics."""
import math
import numpy as np

def allocate(points,force_xz,torque,mu,normal_per_side):
 if not points or len(force_xz)!=2:raise ValueError('Finite point set and planar force required')
 for v in [*force_xz,torque,mu,*normal_per_side.values()]:
  if type(v) not in [int,float] or not math.isfinite(v):raise ValueError('Finite inputs required')
 if mu<0 or set(normal_per_side)!={-1,1} or min(normal_per_side.values())<0:raise ValueError('Contact capacity domain')
 count={s:sum(p['side']==s for p in points) for s in [-1,1]}
 if min(count.values())==0:raise ValueError('Both side patches required')
 a=np.zeros((3,2*len(points)));caps=[]
 for i,p in enumerate(points):
  xyz=p['positionM']
  if len(xyz)!=3 or type(p['side']) is not int or p['side'] not in [-1,1] or any(type(v) not in [int,float] or not math.isfinite(v) for v in xyz):raise ValueError('Finite point geometry')
  x,y,z=xyz;a[0,2*i]=1;a[1,2*i+1]=1;a[2,2*i]=-z;a[2,2*i+1]=x
  caps.append(mu*normal_per_side[p['side']]/count[p['side']])
 b=np.array([*force_xz,torque],dtype=float);scale=np.repeat(caps,2);weighted=a*scale
 u=np.linalg.pinv(weighted)@b;forces=scale*u;residual=float(np.linalg.norm(a@forces-b));util=[float(np.linalg.norm(u[2*i:2*i+2])) for i in range(len(points))]
 force_limit=sum(caps);torque_limit=sum(cap*math.hypot(p['positionM'][0],p['positionM'][2]) for cap,p in zip(caps,points))
 force_fail=math.hypot(*force_xz)>force_limit+1e-9;torque_fail=abs(torque)>torque_limit+1e-9
 lateral=sum(-p['side']*normal_per_side[p['side']]/count[p['side']] for p in points)
 roll=sum(p['positionM'][1]*forces[2*i+1]+p['positionM'][2]*p['side']*normal_per_side[p['side']]/count[p['side']] for i,p in enumerate(points))
 yaw=sum(-p['positionM'][0]*p['side']*normal_per_side[p['side']]/count[p['side']]-p['positionM'][1]*forces[2*i] for i,p in enumerate(points))
 if max(abs(lateral),abs(roll),abs(yaw))>1e-8:status='UNRESOLVED_NONPLANAR_WRENCH'
 elif force_fail or torque_fail:status='EXCEEDS_NECESSARY_CAPACITY_BOUND'
 elif residual<=1e-8 and max(util)<=1+1e-9:status='CONSTRUCTIVE_ALLOCATION_WITHIN_HYPOTHESIS'
 else:status='UNRESOLVED_ALLOCATION_NOT_PROOF_OF_FAILURE'
 return dict(status=status,unbalancedLateralForceN=float(lateral),unbalancedRollNm=float(roll),unbalancedYawNm=float(yaw),pointForcesXZ=forces.reshape((-1,2)).tolist(),pointCapacitiesN=caps,maxUtilization=max(util),wrenchResidual=residual,forceBoundN=force_limit,torqueBoundNm=torque_limit,acceptance=False)

def quintic(t,duration):
 if duration<=0:raise ValueError('Positive duration')
 u=max(0.,min(1.,t/duration));return 10*u**3-15*u**4+6*u**5,(30*u*u-60*u**3+30*u**4)/duration,(60*u-180*u*u+120*u**3)/duration**2

def demand(t,rotation_time,lift_time,pitch,major_radius,mass,inertia,offset_x,lift_distance):
 if t<=rotation_time:
  q,dq,ddq=quintic(t,rotation_time);theta=pitch*q;omega=pitch*dq;alpha=pitch*ddq
  ax=-offset_x*(math.cos(theta)*omega**2+math.sin(theta)*alpha)
  az=(major_radius+offset_x)*(math.cos(theta)*alpha-math.sin(theta)*omega**2)
  stage='ROTATION_REQUIREMENT'
 else:
  q,dq,ddq=quintic(t-rotation_time,lift_time);theta=pitch;alpha=0.;ax=0.;az=lift_distance*ddq;stage='LIFT_REQUIREMENT'
 fx=mass*ax;fz=mass*(az+9.80665)
 return dict(stage=stage,pitchRad=theta,forceLocalXZ=[math.cos(theta)*fx+math.sin(theta)*fz,-math.sin(theta)*fx+math.cos(theta)*fz],torqueNm=inertia*alpha)

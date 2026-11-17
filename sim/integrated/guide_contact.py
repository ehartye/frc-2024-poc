"""Conditional compliant guide wrench with moving-chassis work; not contact acceptance."""
import math
class GuideDomainError(ValueError):pass

def num(x):return type(x) in (int,float) and math.isfinite(x)
def vec(v):
 if type(v) is not list or len(v)!=3 or not all(num(x) for x in v):raise ValueError('finite XYZ vector')
def dot(a,b):return sum(x*y for x,y in zip(a,b))
def cross(a,b):return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
def wrench(q,*,stiffness,damping,mu,slip_scale,chassis_position,chassis_velocity,heading,yaw_rate,piece_center,piece_point_velocity,contact_y_robot=0.):
 for v in [chassis_position,chassis_velocity,piece_center,piece_point_velocity]:vec(v)
 if not all(num(x) for x in [stiffness,damping,mu,slip_scale,heading,yaw_rate,contact_y_robot]) or min(stiffness,slip_scale)<=0 or min(damping,mu)<0:raise ValueError('explicit finite constitutive parameters')
 if not q.get('geometryDomainValid') or q.get('status')=='UNKNOWN':raise GuideDomainError('UNKNOWN_FINITE_GUIDE_GEOMETRY')
 if q['status']=='SEPARATED_FROM_ENTIRE_GUIDE':return dict(forceWorldN=[0.,0.,0.],pieceMomentWorldNm=[0.,0.,0.],chassisReactionWorldN=[0.,0.,0.],chassisMomentWorldNm=[0.,0.,0.],elasticJ=0.,normalHeatW=0.,frictionHeatW=0.,pieceWorkW=0.,guideWorkW=0.,conditionalMaterialModel=True,acceptance=False)
 if q['status']!='ANALYTIC_FINITE_GUIDE_CANDIDATE':raise GuideDomainError('unsupported guide query')
 delta=q['penetrationM'];nx,nz=q['normalRobotXZ'];x,z=q['guidePointRobotXZm']
 if not all(num(v) for v in [delta,nx,nz,x,z]) or delta<0 or abs(nx*nx+nz*nz-1)>1e-10:raise ValueError('finite unit-normal guide geometry')
 c,s=math.cos(heading),math.sin(heading);arm=[c*x-s*contact_y_robot,s*x+c*contact_y_robot,z];point=[a+b for a,b in zip(chassis_position,arm)];normal=[nx*c,nx*s,nz]
 gv=[chassis_velocity[0]-yaw_rate*arm[1],chassis_velocity[1]+yaw_rate*arm[0],chassis_velocity[2]]
 relative=[a-b for a,b in zip(piece_point_velocity,gv)];vn=dot(relative,normal);vt=[a-vn*b for a,b in zip(relative,normal)];speed=math.sqrt(dot(vt,vt));elastic=stiffness*delta;normal_force=max(0.,elastic-damping*vn)
 scale=0. if speed==0 else -mu*normal_force*math.tanh(speed/slip_scale)/speed;friction=[scale*v for v in vt];force=[normal_force*n+f for n,f in zip(normal,friction)];reaction=[-f for f in force];lever=[a-b for a,b in zip(point,piece_center)]
 return dict(pointWorldM=point,normalWorld=normal,guidePointVelocityWorldMps=gv,relativeNormalSpeedMps=vn,normalForceN=normal_force,forceWorldN=force,pieceMomentWorldNm=cross(lever,force),chassisReactionWorldN=reaction,chassisMomentWorldNm=cross(arm,reaction),elasticJ=.5*stiffness*delta*delta,normalHeatW=-(normal_force-elastic)*vn,frictionHeatW=-dot(friction,relative),pieceWorkW=dot(force,piece_point_velocity),guideWorkW=dot(force,gv),penetrationRateMps=-vn,nativeMaterialStatus=q['nativeMaterialStatus'],conditionalMaterialModel=True,acceptance=False)

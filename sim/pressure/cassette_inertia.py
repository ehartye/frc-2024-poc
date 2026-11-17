"""Reciprocal planar chassis/cassette inertia hypothesis; no forces or integrator."""
from dataclasses import dataclass
import math,json,pathlib
import numpy as np
from powered_cam_contact import scalar

@dataclass(frozen=True)
class Partition:
 total_mass: float
 total_yaw_inertia: float
 positive_mass: float
 negative_mass: float
 positive_com: tuple
 negative_com: tuple
 positive_cam_inertia: float
 negative_cam_inertia: float
 def terms(self):
  values=[scalar(n,getattr(self,n)) for n in ['total_mass','total_yaw_inertia','positive_mass','negative_mass','positive_cam_inertia','negative_cam_inertia']]
  if any(v<=0 for v in values):raise ValueError('positive mass/inertia hypotheses')
  points=[]
  for p in [self.positive_com,self.negative_com]:
   if not isinstance(p,(list,tuple)) or len(p)!=2:raise ValueError('two scalar COM coordinates')
   points.append(np.array([scalar('COM',v) for v in p]))
  fixed=self.total_mass-self.positive_mass-self.negative_mass
  if fixed<=0:raise ValueError('moving groups must replace existing aggregate mass')
  fixed_com=-(self.positive_mass*points[0]+self.negative_mass*points[1])/fixed
  fixed_j=self.total_yaw_inertia-fixed*float(fixed_com@fixed_com)-sum(m*float(r@r) for m,r in zip([self.positive_mass,self.negative_mass],points))
  if not math.isfinite(fixed_j) or fixed_j<=0:raise ValueError('nonpositive remaining fixed-COM yaw inertia')
  return fixed,fixed_com,fixed_j,points

def default_partition():
 r=pathlib.Path(__file__).resolve().parents[2]
 drive=json.loads((r/'sim/drivetrain/config.json').read_text());cad=json.loads((r/'cad/assembly/intake_components.json').read_text())
 # Anchored hypotheses ONLY: these candidate frame coordinates are not measured moving-group COMs.
 x=(cad['stageCenterX']+cad['pinch']['beltCenterX'])/1000;y=cad['pinch']['innerFaceYAbs']/1000
 return Partition(drive['massKg'],drive['yawInertiaKgM2'],1.5,1.5,(x,y),(x,-y),1e-4,1e-4)

def vector(v,n,name):
 if not isinstance(v,(tuple,list,np.ndarray)) or len(v)!=n:raise ValueError(name+' shape')
 return np.array([scalar(name,x) for x in v])

def evaluate(partition,coordinates,velocities):
 """q=(X,Y,yaw,u+,u-,theta+,theta-), velocities in inertial coordinate derivatives."""
 if not isinstance(partition,Partition):raise ValueError('typed partition')
 q=vector(coordinates,7,'q');v=vector(velocities,7,'velocity');fixed,fc,fj,points=partition.terms()
 if any(not 0<=q[i]<=.0015 for i in [3,4]):raise ValueError('cassette travel domain')
 if any(not 0<=q[i]<=math.pi for i in [5,6]):raise ValueError('cam angle domain')
 co,si=math.cos(q[2]),math.sin(q[2]);rot=np.array([[co,-si],[si,co]]);turn=np.array([[0.,-1.],[1.,0.]])
 matrix=np.zeros((7,7));bias=np.zeros(7);energy=0.;bodies=[]
 for mass,r0,index,sign in [(fixed,fc,None,0),(partition.positive_mass,points[0],3,1),(partition.negative_mass,points[1],4,-1)]:
  direction=np.array([0.,-float(sign)]);r=r0.copy()
  if index is not None:r+=direction*q[index]
  j=np.zeros((2,7));j[:,:2]=np.eye(2);j[:,2]=rot@turn@r
  if index is not None:j[:,index]=rot@direction
  ud=0. if index is None else v[index]
  convective=rot@(-v[2]**2*r+2*v[2]*(turn@direction)*ud)
  vel=j@v;matrix+=mass*j.T@j;bias+=mass*j.T@convective;energy+=.5*mass*float(vel@vel)
  bodies.append(dict(massKg=mass,positionWorld=(q[:2]+rot@r).tolist(),velocityWorld=vel.tolist(),jacobian=j.tolist(),convectiveAcceleration=convective.tolist()))
 matrix[2,2]+=fj;energy+=.5*fj*v[2]**2
 for index,inertia in [(5,partition.positive_cam_inertia),(6,partition.negative_cam_inertia)]:matrix[index,index]+=inertia;energy+=.5*inertia*v[index]**2
 if not np.isfinite(matrix).all() or not np.isfinite(bias).all() or not math.isfinite(energy):raise ValueError('derived inertia overflow')
 if np.linalg.eigvalsh(matrix).min()<=0:raise ValueError('nonpositive mass matrix')
 return dict(massMatrix=matrix.tolist(),bias=bias.tolist(),kineticJ=energy,bodies=bodies,fixedMassKg=fixed,fixedComBody=fc.tolist(),fixedYawInertiaAtCom=fj,acceptance=False)

def acceleration(partition,q,v,forces,locked=()):
 r=evaluate(partition,q,v);force=vector(forces,7,'generalized force');matrix=np.array(r['massMatrix']);bias=np.array(r['bias'])
 if any(type(i)!=int or not 0<=i<7 for i in locked) or len(set(locked))!=len(locked):raise ValueError('constraint coordinates')
 if any(v[i]!=0 for i in locked):raise ValueError('locking cannot remove existing momentum')
 free=[i for i in range(7) if i not in locked];a=np.zeros(7)
 if free:a[free]=np.linalg.solve(matrix[np.ix_(free,free)],(force-bias)[free])
 reaction=matrix@a+bias-force
 if not np.isfinite(a).all() or not np.isfinite(reaction).all():raise ValueError('derived acceleration overflow')
 return dict(**r,acceleration=a.tolist(),constraintReaction=reaction.tolist(),constraintPowerW=float(reaction@np.array(v)))

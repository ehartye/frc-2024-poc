"""Finite material-face witnesses under explicit rigid transforms; no pressure law."""
import math

def finite(x):return type(x) in (int,float) and math.isfinite(x)
def vector(x):
 if type(x) is not list or len(x)!=3 or not all(finite(v) for v in x):raise ValueError('finite XYZ vector')
def dot(a,b):return sum(x*y for x,y in zip(a,b))
def add(a,b):return [x+y for x,y in zip(a,b)]
def matvec(m,v):return [dot(row,v) for row in m]
def basis(m):
 if type(m) is not list or len(m)!=3:raise ValueError('3x3 basis')
 for row in m:vector(row)
 if any(abs(dot(m[i],m[j])-(1. if i==j else 0.))>1e-10 for i in range(3) for j in range(3)):raise ValueError('orthonormal basis required')
 det=m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])-m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])+m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0])
 if abs(det-1.)>1e-10:raise ValueError('right handed basis required')
def rotation(yaw,pitch):
 if not finite(yaw) or not finite(pitch):raise ValueError('finite angles')
 c,s=math.cos(yaw),math.sin(yaw);a,b=math.cos(pitch),math.sin(pitch)
 return [[c*a,-s,-c*b],[s*a,c,-s*b],[b,0.,a]]
def evaluate(points,cradle_origin,cradle_basis,note_center,note_basis,major,tube):
 vector(cradle_origin);vector(note_center);basis(cradle_basis);basis(note_basis)
 if not finite(major) or not finite(tube) or not 0<tube<major:raise ValueError('torus radii')
 if type(points) is not list or not points:raise ValueError('finite patch inventory')
 out=[]
 for p in points:
  if type(p) is not dict or set(p)!={'motor','cradlePointM','normalCradle'}:raise ValueError('complete patch fields required')
  vector(p['cradlePointM']);vector(p['normalCradle'])
  if abs(dot(p['normalCradle'],p['normalCradle'])-1.)>1e-10:raise ValueError('unit material normal required')
  if type(p['motor']) is not int or p['motor'] not in (0,1):raise ValueError('motor mapping')
  world=add(cradle_origin,matvec(cradle_basis,p['cradlePointM']));lever=[x-y for x,y in zip(world,note_center)]
  local=[sum(note_basis[j][i]*lever[j] for j in range(3)) for i in range(3)]
  implicit=(math.hypot(local[0],local[1])-major)**2+local[2]**2-tube*tube
  out.append(dict(motor=p['motor'],pointWorldM=world,normalWorld=matvec(cradle_basis,p['normalCradle']),leverWorldM=lever,torusImplicitM2=implicit,valid=implicit<=1e-12))
 return dict(points=out,allNominalWitnessesInside=all(p['valid'] for p in out),acceptance=False)

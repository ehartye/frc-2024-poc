"""Conservative flat-torus/AABB domain gate; positive candidates are UNKNOWN."""
import math

def torus_box_distance(center,major,bounds):
 x,y,z=center;lo=bounds[:3];hi=bounds[3:]
 near=math.hypot(max(lo[0]-x,0,x-hi[0]),max(lo[1]-y,0,y-hi[1]))
 far=math.hypot(max(abs(lo[0]-x),abs(hi[0]-x)),max(abs(lo[1]-y),abs(hi[1]-y)))
 planar=max(near-major,major-far,0.)
 vertical=max(lo[2]-z,0,z-hi[2])
 return math.hypot(planar,vertical)

def torus_vertical_cylinder_distance(center,major,bounds):
 radius=(bounds[3]-bounds[0])/2
 if abs(radius-(bounds[4]-bounds[1])/2)>1e-6:raise ValueError('circular vertical cylinder bound required')
 d=math.hypot(center[0]-(bounds[0]+bounds[3])/2,center[1]-(bounds[1]+bounds[4])/2)
 return math.hypot(max(abs(d-major)-radius,0.),max(bounds[2]-center[2],0.,center[2]-bounds[5]))

def make_gate(report,major,tube,height,refine_cylinders=True):
 if report['schema']!='conservative-floor-obstacles/1' or report['assemblyPartCount']!=711:raise ValueError('source obstacle inventory')
 if set(report['excludedContactParts'])!={'intake candidate side pinch belt -1','intake candidate side pinch belt 1'}:raise ValueError('only exact belts excluded')
 obstacles=[]
 for o in report['obstacles']:
  b=o['boundsMm']
  if len(b)!=6 or any(type(v) not in [int,float] or not math.isfinite(v) for v in b) or any(b[i]>b[i+3] for i in range(3)):raise ValueError('finite ordered box')
  if b[2]>height+tube or b[5]<height-tube or b[1]>20+major+tube or b[4]<-20-major-tube or b[0]>658+major+tube or b[3]<178-major-tube:continue
  obstacles.append(o)
 def gate(center,R,r):
  if R!=major or r!=tube or abs(center[2]-height)>1e-9:raise ValueError('bound source torus changed')
  if not 178<=center[0]<=658 or abs(center[1])>20:return dict(status='OUTSIDE_BOUNDED_PLANAR_EXPERIMENT')
  for o in obstacles:
   refined=refine_cylinders and (o['basis']=='configured source profile band conservative box' or o['basis']=='full native bearing outer-bounds correction; existing unseated center, both flange directions')
   distance=torus_vertical_cylinder_distance(center,R,o['boundsMm']) if refined else torus_box_distance(center,R,o['boundsMm'])
   if distance<=r+1e-7:return dict(status='UNKNOWN_HARD_OR_RESERVE_CANDIDATE: '+o['name'],obstacle=o)
  return dict(status='SUPPORTED')
 gate.candidate_count=len(obstacles)
 return gate

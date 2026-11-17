"""Conservative separation only; overlapping bounds never establish contact or clearance."""
import math

def _number(x):
 if type(x) not in (int,float) or not math.isfinite(x):raise ValueError('finite numeric scalar required')
 return float(x)

def query(center_robot_m,axis_robot,contract,*,margin_m=1e-9):
 """Exact torus AABB versus enclosing authored-guide AABB, all orientations.
 axis_robot is the unit torus symmetry axis. Units metres, robot frame.
 Preserves global native UNKNOWN metadata; never resolves a Boolean disagreement.
 """
 if len(center_robot_m)!=3 or len(axis_robot)!=3:raise ValueError('XYZ triples required')
 c=list(map(_number,center_robot_m));n=list(map(_number,axis_robot));margin=_number(margin_m)
 if margin<0 or abs(sum(x*x for x in n)-1)>1e-10:raise ValueError('unit axis and nonnegative margin required')
 g=contract['guide'];major=_number(contract['majorRadiusMm'])/1000;tube=_number(contract['tubeRadiusMm'])/1000
 if not major>tube>0:raise ValueError('ring torus radii required')
 lo=[_number(g['guideFaceXmm'])/1000,-_number(g['halfWidthMm'])/1000,_number(g['bottomZmm'])/1000]
 hi=[_number(g['supportInboardFaceXmm'])/1000,-lo[1],_number(g['topZmm'])/1000]
 if any(a>=b for a,b in zip(lo,hi)):raise ValueError('positive guide extents required')
 extent=[major*math.sqrt(max(0.,1-x*x))+tube for x in n]
 lower=[x-e for x,e in zip(c,extent)];upper=[x+e for x,e in zip(c,extent)]
 gaps=[max(a-v,u-b) for a,b,u,v in zip(lo,hi,lower,upper)]
 separated=max(gaps)>margin
 return dict(status='CERTIFIED_AABB_SEPARATION' if separated else 'UNKNOWN_OVERLAPPING_BOUNDS',separated=separated,geometryDomainValid=separated,axisGapM=gaps,marginM=margin,torusBoundsM=[lower,upper],guideBoundsM=[lo,hi],frame='robot',nativeMaterialStatus='UNKNOWN_BOOLEAN_WITNESS_DISAGREEMENT' if contract['nativeConsistencyUnknown'] else 'NOT_QUALIFIED',receiptSha256=contract['receiptSha256'],acceptance=False)

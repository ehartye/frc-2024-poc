"""Standalone shallow finite-belt penalty density on material NOTE surface samples.
No calibrated material, friction, capture, or continuous checkpoint handoff.
"""
import math,pathlib,sys
R=pathlib.Path(__file__).resolve().parents[2];sys.path.insert(0,str(R/'cad/assembly'))
import belt_contact_geometry as belt
class DomainError(ValueError):pass

def scalar(x):
 if type(x) not in (float,int) or not math.isfinite(x):raise ValueError('finite scalar')
 return float(x)
def vec(v):
 if len(v)!=3:raise ValueError('XYZ vector')
 return [scalar(x) for x in v]
def add(a,b):return [x+y for x,y in zip(a,b)]
def sub(a,b):return [x-y for x,y in zip(a,b)]
def scale(a,k):return [x*k for x in a]
def dot(a,b):return sum(x*y for x,y in zip(a,b))
def cross(a,b):return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
def mv(m,v):return [dot(r,v) for r in m]
def mtv(m,v):return [sum(m[j][i]*v[j] for j in range(3)) for i in range(3)]
def basis(m):
 if len(m)!=3:raise ValueError('rotation matrix')
 m=[vec(r) for r in m]
 if max(abs(dot(m[i],m[j])-(i==j)) for i in range(3) for j in range(3))>1e-10 or dot(cross(m[0],m[1]),m[2])<1-1e-10:raise ValueError('right handed orthonormal basis')
 return m
I=[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]]

def material_points(major,tube,nmajor,nminor):
 major=scalar(major);tube=scalar(tube)
 if not major>tube>0 or any(type(n) is not int or not 8<=n<=2048 for n in (nmajor,nminor)):raise ValueError('ring torus and integer resolution >=8')
 da=2*math.pi/nmajor;db=2*math.pi/nminor
 for i in range(nmajor):
  a=(i+.5)*da
  for j in range(nminor):
   b=(j+.5)*db;radius=major+tube*math.cos(b)
   yield [radius*math.cos(a),radius*math.sin(a),tube*math.sin(b)],tube*radius*da*db

def shallow_gradient(point,side,g):
 """SI point, exact source SDF, analytic derivative only on shallow supported features."""
 p=scale(vec(point),1000.);q=belt.query(p,side,g);s=q['signedDistanceToMaterialMm']/1000.
 if s>=0:return dict(penetrationM=0.,normal=None,feature='separated',sourceSdfM=s)
 x,y,z=p;ax=min(g['frontX'],max(g['rearX'],x));dx=x-ax;dy=y-side*g['axisYAbs'];rad=math.hypot(dx,dy)
 if rad==0:raise DomainError('UNSUPPORTED_CAPSULE_AXIS')
 radial=rad-g['outerRadius'];inner=g['innerRadius']-rad;height=abs(z)-g['halfHeight']
 if inner>=radial or side*dy>=0:raise DomainError('UNSUPPORTED_DEEP_OR_CAVITY_CONTACT')
 if x<g['rearX'] or x>g['frontX']:raise DomainError('UNSUPPORTED_ENTRY_WRAP_CONTACT')
 if abs(radial-height)<1e-9:raise DomainError('UNSUPPORTED_SDF_GRADIENT_TIE')
 normal=[dx/rad,dy/rad,0.] if radial>height else [0.,0.,1. if z>0 else -1.]
 return dict(penetrationM=-s,normal=normal,feature='outer-straight' if radial>height else 'finite-height-cap',sourceSdfM=s)

def evaluate(config,*,note_center,note_basis=I,cradle_origin=(0.,0.,0.),cradle_basis=I,u_positive_m=0.,k_area=1e7,resolution=(128,64),note_velocity=(0.,0.,0.),note_omega=(0.,0.,0.),cradle_velocity=(0.,0.,0.),cradle_omega=(0.,0.,0.),u_velocity=0.):
 """Positive-side u moves that cassette -cradleY; negative cassette fixed.
 U=sum(0.5*k_area*delta^2*material_area). No active-cell renormalization.
 Forces are exact derivatives of this discrete potential on supported branches.
 """
 center=vec(note_center);origin=vec(cradle_origin);nb=basis(note_basis);cb=basis(cradle_basis);nv=vec(note_velocity);nw=vec(note_omega);cv=vec(cradle_velocity);cw=vec(cradle_omega);u=scalar(u_positive_m);ud=scalar(u_velocity);k=scalar(k_area)
 if not 0<=u<=.0015 or k<=0:raise ValueError('source travel or positive penalty density')
 g0=belt.geometry(config);major=config['note']['majorRadius']/1000.;tube=config['note']['tubeRadius']/1000.
 force=[0.,0.,0.];moment=[0.,0.,0.];reaction=[0.,0.,0.];reaction_moment=[0.,0.,0.];potential=0.;qu=0.;active=0;maxdepth=0.;features={};sides={-1:dict(forceWorldN=[0.,0.,0.],normalMagnitudeN=0.,activeAreaM2=0.),1:dict(forceWorldN=[0.,0.,0.],normalMagnitudeN=0.,activeAreaM2=0.)}
 for local,area in material_points(major,tube,*resolution):
  lever=mv(nb,local);point=add(center,lever);cradle_point=mtv(cb,sub(point,origin));arm=sub(point,origin)
  for side in (-1,1):
   g=dict(g0)
   if side==1:g['axisYAbs']-=u*1000.
   q=shallow_gradient(cradle_point,side,g);depth=q['penetrationM']
   if depth==0:continue
   normal=mv(cb,q['normal']);f=scale(normal,k*depth*area);back=scale(f,-1.);potential+=.5*k*depth*depth*area;force=add(force,f);moment=add(moment,cross(lever,f));reaction=add(reaction,back);reaction_moment=add(reaction_moment,cross(arm,back));active+=1;maxdepth=max(maxdepth,depth);features[q['feature']]=features.get(q['feature'],0)+1
   sides[side]['forceWorldN']=add(sides[side]['forceWorldN'],f);sides[side]['normalMagnitudeN']+=k*depth*area;sides[side]['activeAreaM2']+=area
   if side==1:qu+=dot(back,mv(cb,[0.,-1.,0.]))
 bodywork=dot(force,nv)+dot(moment,nw);basework=dot(reaction,cv)+dot(reaction_moment,cw);uwork=qu*ud
 result=dict(potentialJ=potential,noteForceWorldN=force,noteMomentWorldNm=moment,cradleReactionWorldN=reaction,cradleMomentWorldNm=reaction_moment,cassetteGeneralizedForceN=qu,bodyWorkRateW=bodywork,cradleWorkRateW=basework,cassetteWorkRateW=uwork,predictedPotentialRateW=-(bodywork+basework+uwork),activeSamples=active,features=features,maxSampledPenetrationM=maxdepth,sides=sides,kAreaNpm3=k,resolution=list(resolution),acceptance=False,scope='sampled shallow straight-face/cap penalty; no entry-wrap, friction, spring equilibrium, hardware acceptance or handoff continuity')
 def check(value):
  if isinstance(value,dict):
   for v in value.values():check(v)
  elif isinstance(value,list):
   for v in value:check(v)
  elif isinstance(value,float) and not math.isfinite(value):raise DomainError('NONFINITE_DERIVED_RESULT')
 check(result)
 return result

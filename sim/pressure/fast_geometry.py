"""Same material quadrature and scalar contact law; conservative point culling only."""
import math,functools
import numpy as np
import finite_transport as reference
import finite_area as a

@functools.lru_cache(maxsize=2)
def material_grid(major,tube,nmajor,nminor):
 rows=np.array([(*p,w) for p,w in a.material_points(major,tube,nmajor,nminor)],dtype=np.float64);rows.flags.writeable=False;return rows

def prepare(config,*,note_center,note_basis=a.I,cradle_origin=(0.,0.,0.),cradle_basis=a.I,u_positive_m=0.,k_area=1e7,resolution=(128,128)):
 center=a.vec(note_center);origin=a.vec(cradle_origin);nb=a.basis(note_basis);cb=a.basis(cradle_basis);u=a.scalar(u_positive_m);k=a.scalar(k_area)
 if not 0<=u<=.0015 or k<=0:raise ValueError('source travel and positive penalty density')
 g0=a.belt.geometry(config);major=config['note']['majorRadius']/1000.;tube=config['note']['tubeRadius']/1000.;next(a.material_points(major,tube,*resolution))
 # Extreme finite inputs must pass scalar transform/mm validation, even if broadphase separates.
 # Conservative fallback preserves reference exception type/message and cancellation behavior.
 if max(map(abs,center+origin+[major,tube]))>np.finfo(float).max/1e6:
  return reference.prepare(config,note_center=center,note_basis=nb,cradle_origin=origin,cradle_basis=cb,u_positive_m=u,k_area=k,resolution=resolution)
 empty=dict(contacts=[],potentialJ=0.,features={},cassetteDirectionWorld=a.mv(cb,[0.,-1.,0.]),resolution=list(resolution),kAreaNpm3=k,acceptance=False)
 boxes=[]
 for side in (-1,1):
  axis=side*(g0['axisYAbs']-(u*1000 if side==1 else 0));boxes.append(([ (g0['rearX']-g0['outerRadius'])/1000,(axis-g0['outerRadius'])/1000,-g0['halfHeight']/1000],[(g0['frontX']+g0['outerRadius'])/1000,(axis+g0['outerRadius'])/1000,g0['halfHeight']/1000]))
 relative=a.mtv(cb,a.sub(center,origin));transform=[[sum(cb[j][i]*nb[j][q] for j in range(3)) for q in range(3)] for i in range(3)]
 extent=[major*math.hypot(row[0],row[1])+tube*math.hypot(*row) for row in transform]
 pad=max(1e-12,64*np.finfo(float).eps*(1+max(map(abs,center+origin+extent))))
 if not all(math.isfinite(v) for v in relative+extent+[pad]+[v for lo,hi in boxes for v in lo+hi]):raise a.DomainError('NONFINITE_GEOMETRY_BOUND')
 if all(any(relative[i]+extent[i]+pad<lo[i] or relative[i]-extent[i]-pad>hi[i] for i in range(3)) for lo,hi in boxes):return empty
 grid=material_grid(major,tube,*resolution);lever=np.zeros((len(grid),3));local=np.zeros_like(lever)
 # Vector transforms are conservative culling candidates only; retained points are recomputed by scalar helpers.
 for i in range(3):
  for j in range(3):lever[:,i]+=grid[:,j]*nb[i][j]
 world=lever+np.array(center);arm=world-np.array(origin)
 for i in range(3):
  for j in range(3):local[:,i]+=arm[:,j]*cb[j][i]
 if not np.isfinite(local).all() or np.max(np.abs(local))>np.finfo(float).max/1000:raise a.DomainError('NONFINITE_MILLIMETRE_POINT')
 mask=np.zeros(len(grid),dtype=bool)
 for lo,hi in boxes:mask|=np.all((local>=np.array(lo)-pad)&(local<=np.array(hi)+pad),axis=1)
 contacts=[];potential=0.;features={}
 for index in np.flatnonzero(mask):
  lv=a.mv(nb,grid[index,:3].tolist());point=a.add(center,lv);ar=a.sub(point,origin);p=a.mtv(cb,ar);area=float(grid[index,3])
  for side in (-1,1):
   g=dict(g0)
   if side==1:g['axisYAbs']-=u*1000.
   q=reference.gradient(p,side,g);depth=q['penetrationM']
   if depth==0:continue
   wall=a.belt.query(a.scale(p,1000.),side,g)['outerSidewall']
   if wall is None or not wall['noteFacingHalf']:raise a.DomainError('UNSUPPORTED_BELT_TANGENT')
   normal=a.mv(cb,q['normal']);tangent=a.mv(cb,wall['acquisitionUnitTangent'])
   if abs(a.dot(normal,tangent))>1e-10:raise a.DomainError('NORMAL_TANGENT_NOT_ORTHOGONAL')
   fn=k*depth*area;energy=.5*k*depth*depth*area
   if not all(math.isfinite(x) for x in [fn,energy]):raise a.DomainError('NONFINITE_CONTACT_RESULT')
   potential+=energy;features[q['feature']]=features.get(q['feature'],0)+1
   contacts.append(dict(side=side,motor=0 if side==-1 else 1,pointWorldM=point,noteLeverWorldM=lv,cradleLeverWorldM=ar,normalWorld=normal,acquisitionTangentWorld=tangent,normalN=fn,areaM2=area,penetrationM=depth,feature=q['feature']))
 return dict(empty,contacts=contacts,potentialJ=potential,features=features)

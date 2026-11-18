"""Fixed-normal-load, maximum-dissipation Coulomb impulse hypothesis.

Each finite-area pressure sample retains its own isotropic friction disk.
The solver changes velocities through reciprocal generalized impulses only.
"""
import numpy as np

class FrictionSolveError(ValueError):pass

def solve_disks(jacobian,mass,velocity,capacities,*,gap_tolerance=1e-12,max_iterations=1000, initial_impulse=None):
 j=np.asarray(jacobian,dtype=float);m=np.asarray(mass,dtype=float);v=np.asarray(velocity,dtype=float);cap=np.asarray(capacities,dtype=float)
 if j.ndim!=3 or j.shape[1]!=2 or j.shape[0]!=len(cap) or j.shape[2]!=len(v) or m.shape!=(len(v),len(v)):raise ValueError('friction dimensions')
 if not all(np.isfinite(a).all() for a in [j,m,v,cap]) or np.any(cap<0) or gap_tolerance<=0 or max_iterations<1:raise ValueError('friction domain')
 if not np.allclose(m,m.T,atol=1e-12,rtol=0):raise ValueError('symmetric mass')
 chol=np.linalg.cholesky(m);mi=np.linalg.solve(m,np.eye(len(v)));root=np.sqrt(cap);b=(j*root[:,None,None]).reshape(-1,len(v));whitened=np.linalg.solve(chol,b.T).T
 if len(cap)==0 or np.max(cap,initial=0)==0:return dict(velocity=v.copy(),impulse=np.zeros((len(cap),2)),iterations=0,gapJ=0.,heatJ=0.,projectionDissipationJ=0.,projectionIdentityJ=0.,maximumConeExcessNs=0.)
 lipschitz=float(np.linalg.eigvalsh(whitened.T@whitened)[-1]);g=b@v;y=np.zeros((len(cap),2))
 if initial_impulse is not None:
  seed=np.asarray(initial_impulse,dtype=float)
  if seed.shape!=y.shape or not np.isfinite(seed).all():raise ValueError('friction seed')
  y=np.divide(seed,root[:,None],out=y,where=root[:,None]>0)
  norm=np.linalg.norm(y,axis=1);y*=np.minimum(1.,np.divide(root,norm,out=np.ones_like(root),where=norm>0))[:,None]
 z=y.copy();t=1.
 if lipschitz<=0:raise FrictionSolveError('ZERO_CONTACT_INVERSE_MASS')
 for iteration in range(1,max_iterations+1):
  gradient=(b@(mi@(b.T@z.ravel()))+g).reshape(-1,2);candidate=z-gradient/lipschitz;norm=np.linalg.norm(candidate,axis=1);candidate*=np.minimum(1.,np.divide(root,norm,out=np.ones_like(root),where=norm>0))[:,None]
  new_t=(1+np.sqrt(1+4*t*t))/2;z=candidate+(t-1)/new_t*(candidate-y);y=candidate;t=new_t
  if iteration%10==0 or iteration==max_iterations:
   impulse=y*root[:,None];dv=mi@(j.reshape(-1,len(v)).T@impulse.ravel());end=v+dv;slip=np.einsum('nik,k->ni',j,end);gap=float(np.sum(impulse*slip)+np.sum(cap*np.linalg.norm(slip,axis=1)))
   if gap<=gap_tolerance:break
 else:raise FrictionSolveError('FRICTION_ITERATION_LIMIT')
 if gap>gap_tolerance:raise FrictionSolveError('FRICTION_GAP_NOT_RESOLVED')
 excess=float(np.max(np.linalg.norm(impulse,axis=1)-cap,initial=0));heat=-float(np.sum(impulse*slip));diss=float(.5*dv@m@dv);identity=float(.5*(end@m@end-v@m@v)+heat+diss)
 if excess>1e-12 or heat< -gap_tolerance or abs(identity)>1e-9:raise FrictionSolveError('FRICTION_WORK_OR_CONE')
 return dict(velocity=end,impulse=impulse,iterations=iteration,gapJ=gap,heatJ=heat,projectionDissipationJ=diss,projectionIdentityJ=identity,maximumConeExcessNs=excess)


class WarmSolve:
 """Numerical iterate reuse only; every accepted solve meets unchanged gates.

 Point indices may change between calls: an arbitrary finite seed is permitted,
 projected onto the current disks, and cannot bypass the current optimality test.
 This cache is not tangential elasticity, friction memory or physical state.
 """
 def __init__(self):self.previous=None
 def __call__(self,jacobian,mass,velocity,capacities,**kwargs):
  seed=self.previous if self.previous is not None and self.previous.shape==(len(capacities),2) else None
  result=solve_disks(jacobian,mass,velocity,capacities,initial_impulse=seed,**kwargs)
  self.previous=result['impulse'].copy()
  return result

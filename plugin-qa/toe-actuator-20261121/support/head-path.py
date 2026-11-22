from pathlib import Path
import runpy,json,numpy as np,math
q=Path(__file__).parent;parent=q.parent;w=runpy.run_path(str(parent/'rack-fit.py'));b=runpy.run_path(str(parent/'bounds-fit.py'));rotq=runpy.run_path(str(parent/'ring-fit.py'))['rotq']
bits={}
for part in w['make'](rx=-106,rz=200):
 lo,hi=b['localBounds'](part);bits[part.name]=(part.k,lo,hi)
for r in json.loads((q/'roster.json').read_bytes()):
 if 'home switch' in r['name']:
  lo,hi=np.array(r['bbox']).reshape(2,3)
  if 'rear' in r['name']:lo[2]-=13;hi[2]-=13
  bits[r['name']]=(None,lo,hi)
for name,lo,hi in [
 ('high rail and elbow plate',[-182.372,-19,216.95],[-41,-13,225.95]),('upper rear offset stem',[-177,-16,217],[-169,-13,237.45]),('top bridge',[-177,-16,231.45],[-169,2,237.45]),('vertical lower rear',[-177,-2,82],[-169,2,237.45]),('low rear toe arm',[-177,-3,82],[-72,3,87])]:bits[name]=('rear',np.array(lo),np.array(hi))
res={}
for n in (96,192):
 a=np.load(parent/f'ring-{n}'/'ring.npy');t=a[:,0];N=len(t);ang=a[:,5];sn=np.sin(ang);cs=np.cos(ang);f=np.zeros((N,3,3));f[:,0,0]=cs;f[:,0,2]=-sn;f[:,1,1]=1;f[:,2,0]=sn;f[:,2,2]=cs
 calcs={}
 for field,anchor in [('floor',9),('preload',16)]:
  center=a[:,anchor:anchor+3].copy()*1000;center[:,:2]-=a[:,1:3]*1000;center[:,[0,2]]-=a[:,[3,4]]*1000;loc=np.einsum('nij,nj->ni',f,center);rot=f@rotq(a[:,anchor+3:anchor+7]);alpha=np.arange(256)*2*np.pi/256;nodes=np.stack([np.cos(alpha),np.sin(alpha),np.zeros(256)])*152.4
  for name,(lab,lo,hi) in bits.items():
   c=(lo+hi)/2;e=(hi-lo)/2;d=np.zeros_like(loc)
   if lab=='left':d[:,1]=a[:,6]*1000
   if lab=='right':d[:,1]=-a[:,7]*1000
   if lab=='rear':d[:,0]=-a[:,8]*1000
   vec=loc-d-c;cd=np.linalg.norm(np.maximum(0,abs(vec)-e),axis=1);idx=np.argsort(cd);best=1e6
   for k in range(0,N,96):
    ii=idx[k:k+96]
    if cd[ii[0]]-177.8>=best:break
    delta=vec[ii,:,None]+rot[ii]@nodes;dist=np.linalg.norm(np.maximum(0,abs(delta)-e[None,:,None]),axis=1).min(axis=1)-25.4;ii0=np.argmin(dist)
    if dist[ii0]<best:best=float(dist[ii0]);wi=ii[ii0]
   calcs[name+'/'+field]=[best-152.4*np.pi/256-152.4*(1-np.cos(np.pi/n)),float(t[wi])]
 # Exact axis-aligned local boxes for the bottom rear link and post, versus bumper OBB on every sampled wrist position.
 for name,(lab,lo,hi) in bits.items():
  best=[1e4,0];e=(hi-lo)/2;c=(lo+hi)/2
  dl=np.zeros((N,3))
  if lab=='rear':dl[:,0]=-a[:,8]*1000
  if lab=='left':dl[:,1]=a[:,6]*1000
  if lab=='right':dl[:,1]=-a[:,7]*1000
  cv=np.einsum('nij,nj->ni',np.swapaxes(f,1,2),c+dl);cv[:,[0,2]]+=a[:,[3,4]]*1000
  center,ext,_=w['cover'];rel=cv-center
  # SAT restricted to common axes of the Y-aligned pitch rotation.
  gaps=np.stack([abs(rel[:,0])-ext[0]-abs(cs)*e[0]-abs(sn)*e[2],abs(rel[:,2])-ext[2]-abs(sn)*e[0]-abs(cs)*e[2],abs(rel[:,1])-ext[1]-e[1],abs(cs*rel[:,0]-sn*rel[:,2])-e[0]-abs(cs)*ext[0]-abs(sn)*ext[2],abs(sn*rel[:,0]+cs*rel[:,2])-e[2]-abs(sn)*ext[0]-abs(cs)*ext[2]],axis=1).max(axis=1)
  i=np.argmin(gaps);calcs[name+'/bumper']=[float(gaps[i]),float(t[i])]
 res[n]=dict(clockSHA=w['H'](parent/f'ring-{n}'/'ring.npy'),sampledPoses=N,clearanceLowerBounds=calcs);print(n,sorted(calcs.items(),key=lambda t:t[1][0])[:7],flush=True)
(q/'head-path.json').write_text(json.dumps(res,indent=1))

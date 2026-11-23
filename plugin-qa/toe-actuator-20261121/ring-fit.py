from pathlib import Path
import sys,json,math,hashlib,runpy
import numpy as np
q=Path(__file__).parent;w=runpy.run_path(str(q/'rack-fit.py'));b=runpy.run_path(str(q/'bounds-fit.py'))
def rotq(U):
 W,X,Y,Z=U.T
 return np.stack([1-2*(Y*Y+Z*Z),2*(X*Y-W*Z),2*(X*Z+W*Y),2*(X*Y+W*Z),1-2*(X*X+Z*Z),2*(Y*Z-W*X),2*(X*Z-W*Y),2*(Y*Z+W*X),1-2*(X*X+Y*Y)],axis=1).reshape(-1,3,3)
def calc(n,nodes=256):
 f=q/f'ring-{n}';c=w['repo']/f'head-clear-{n}'
 assert all(json.loads((f/k).read_bytes())==json.loads((c/k).read_bytes())for k in ['run.json','floor/cycle.json','preload/cycle.json'])
 a=np.load(f/'ring.npy');t=a[:,0];theta=a[:,5];ct=np.cos(theta);st=np.sin(theta);centerHead=a[:,[3,4]]*1000;alpha=np.arange(nodes)*2*np.pi/nodes;uv=np.stack([np.cos(alpha),np.sin(alpha),np.zeros(nodes)],axis=0)*152.4
 frame=np.zeros((len(t),3,3));frame[:,0,0]=ct;frame[:,0,2]=-st;frame[:,1,1]=1;frame[:,2,0]=st;frame[:,2,2]=ct
 parts=w['make'](rz=185);boxes=[b['localBounds'](p)for p in parts];worst={};error=152.4*np.pi/nodes;chord=152.4*(1-math.cos(math.pi/n))
 for id,k in [('floor',9),('preload',16)]:
  xyz=1000*a[:,k:k+3];xyz[:,:2]-=1000*a[:,1:3];xyz[:,[0,2]]-=centerHead
  center=np.einsum('nij,nj->ni',frame,xyz)
  R=frame@rotq(a[:,k+3:k+7])
  for j,p in enumerate(parts):
   lo,hi=boxes[j];e=(hi-lo)/2;mid=(hi+lo)/2
   disp=np.zeros_like(center)
   if p.k=='left':disp[:,1]=a[:,6]*1000
   elif p.k=='right':disp[:,1]=-a[:,7]*1000
   elif p.k=='rear':disp[:,0]=-a[:,8]*1000
   rel=center-disp-mid
   # The entire torus fits in this sphere. Prune only where the sphere's rigorous lower bound cannot beat the current exact sample minimum.
   centerDist=np.linalg.norm(np.maximum(0,abs(rel)-e),axis=1); idx=np.argsort(centerDist)
   gap=1e12;winner=None
   for start in range(0,len(t),80):
    ii=idx[start:start+80]
    if min(centerDist[ii])-177.8>=gap:break
    pts=rel[ii,:,None]+R[ii]@uv
    d=np.sqrt((np.maximum(0,abs(pts)-e[None,:,None])**2).sum(axis=1)).min(axis=1)-25.4
    z=np.argmin(d)
    if d[z]<gap:
     gap=d[z];winner=ii[z]
   row=a[winner].tolist();jpos=disp[winner].tolist()
   worst[id+'/'+p.name]=dict(t=t[winner],nodeToEnvelopeGapMm=gap,certifiedSampledLowerBoundMm=gap-error-chord,ringCenterHead=center[winner].tolist(),ringNormalHead=R[winner,:,2].tolist(),translatePartHead=jpos,ring=id,part=p.name)
 out=dict(n=n,poseSource=w['H'](f/'ring.npy'),run=w['H'](f/'run.json'),sampleDtS=.002,poses=len(a),centerlinePoints=nodes,centerlineInterpolationMm=error,capsuleVsTorusSagMm=chord,receipt='same phase original, new geometry as observer only',rows=worst)
 (q/f'free-ring-{n}.json').write_text(json.dumps(out,indent=2))
 print(n,'smallest',sorted([(r['certifiedSampledLowerBoundMm'],r['part'],r['t'],r['ring']) for r in worst.values()])[:7],flush=True)
if __name__=='__main__':calc(int(sys.argv[1]))


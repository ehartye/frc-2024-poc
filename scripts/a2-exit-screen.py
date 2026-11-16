"""Actual exit-row sampled NOTE/aperture screen. Not continuous clearance or scoring."""
import argparse,csv,hashlib,json,math,pathlib
import numpy as np

def screen(row,nt=256,np_=64):
 keys=['pieceXWorld','pieceYWorld','pieceZWorld','exitVx','exitVy','exitVz','bodyPitchRad']
 try:values=[float(row[k]) for k in keys]
 except (KeyError,ValueError,TypeError):return {'status':'unknown','reason':'missing actual exit or attitude hypothesis'}
 if not all(math.isfinite(v) for v in values):raise ValueError('nonfinite exit')
 x,y,z,vx,vy,vz,pitch=values
 # Source-local target frame is world yaw180; assembly bottom provisionally equals floor.
 p=np.array([-x,-y,z]);v=np.array([-vx,-vy,vz]);horizontal=math.hypot(vx,vy)
 if horizontal<=0:return {'status':'unknown','reason':'no forward velocity'}
 forward=np.array([-vx/horizontal,-vy/horizontal,0]);lateral=np.array([-forward[1],forward[0],0]);vertical=np.array([0,0,1]);e=math.cos(pitch)*forward+math.sin(pitch)*vertical;axis=-math.sin(pitch)*forward+math.cos(pitch)*vertical
 theta=np.arange(nt)*2*math.pi/nt;phi=np.arange(np_)*2*math.pi/np_;radial=np.cos(theta)[:,None]*e+np.sin(theta)[:,None]*lateral
 q=((.1524+.0254*np.cos(phi))[None,:,None]*radial[:,None,:]+.0254*np.sin(phi)[None,:,None]*axis).reshape(-1,3)
 alpha=math.atan2(82.9-78.13,18.11);u=np.array([math.cos(alpha),0,math.sin(alpha)]);n=np.array([-math.sin(alpha),0,math.cos(alpha)]);lo=np.array([0,0,78.13*.0254]);length=math.hypot(18.11,82.9-78.13)*.0254;width=41.38*.0254
 A=-.5*9.80665*n[2];B=float(v@n);C=(p+q-lo)@n;D=B*B-4*A*C
 if np.any(D<0):return {'status':'sampled-failure','reason':'some sampled surface points never reach plane'}
 times=(-B+np.sqrt(D))/(2*A)
 if np.any(times<0):return {'status':'sampled-failure','reason':'no future ascending whole-sample crossing'}
 positions=p+q+times[:,None]*v;positions[:,2]-=.5*9.80665*times*times;along=(positions-lo)@u
 margin=float(min(along.min(),length-along.max(),width/2-np.abs(positions[:,1]).max()))
 return {'status':'sampled-pass' if margin>0 else 'sampled-failure','sampledMarginM':margin,'surfaceSamples':len(q),'firstCrossingS':float(times.min()),'lastCrossingS':float(times.max()),'actualSpeedMps':float(np.linalg.norm(v)),'fixedBodyPitchHypothesisRad':pitch,'spin':'unknown; fixed attitude is an explicit hypothesis'}

def report(trace,nt=256,np_=64):
 trace=pathlib.Path(trace)
 with trace.open(newline='',encoding='utf-8') as f:rows=[r for r in csv.DictReader(f) if r.get('event')=='EXIT']
 return {'schema':'frc-a2-actual-exit-screen/1','traceSha256':hashlib.sha256(trace.read_bytes()).hexdigest(),'cases':[dict(timeS=float(r['timeS']),**screen(r,nt,np_)) for r in rows],'officialScore':None,'limits':['Sampled torus surface, not continuous certificate','No spin, aerodynamic drag, deformation or roof/wall collision','Assembly bottom-to-floor and target registration are provisional','Input hash binds this report to trace, not independently to execution'],'source':{'url':'https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024FieldDrawings.pdf','dimensionsInches':{'low':78.13,'high':82.9,'reach':18.11,'width':41.38},'scope':'drawing scalar opening dimensions, not complete installed target'}}
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--trace',required=True);p.add_argument('--output',required=True);a=p.parse_args();out=pathlib.Path(a.output)
 if out.exists():raise ValueError('Fresh output required')
 out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(report(a.trace),indent=2,allow_nan=False)+'\n',encoding='utf-8')

"""Find an interval of common station X that covers observed free release states, including rotating ring extent."""
from pathlib import Path
import json,math,numpy as np
from scipy.spatial.transform import Rotation
p=Path(__file__).resolve().parent
pitch=np.array([.459994,0,.121269]);L=np.linalg.norm(pitch);u=pitch/L;norm=np.cross(u,[0,1,0]);low=np.array([-.001050,5.5458275,1.984464])
stations=np.arange(1.19,1.76,.01)
def margin(row):
    q,v=np.array(row['q']),np.array(row['v']);axes=Rotation.from_quat([*q[4:],q[3]]).as_matrix();m=.1524;minor=.0254
    it=m*m/2+5*minor**2/8;iz=m*m+3*minor**2/4
    mom=axes@(v[3:]*[it,it,iz]);w=np.linalg.norm(mom)/it;k=mom/np.linalg.norm(mom)
    a=np.arange(256)*math.tau/256;ring=m*(np.outer(np.cos(a),axes[:,0])+np.outer(np.sin(a),axes[:,1]))
    para=np.outer(ring@k,k);tt=np.arange(0,.58,.001);tm=tt[:,None,None]
    vv=para+(ring-para)*np.cos(w*tm)+np.cross(k,ring)*np.sin(w*tm)+q[:3]+v[:3]*tm-low
    vv[:,:,2]-=4.903325*tt[:,None]**2;du=vv@u;dn=vv@norm;dy=vv[:,:,1]
    r=minor+m*math.pi/256+(np.linalg.norm(v[:3])+m*w+5.7)*.0005
    result=[]
    for station in stations:
        dh=station-1.43;hh=dn+dh*norm[0];post=np.flatnonzero(hh.min(axis=1)>r)
        if not len(post):result.append(-1.);continue
        hh=hh[:post[0]+1];near=abs(hh)<=r;disk=np.sqrt(np.maximum(0,r*r-hh[near]**2));along=du[:len(hh)][near]+dh*u[0];y=dy[:len(hh)][near]
        result.append(float(min(np.min(along-disk),np.min(L-along-disk),np.min(.5254625-np.abs(y)-disk))))
    return np.array(result)
out={}
for prefix in ['scored','indexed']:
    data={f.name:margin(json.loads(f.read_bytes())['release']) for f in p.glob(prefix+'-dev-*-48.json')}
    if len(data)!=6:continue
    worst=np.min(list(data.values()),axis=0);at=np.argmax(worst)
    out[prefix]=dict(bestStation=float(stations[at]),worstMargin=float(worst[at]),asDrivenMargin=float(worst[np.argmin(abs(stations-1.43))]),perPiece={k:float(v[at]) for k,v in data.items()},feasible=stations[worst>0].tolist())
(p/'common-aim-window.json').write_text(json.dumps(out,indent=2));print(out)


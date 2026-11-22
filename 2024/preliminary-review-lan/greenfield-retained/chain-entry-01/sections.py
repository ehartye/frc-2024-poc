from pathlib import Path
import numpy as np,json,base64,math,hashlib
p=Path(__file__).resolve().parent;root=p.parent;f=root/'mission-physics/fixtures.json'
# Raw un-decimated source triangulation (field coordinates, millimeters).
chain=json.loads(f.read_bytes())['workingChain'];v=np.frombuffer(base64.b64decode(chain['v']),'<f4').astype(float).reshape(-1,3)*1000
faces=np.frombuffer(base64.b64decode(chain['f']),'<u4').reshape(-1,3);tri=v[faces];fieldY=4104.32
def slab(y,w):
    T=tri[(tri[:,:,1].max(axis=1)>=y-w)&(tri[:,:,1].min(axis=1)<=y+w)]
    pts=[T[(T[:,:,1]>=y-w)&(T[:,:,1]<=y+w)]]
    for a in range(3):
        b=(a+1)%3;dy=T[:,b,1]-T[:,a,1]
        for plane in [y-w,y+w]:
            mask=((T[:,a,1]-plane)*(T[:,b,1]-plane)<=0)&(abs(dy)>1e-9)
            u=(plane-T[mask,a,1])/dy[mask]
            pts.append(T[mask,a]+u[:,None]*(T[mask,b]-T[mask,a]))
    return np.unique(np.round(np.concatenate(pts),6),axis=0)
if __name__=='__main__':
    print(v.min(0),v.max(0))
    for k in [-1,1]:
        ss=slab(fieldY+k*100.88,4)
        print(k,ss.min(0),ss.max(0),len(ss))


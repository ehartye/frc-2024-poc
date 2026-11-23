"""Discriminate a proposed hanging motion using the actual stage leaf and chassis solids."""
from pathlib import Path
import sys,numpy as np,json,math,hashlib
from OCP.BRepTools import BRepTools
from OCP.TopoDS import TopoDS_Shape
from OCP.BRep import BRep_Builder
p=Path(__file__).resolve().parent;root=p.parent;sys.path.insert(0,str(root/'assembly-recheck-01'))
from sweep import load_solids,name
from layout import common,place,mat,bounds,distance
def read(f):return json.loads(f.read_bytes())
def orient(q):
    w,x,y,z=q;return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
scene=read(p/'original-climb.json');pkg=root/'tread-underlay-01/field-review.step'
stock={name(n):s for n,s in load_solids(pkg)[0].items()}
registry=read(p/'field-catalog.json');catalog=[]
for r in registry['items']:
    # Focus first on confirmed bar, the stage's adjacent covers, and the near hook support. Other leaves remain open.
    if not ('Tag14' in r['path'] and r['path'].split('/')[-1] in ('GE-24221-01','GE-24222','GE-24243','GE-24241','GE-24244')):continue
    s=TopoDS_Shape();file=p/'field-stock'/r['file'];assert hashlib.sha256(file.read_bytes()).hexdigest()==r['sha'] and BRepTools.Read_s(s,str(file),BRep_Builder())
    catalog.append((r['path'],s,np.array(r['bounds'])))
bodyNames=[n for n in stock if n in scene['rig'] and n not in scene['removed']]
parts=read(root.parent/'www/data.json');meshes={**parts['bodies'],**parts['concepts']['field']['add']}
verts={n:np.array(meshes[n]['v']) for n in bodyNames}
park=mat(125,185,0,501);ip=np.linalg.inv(park);H=mat(0,457.5,0,96);axis=np.r_[read(p/'outward.json')['axis'],0];base=np.r_[scene['positions'][30],0]
def transforms(r,gate,direction):
    fr=np.eye(4);fr[:3,:3]=orient(r['q']);fr[:3,3]=np.array(r['p'])*1000+base
    z=np.eye(4)
    if direction: z=np.diag([-1.,-1.,1.,1.]);z[:3,3]=[2*axis[0],2*axis[1],0]
    fr=z@fr;arm=fr@mat(r['angle'],185,0,501)
    return dict(frame=fr,arm=arm@ip,flap=arm@H@mat(90*np.clip((r['angle']-102)/18,0,1),0,0,0)@np.linalg.inv(H)@ip,keeper=arm@mat(0,0,0,gate)@ip,pin=fr@mat(0,0,-r['strokeMm'],0))
def trial(label,direction):
    f=p/(label+'.json');data=read(f);seen=[];pos=[];first=0;coarseCount=0
    times=list(dict.fromkeys([4.,5.15,3.,8.,12.,2.4,2.8]+list(np.arange(0,data['trace'][-1]['t']+.001,.2))))
    for t in times:
        i=min(round(t/.025),len(data['trace'])-1);r=data['trace'][i];m=transforms(r,data['gateTrace'][i][1]*1000,direction)
        for n in bodyNames:
            tr=m[scene['rig'][n]];v=verts[n]@tr[:3,:3].T+tr[:3,3];lo=v.min(axis=0)-.25;hi=v.max(axis=0)+.25
            real=None
            for path,s,b in catalog:
                if np.any(lo>b[1]) or np.any(hi<b[0]):continue
                coarseCount+=1
                if real is None:real=place(stock[n],tr)
                g=distance(s,real);vol=common(s,real) if g<.01 else 0;row=[float(t),n,path,g,vol];seen.append(row)
                if vol>.01:
                    pos.append(row);break
            if pos:break
        if pos:break
    out=dict(input=hashlib.sha256(f.read_bytes()).hexdigest(),direction=direction,nominalCatchDeg=label,count=len(seen),testedPoses=times.index(t)+1,possible=len(times),stageOnly=True,close=sorted(seen,key=lambda x:x[3])[:10],positives=pos,scope='Stops on first occupied intersection. A clear result covers five named high-risk source leaves sampled at 0.2 s plus peak observations. Other load/field checks are independent.')
    (p/f'compare-{label}-{direction}.json').write_text(json.dumps(out,indent=2));print(label,direction,coarseCount,pos,flush=True)
if __name__=='__main__':trial(sys.argv[1],int(sys.argv[2]))


"""Read the last complete assembly, apply the live operating rig, and position the battery service pose."""
from pathlib import Path
import sys,math,json,hashlib,numpy as np
p=Path(__file__).resolve().parent;root=p.parent
exec(compile((p/'package.py').read_text('utf8').split('source=root/')[0],'layout BRep helpers','exec'))
sys.path.insert(0,str(root/'follower-01'))
from stroke import scheduled
source=p/'service-index-guide.step';assert digest(source)==read(p/'service-pin.json')['whole']
stock={name(k):s for k,s in load_solids(source)[0].items()};assert len(stock)==232
doc=read(root.parent/'www/data.json');f=doc['concepts']['field'];g=doc['concepts']['guided']
base=next(c for c in f['cases'] if c['key']=='field-r08')
rig={**{k:'arm' for k in g['add'] if not k.startswith('Service slide')},**base['rig']}
rightBlock='Translating bearing block · right';rightFrame='Pivot A-frame · right'
def move(x=0,y=0,z=0):return mat(0,x,y,z)
delta=scheduled(-40)-185
def pose(n,s):
    role=rig.get(n,'frame')
    if n=='Battery service pin':return place(s,move(delta,-12,0))
    if role in ('slide','pin'):return place(s,move(delta))
    if role in ('arm','flap','keeper'):return place(s,mat(-40,scheduled(-40),0,501)@ip)
    return s
scene={n:pose(n,s) for n,s in stock.items() if n not in set(base.get('removed',[]))|{'Pack lead','Distribution lead','Chambered NOTE'}}
def nearby(a,b,tol=0.01):
    a,b=np.array(a),np.array(b);return all(a[:3]<=b[3:]+tol) and all(b[:3]<=a[3:]+tol)
def ypeg(x,z,r,y,h):return cyl((x,y,z),Y,r,h)


"""Recheck the issued 154-member preliminary geometry at the actual coupled slide/pivot poses."""
from pathlib import Path
import json,math,sys,hashlib
import numpy as np
p=Path(__file__).resolve().parent;root=p.parent;sys.path[:0]=[str(root),str(root/'follower-01')]
from probe import names as unused
from layout import mat,Y,bounds,common,distance
from showcase_contacts import load_solids
from stroke import scheduled
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.gp import gp_Trsf
from OCP.TopLoc import TopLoc_Location
def place(s,m):
    t=gp_Trsf();t.SetValues(*m[:3].ravel());return s.Moved(TopLoc_Location(t))
src=root/'follower-01/paired-follower.step';d=json.loads((root/'follower-01/review-data.json').read_bytes())
assert hashlib.sha256(src.read_bytes()).hexdigest()==d['step']
raw,_=load_solids(src)
def name(n):
    while chr(194)+chr(183) in n:n=n.replace(chr(194)+chr(183),chr(183))
    return n
shapes={name(n):s for n,s in raw.items()};assert set(shapes)==set(d['bodies'])
case=next(c for c in d['cases'] if c['key']=='pickup-to-shot');roles=case['rig'];park=mat(125,185,0,501);ip=np.linalg.inv(park)
names=[n for n in shapes if n!='Chambered NOTE'];pairs=[(i,j) for i in range(len(names)) for j in range(i)]
def equiv(n):return 'arm' if n=='Flap axle' and roles[n]=='arm' else roles[n]
def audit():
    seen=set();collisions=[];errors=[];caseRows=[]
    q0=-49.7445989264
    poses=[(125,0,0,'archive')]+[(q,90*np.clip((q-102)/18,0,1),0,'cycle') for q in np.unique(np.r_[np.arange(-50,166,3),q0,0,102,120,125,165])]+[(q0,f,0,'admitting') for f in np.arange(-90,1,5)]+[(a,90,10.8,'climb') for a in [165,162,160,158]]
    for iq,(q,f,dep,mode) in enumerate(poses):
        x=185 if mode in ('archive','climb') else scheduled(q)
        arm=mat(q,x,0,501);fl=arm@mat(0,457.5,0,96)@mat(f,0,0,0)@mat(0,-457.5,0,-96)@ip
        M=dict(frame=np.eye(4),arm=arm@ip,flap=fl,slide=mat(0,x-185,0,0),pin=mat(0,x-185,-dep,0));mm=np.array([M['pin' if n=='Catch dog' else roles[n]] for n in names])
        S=[place(shapes[n],m) for n,m in zip(names,mm)]
        B=np.array([bounds(s) for s in S]);narrow=0
        for i,j in pairs:
            u,v=names[i],names[j]
            if any(B[i,3:]<B[j,:3]-.0001) or any(B[j,3:]<B[i,:3]-.0001):continue
            # Only repeat a close pair when its relative transform actually changes.
            key=(i,j,tuple(np.round((np.linalg.inv(mm[i])@mm[j])[:3].ravel(),7)))
            if key in seen:continue
            seen.add(key);narrow+=1
            try:
                val=common(S[i],S[j])
                if val>.001:collisions.append(dict(angle=q,flap=f,mode=mode,pair=[u,v],mm3=val))
            except Exception as e:errors.append(dict(pair=[u,v],mode=mode,error=str(e)))
        caseRows.append([q,f,dep,mode,narrow])
        if iq%10==0:(p/'progress.json').write_text(json.dumps(dict(poses=len(caseRows),narrow=len(seen),positive=len(collisions),unresolved=len(errors))))
    r=dict(parent=d['step'],subjectCount=len(names),distinctStockPairs=len(pairs),kernelComparisons=len(seen),poseRows=caseRows,positive=collisions,errors=errors,scope='Parked rigid stock, commanded pitch-slide cycle, flap admission and seated climb dog. Tread compression against the game piece is handled in the contact solver. This is sampled interference, with no between-pose guarantee for touching neighbors.')
    (p/'sweep-result.json').write_text(json.dumps(r,indent=2,default=lambda x:x.item()));print(len(seen),len(collisions),len(errors),collisions[:3])
if __name__=='__main__':audit()



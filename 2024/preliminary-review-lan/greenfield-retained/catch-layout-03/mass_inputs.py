from pathlib import Path
import json,numpy as np,math
p=Path(__file__).resolve().parent;ledger=json.loads((p/'mass-allowance.json').read_bytes())
def revised(group,original,oldcenter,olddiag):
    pts=[];weights=[]
    for r in ledger['rows'].values():
        if (r['role']=='arm')!=(group=='arm'):continue
        xyz=np.array(r['cm'])/1000
        if group=='arm':
            x,y,z=xyz-[.185,0,.501];a=math.radians(125)
            xyz=np.array([x*math.cos(a)+z*math.sin(a),y,-x*math.sin(a)+z*math.cos(a)])
        pts.append(xyz);weights.append(r['high'])
    if group=='frame':
        # Bring the original model to the existing assembly upper running bound.
        pts.append(oldcenter);weights.append(ledger['totalRunningHighKg']-ledger['shaftGroupHighKg']-ledger['fixedGroupHighKg']-68.56)
    pts=np.array([*pts,oldcenter]);m=np.array([*weights,original]);s=m.sum()
    cg=(pts*m[:,None]).sum(0)/s;delta=pts-cg
    inertia=np.diag(olddiag)
    for w,v in zip(m,delta): inertia+=w*(np.dot(v,v)*np.eye(3)-np.outer(v,v))
    return dict(mass=s,pos=cg.tolist(),fullinertia=[*np.diag(inertia),inertia[0,1],inertia[0,2],inertia[1,2]])
def bodytext(group,original,cg,diag):
    v=revised(group,original,np.array(cg),diag)
    vec=lambda q:' '.join(f'{x:.12f}' for x in q)
    return f'mass="{v["mass"]:.12f}" pos="{vec(v["pos"])}" fullinertia="{vec(v["fullinertia"])}"'


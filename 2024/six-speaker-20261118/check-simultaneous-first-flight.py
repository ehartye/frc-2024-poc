"""Cross-check the simultaneous first Speaker flight and the empty robot's return."""
import json,hashlib,math,sys,tempfile
from pathlib import Path
import numpy as np

base=Path(__file__).parent
sys.path.insert(0,str(Path(sys.argv[1])/'sim/pressure'))
from turning_sweep import NativeTurnPath
def read(f):return json.loads(f.read_bytes())
def digest(f):
    with f.open('rb') as s:return hashlib.file_digest(s,'sha256').hexdigest()
field=base/'first-field-with-return';robot=base/'wing2-certified-pickup'
fr=read(field/'result.json');rr=read(robot/'result.json')
assert fr['parameters']['trace_every_step'] and rr['traceStride']==1 and fr['sourceTrace']==rr['firstReleaseTrace']
assert digest(field/'trace.jsonl')==fr['trace'] and digest(robot/'trace.jsonl')==rr['trace']
rf=[json.loads(l) for l in (field/'trace.jsonl').open('rb')]
old=[json.loads(l) for l in (base/'field-complete-fine/trace.jsonl').open('rb')]
start=rr['start'];df=fr['parameters']['dt'];dr=rr['dt'];interval=.00005
for r in old:assert r==rf[round((r['t']-start)/df)]
body=np.array([rr['initialRobotQ'][:5]]+[json.loads(l)['q'][:5] for l in (robot/'trace.jsonl').open('rb')])
note=np.array([r['q'] for r in rf])
assert abs(fr['releaseTimeS']-start)<1e-9 and abs((len(note)-1)*df+start-fr['endS'])<1e-8
t=np.arange(round((len(note)-1)*df/interval)+1)*interval
chassis=np.column_stack([np.interp(t,np.arange(len(body))*dr,b) for b in body.T])
world=np.column_stack([np.interp(t,np.arange(len(note))*df,b) for b in note[:,:3].T])
xyz=np.stack([world[:,1]-161.625*.0254+.658,114*.0254-world[:,0],world[:,2]],axis=1)
i=np.minimum(np.floor(t/df).astype(int),len(note)-2);u=t/df-i
a=note[i,3:].copy();b=note[i+1,3:].copy()
prod=(a*b).sum(axis=1);b[prod<0]*=-1.;theta=np.arccos(np.clip(abs(prod),0.,1.))
def sinc(x):return np.sinc(x/np.pi)
interp=a*((1-u)*sinc((1-u)*theta)/sinc(theta))[:,None]+b*(u*sinc(u*theta)/sinc(theta))[:,None]
w,x,y,z=interp.T
quat=np.stack([w+z,x+y,y-x,z-w],axis=1)/math.sqrt(2.)
q=np.column_stack([chassis,xyz,quat])
with tempfile.TemporaryDirectory() as tmp:
    d=Path(tmp);(d/'result.json').write_text(json.dumps(dict(dt=interval,initialQ=q[0].tolist(),end=t[-1],outcome='HORIZON')))
    with (d/'trace.jsonl').open('w') as f:
        for j,pose in enumerate(q[1:],1):f.write(json.dumps(dict(t=j*interval,q=pose.tolist()))+'\n')
    path=NativeTurnPath(d)
inv=read(base.parent/'track-tread-candidate-20261118/route-relieved/scan.json')
assert inv['sourceSha256']==rr['nativeSha256']
groups={g:np.array([b['boundsMm'] for b in inv['inventory'] if b['motionGroup']==g]).reshape(-1,2,3) for g in path.frames}
pending=[(0,path.length)];levels=[];minimum=math.inf
for stride in (100,20,5,1):
    nxt=[];n=0
    for low,high in pending:
        for lo in range(low,high,stride):
            hi=min(lo+stride,high);p=path.interval(lo,hi);closest=math.inf
            for g,box in groups.items():
                value=p['frames'][g];c=np.array(value['center']);axis=np.array(value['normal'])
                ext=152.4*np.sqrt(np.maximum(0.,1-axis**2))+25.4+value['padMm']
                gap=np.maximum(box[:,0]-(c+ext),(c-ext)-box[:,1]).max(axis=1).min()
                closest=min(closest,float(gap))
            if closest<=0:nxt.append((lo,hi))
            else:minimum=min(minimum,closest)
            n+=1
    levels.append([stride,n,len(nxt)]);pending=nxt
    if not pending:break
out=dict(native=rr['nativeSha256'],robotTrace=rr['trace'],fieldTrace=fr['trace'],verifiedEarlierFieldRows=len(old),
    commonIntervals=path.length,levels=levels,positiveMinimumMm=minimum,unresolved=pending)
(base/'simultaneous-first-flight.json').write_text(json.dumps(out))
print('first-flight',levels,minimum,'remaining',len(pending))

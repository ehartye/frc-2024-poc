"""Bind a continued wing NOTE trace to the previously audited saved Fusion assembly."""
import hashlib,json,pathlib,sys,tempfile,subprocess
import numpy as np
out=pathlib.Path(__file__).parent
root=pathlib.Path(sys.argv[1]);case=out/sys.argv[2];tag=sys.argv[3]
sys.path.insert(0,str(root/'sim/pressure'))
from turning_sweep import NativeTurnPath
def sha(path):
    with path.open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()
load=lambda f:json.loads(f.read_bytes())
rc=load(case/'result.json')
trace=case/'trace.jsonl';pin=rc.get('trace',rc.get('traceSha256'))
assert sha(trace)==pin and rc.get('traceStride')==1
dt=rc['dt'];q=rc.get('initialQ')
if q is None:
    z=.0254-.235*9.80665/20000
    q=rc['initialRobotQ']+[*rc['waitingNoteLocalXY'],z,1.,0.,0.,0.]
with tempfile.TemporaryDirectory() as td:
    temp=pathlib.Path(td)
    with (temp/'trace.jsonl').open('w') as target, trace.open() as src:
        for j,line in enumerate(src,1):
            row=json.loads(line);t=row.get('time',row.get('t'));pos=row['q']
            assert abs(t-rc['start']-j*dt)<1e-7
            target.write(json.dumps(dict(t=j*dt,q=pos[:5]+pos[-7:]))+'\n')
    (temp/'result.json').write_text(json.dumps(dict(dt=dt,initialQ=q[:5]+q[-7:],outcome='HORIZON',end=j*dt)))
    path=NativeTurnPath(temp)
inv=load(out.parent/'track-tread-candidate-20261118/route-relieved/scan.json')['inventory']
wb=load(root/'sim/mujoco/evidence/native-launcher-binding-20261118.json')
allowed={x['key'] for x in inv if x['name'] in {w['name'] for w in wb['wheels']}} if 'feedStart' in rc else set()
template=(out/'fusion-bound-handoff-100-0.py').read_text().replace("occ[x['path']]==y['transformCm']","list(occ[x['path']])==y['transformCm']")
pending=[(0,path.length,None)];levels=[]
for stride in (120,24,4,1):
    jobs=[]
    for lo,hi,ids in pending:
        for a in range(lo,hi,stride):
            pose=path.interval(a,min(a+stride,hi));pose['parentIds']=ids;jobs.append(pose)
    pending=[];checks=0;hashes=[]
    for i in range(0,len(jobs),60):
        group=jobs[i:i+60];name=f'{tag}-{stride}-{i}';pp=out/(name+'-poses.json');rp=out/(name+'-clearance.jsonl');code=out/('fusion-'+name+'.py')
        if not pp.exists():pp.write_text(json.dumps(group))
        assert load(pp)==group
        code.write_text(template.replace('bound-handoff-100-0-',name+'-'))
        if not rp.exists():
            res=json.loads(subprocess.check_output([sys.executable,'-B',str(out.parent/'native-loading-gate-20261118/fusion_probe.py'),str(code)]))
            assert json.loads(res['result']['content'][0]['text'])['success'],res
        native=list(map(json.loads,rp.read_bytes().splitlines()));assert len(native)==len(group)
        h=sha(pp);hashes.append([h,sha(rp)])
        for pose,row in zip(group,native):
            assert all(pose[k]==row[k] for k in pose) and h==row['posePacketSha256'] and rc['nativeSha256']==row['archiveSha256']
            checks+=row['checks']
            ids=sorted({hit['id'] for hit in row['hits'] if hit['id'] not in allowed or 'unresolved' in hit})
            if ids:pending.append((pose['first'],pose['last'],ids))
    levels.append(dict(stride=stride,intervals=len(jobs),pairChecks=checks,ambiguous=len(pending),files=hashes))
    print(tag,stride,len(jobs),len(pending),checks,flush=True)
    if not pending:break
with (out/(tag+'-summary.json')).open('x') as f:json.dump(dict(source=pin,steps=path.length,levels=levels,unresolved=pending),f,indent=2)

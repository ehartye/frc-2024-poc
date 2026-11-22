import json, hashlib, pathlib, sys, tempfile, subprocess
out=pathlib.Path(__file__).parent; packets=out.parent; root=pathlib.Path(sys.argv[1]); sys.path.insert(0,str(root/'sim/pressure'))
from turning_sweep import NativeTurnPath
load=lambda f:json.loads(f.read_bytes()); h=lambda f:hashlib.sha256(f.read_bytes()).hexdigest()
case=out/'result-refined';rc=load(case/'result.json');assert h(case/'trace.jsonl')==rc['traceSha256'] and rc['traceStride']==1
with tempfile.TemporaryDirectory() as td:
 directory=pathlib.Path(td); dt=rc['dt']; q=rc['initialQ']; rows=(case/'trace.jsonl').read_bytes().splitlines()
 (directory/'result.json').write_text(json.dumps(dict(dt=dt,initialQ=q[:5]+q[-7:],outcome='HORIZON',end=len(rows)*dt)))
 with (directory/'trace.jsonl').open('w') as f:
  for j,line in enumerate(rows,1):
   r=json.loads(line);assert abs(r['time']-rc['start']-j*dt)<1e-8;q=r['q'];f.write(json.dumps(dict(t=j*dt,q=q[:5]+q[-7:]))+'\n')
 path=NativeTurnPath(directory)
inv=load(packets/'track-tread-candidate-20261118/route-relieved/scan.json')['inventory'];wb=load(root/'sim/mujoco/evidence/native-launcher-binding-20261118.json')
wheel={x['key'] for x in inv if x['name'] in {w['name'] for w in wb['wheels']}};assert len(wheel)==4
template=(out/'fusion-bound-handoff-100-0.py').read_text().replace("occ[x['path']]==y['transformCm']","list(occ[x['path']])==y['transformCm']")
pending=[(0,path.length,None)];levels=[]
for stride in (100,20,4,1):
 jobs=[]
 for lo,hi,ids in pending:
  for a in range(lo,hi,stride):
   pose=path.interval(a,min(a+stride,hi));pose['parentIds']=ids;jobs.append(pose)
 pending=[];checks=0;hashes=[]
 for i in range(0,len(jobs),180):
  group=jobs[i:i+180];name=f'checked-feed-{stride}-{i}'; pp=out/(name+'-poses.json');rp=out/(name+'-clearance.jsonl');code=out/('fusion-'+name+'.py')
  if not pp.exists():pp.write_text(json.dumps(group))
  assert load(pp)==group
  code.write_text(template.replace('bound-handoff-100-0-',name+'-'))
  if not rp.exists():
   res=json.loads(subprocess.check_output([sys.executable,'-B',str(packets/'native-loading-gate-20261118/fusion_probe.py'),str(code)]));assert json.loads(res['result']['content'][0]['text'])['success'],res
  native=list(map(json.loads,rp.read_bytes().splitlines()));assert len(native)==len(group);sh=h(pp);hashes.append([sh,h(rp)])
  for q,r in zip(group,native):
   assert all(q[k]==r[k] for k in q) and sh==r['posePacketSha256'] and rc['nativeSha256']==r['archiveSha256']
   checks+=r['checks'];ids=sorted({x['id'] for x in r['hits'] if x['id'] not in wheel or 'unresolved' in x})
   if ids:pending.append((q['first'],q['last'],ids))
 levels.append(dict(stride=stride,intervals=len(jobs),pairChecks=checks,ambiguous=len(pending),files=hashes))
 print(stride,len(jobs),len(pending),checks,flush=True)
 if not pending:break
with (out/'checked-feed-summary.json').open('x') as f:json.dump(dict(source=rc['traceSha256'],steps=len(rows),levels=levels,unresolved=pending),f,indent=2)

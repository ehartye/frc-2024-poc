from pathlib import Path
import json,sys,subprocess
d=Path(__file__).parent
tag=sys.argv[1];dst=tag+'-unpadded-check'
picked=[]
for f in d.glob(tag+'-1-*-clearance.jsonl'):
 ps=json.loads((d/f.name.replace('-clearance.jsonl','-poses.json')).read_bytes())
 for row,pose in zip(map(json.loads,f.read_bytes().splitlines()),ps,strict=True):
  score=max([item.get('volumeMm3',0) for item in row['hits']]+[0.])
  if score: picked.append((score,pose['first'],pose))
if not picked: raise ValueError('Expected padded positive volumes')
picked.sort(key=lambda x:-x[0])
examples=[picked[0],picked[len(picked)//2]]
poses=[]
for _,_,src in examples:
 pose={**src,'frames':{k:{**v,'padMm':0.} for k,v in src['frames'].items()}}
 poses.append(pose)
(d/(dst+'-poses.json')).write_text(json.dumps(poses))
template=(d/'fusion-bound-handoff-100-0.py').read_text().replace("occ[x['path']]==y['transformCm']","list(occ[x['path']])==y['transformCm']")
code=d/('fusion-'+dst+'.py')
code.write_text(template.replace('bound-handoff-100-0-',dst+'-'))
res=json.loads(subprocess.check_output([sys.executable,'-B',str(d.parent/'native-loading-gate-20261118/fusion_probe.py'),str(code)]))
assert json.loads(res['result']['content'][0]['text'])['success']
rows=list(map(json.loads,(d/(dst+'-clearance.jsonl')).read_bytes().splitlines()))
for v,r in zip(examples,rows):print(v[1],v[0],r['hits'])

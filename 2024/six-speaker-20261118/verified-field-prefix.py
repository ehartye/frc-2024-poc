"""Copy a verified single-flight prefix ending on an actual recorded integration row."""
import hashlib,json,sys
from pathlib import Path
base=Path(__file__).parent
src=base/sys.argv[1];dst=base/sys.argv[2];steps=int(sys.argv[3])
rc=json.loads((src/'result.json').read_bytes())
assert steps>0 and rc['parameters']['trace_every_step'] and rc['schema']=='speaker-field-contact/1'
with (src/'trace.jsonl').open('rb') as s:assert rc['trace']==hashlib.file_digest(s,'sha256').hexdigest()
dst.mkdir();h=hashlib.sha256();depth=0.;first={};n=0
with (src/'trace.jsonl').open('rb') as inp,(dst/'trace.jsonl').open('xb') as out:
 for i,line in enumerate(inp):
  row=json.loads(line)
  assert abs(row['t']-rc['releaseTimeS']-i*rc['parameters']['dt'])<1e-8
  out.write(line);h.update(line);n+=1
  for key in row['contacts']:first.setdefault(key,row['t'])
  depth=max(depth,row['indentationM'])
  if i==steps:break
assert n==steps+1 and row['t']<rc['endS']
rc.update(completeSourceTrace=rc['trace'],trace=h.hexdigest(),endS=row['t'],firstContactS=first,maximumIndentM=depth)
rc['parameters']['horizon']=steps*rc['parameters']['dt']
(dst/'result.json').write_text(json.dumps(rc,indent=2,allow_nan=False)+'\n')
print('Prefix rows',n,'end',rc['endS'],flush=True)

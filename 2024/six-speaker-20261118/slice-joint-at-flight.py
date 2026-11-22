"""Take a byte-exact prefix of every participant in a recorded joint solve."""
import hashlib,json,math,sys
from pathlib import Path
def sha(path):
 with path.open('rb') as src:return hashlib.file_digest(src,'sha256').hexdigest()
def read(path):return json.loads(path.read_bytes())
root=Path(__file__).parent
group=root/sys.argv[1];new=root/sys.argv[2];until=read(root/sys.argv[3]/'result.json')['endS']
parent=group/'manifest.json';m=read(parent);source=sha(parent)
assert m['schema']=='coupled-note-collection/1' and m['start']<until<m['end']
parts=[];new_hashes=[];cut=None
new.mkdir()
for index,(shot,pin,prev) in enumerate(zip(m['shots'],m['traces'],m['inputs'],strict=True)):
 src=group/str(index);rc=read(src/'result.json');trace=src/'trace.jsonl'
 assert rc['trace']==pin==sha(trace) and rc['sourceTrace']==shot and rc['previousFieldTrace']==prev and rc['coupledManifest']==source
 out=new/str(index);out.mkdir();h=hashlib.sha256()
 with trace.open('rb') as inp,(out/'trace.jsonl').open('xb') as outstream:
  for k,line in enumerate(inp):
   row=json.loads(line)
   if row['t']>=until-1e-9:break
   outstream.write(line);h.update(line);last=row['t']
  else:raise ValueError('No subsequent original sample')
 assert 0.<until-last<=rc['parameters']['dt']+1e-8 and abs(row['t']-last-rc['parameters']['dt'])<1e-8 and k>=2
 if cut is None:cut=last
 assert cut==last
 new_hashes.append(h.hexdigest());parts.append((out,rc,k))
man=dict(schema=m['schema'],shots=m['shots'],traces=new_hashes,inputs=m['inputs'],start=m['start'],end=cut,
 firstPairContactS={k:v for k,v in m['firstPairContactS'].items() if v<=cut},
 parentMaximumPairIndentationM=m['maximumPairIndentationM'],solver=m['solver'],originalManifest=source,
 originalTraces=m['traces'],rowsPerBody=k)
mp=new/'manifest.json';mp.write_text(json.dumps(man,indent=2))
for (out,rc,k),pin in zip(parts,new_hashes,strict=True):
 rc.pop('finalStepS',None)
 rc.update(trace=pin,endS=cut,coupledManifest=sha(mp),originalCoupledManifest=source,
           parameters={**rc['parameters'],'horizon':cut-m['start']})
 (out/'result.json').write_text(json.dumps(rc,indent=2))
print(k,'matching rows per body, until',cut,'next join',until,flush=True)

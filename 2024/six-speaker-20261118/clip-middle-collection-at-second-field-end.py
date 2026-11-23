import json,sys,hashlib
from pathlib import Path
p=Path(__file__).parent
source=p/'middle245-passive-collection'
target=p/'middle245-collection-to-joint-start'
other=json.loads((p/'outer-pair-aligned-field/result.json').read_bytes())
old=json.loads((source/'result.json').read_bytes())
with (source/'trace.jsonl').open('rb') as f: assert hashlib.file_digest(f,'sha256').hexdigest()==old['trace']
target.mkdir()
with (target/'trace.jsonl').open('xb') as out, (source/'trace.jsonl').open('rb') as inp:
    for line in inp:
        row=json.loads(line)
        out.write(line)
        if row['t']>=other['endS']-1e-8:break
assert abs(row['t']-other['endS'])<1e-8
with (target/'trace.jsonl').open('rb') as f: pin=hashlib.file_digest(f,'sha256').hexdigest()
old.update(endS=row['t'],clippedFromTrace=old['trace'],trace=pin)
(target/'result.json').write_text(json.dumps(old,indent=2))
print(row['t'])

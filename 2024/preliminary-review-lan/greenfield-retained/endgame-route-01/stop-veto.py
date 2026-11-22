from pathlib import Path
import shutil, json, hashlib
p=Path(__file__).resolve().parent;f=p/'field-narrow.json';b=f.read_bytes();r=json.loads(b)
assert len(r['positive'])>0 and not r['errors']
(p/'inward-field-veto.json').write_bytes(b)
(p/'inward-field-decision.json').write_text(json.dumps(dict(status='rejected',basis='Source field and robot Breps intersect on the inward swing. Terminate remaining measurements after this decisive veto.',capturedPairs=r['pairs'],positives=len(r['positive']),largest=max(r['positive'],key=lambda x:x[-1]),raw=hashlib.sha256(b).hexdigest()),indent=2))


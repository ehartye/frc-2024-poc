from pathlib import Path
import sys,json
p=Path(__file__).resolve().parent;sys.path.insert(0,str(p.parent/'assembly-recheck-01'))
from sweep import load_solids,name
from layout import viewmesh
receipt=json.loads((p/'package.json').read_bytes());new=set(receipt['added'])
bodies={name(n):viewmesh(s) for n,s in load_solids(p/'split-drive-layout.step')[0].items() if name(n) in new}
assert set(bodies)==new
(p/'meshes.json').write_text(json.dumps(bodies,separators=(',',':')))


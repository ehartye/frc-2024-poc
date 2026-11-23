from pathlib import Path
import json,sys
p=Path(__file__).resolve().parent;sys.path.insert(0,str(p.parent))
from layout import viewmesh
from showcase_contacts import load_solids
info=json.loads((p/'layout-results.json').read_bytes());ss,_=load_solids(p/'catch-install-trial.step');ss={n.replace(chr(194)+chr(183),chr(183)):v for n,v in ss.items()};assert len(ss)==146
meshes={n:viewmesh(ss[n]) for n in [*info['attr'],'Cassette cheek · left','Translating bearing block · left']}
(p/'meshes.json').write_text(json.dumps(meshes,separators=(',',':')))


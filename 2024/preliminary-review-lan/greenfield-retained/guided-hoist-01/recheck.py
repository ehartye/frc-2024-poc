"""Re-use earlier acceptance inspections on the spliced successor, assigning new receipts to that geometry."""
from pathlib import Path
import sys
p=Path(__file__).resolve().parent;job=sys.argv[1]
if job=='export':
    import json,hashlib
    src=(p/'selected.py').read_text(encoding='utf8');exec(compile(src.split('partsAll={')[0],'final stock','exec'))
    out=read(p/'selected.json');f=p/'selected-guide.step';assert digest(f)==out['whole']
    saved={name(k):s for k,s in load_solids(f)[0].items()}
    assert set(saved)==set(original)|set(parts)
    mesh={k:viewmesh(saved[k]) for k in parts}
    assert all(x['closed'] for x in mesh.values()) and all(abs(v['volume']-vol(parts[k]))<max(.02,.002*vol(parts[k])) for k,v in mesh.items())
    save(p/'selected-mesh.json',mesh);save(p/'selected-exchange.json',dict(whole=digest(f),parent=digest(source),changed=len(parts),count=len(saved),mesh=digest(p/'selected-mesh.json')))
else:
    files={'chassis':'whole-screen.py','notes':'piece-path.py','field':'field-check.py','route':'route-check.py','first':'first-leg.py','dormant':'dormant.py'}
    src=(p/files[job]).read_text(encoding='utf8')
    src=src.replace("p/'stop-slot.py'","p/'selected.py'").replace("src.split('partsAll=')[0]","src.split('partsAll={')[0]")
    src=src.replace("p/'stop-slot.step'","p/'selected-guide.step'").replace("p/'mesh.json'","p/'selected-mesh.json'").replace("p/'exchange.json'","p/'selected-exchange.json'").replace("p/'guided-cassette.step'","p/'selected-guide.step'")
    src=src.replace("s=placedAt(n,stroke,dep,release)","s=posed(n,stroke,dep,release)")
    if job=='route':
        insert="""s=s.replace("p/'mesh.json'","p/'selected-mesh.json'").replace("p/'exchange.json'","p/'selected-exchange.json'").replace("p/'route-review.json'","p/'selected-route.json'")"""
        src=src.replace("exec(compile(s,'rerun original",insert+"\nexec(compile(s,'rerun original")
    for old in ['field-check.json','whole-screen.json','dormant.json','piece-path.json']:src=src.replace("p/'"+old+"'","p/'selected-"+old+"'")
    src=src.replace("('first-'+variant","('selected-first-'+variant")
    exec(compile(src,'spliced successor '+job,'exec'))


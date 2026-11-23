"""Save the whole preliminary assembly and mesh the changed pieces from the exchanged, reopened STEP."""
from pathlib import Path
p=Path(__file__).resolve().parent
src=(p/'stop-slot.py').read_text(encoding='utf8');exec(compile(src.split('partsAll=')[0],'selected corrected stock','exec'))
G.clear();G.update(original);G.update({k:place(s,park) for k,s in parts.items()})
write_step(p/'guided-cassette.step')
saved={name(k):s for k,s in load_solids(p/'guided-cassette.step')[0].items()};assert saved.keys()==G.keys()
items={}
for k in parts:
    g=viewmesh(saved[k]);assert g['closed'],k
    assert abs(g['volume']-vol(G[k]))<max(.01,vol(G[k])*.002),k
    items[k]=g
save(p/'mesh.json',items);save(p/'exchange.json',dict(parent=digest(source),package=digest(p/'stop-slot.step'),whole=digest(p/'guided-cassette.step'),mesh=digest(p/'mesh.json'),allSolids=len(saved),changed=len(parts),role=role))


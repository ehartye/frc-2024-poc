"""Cache selected original placed CAD leaves for both sides of the chain."""
from pathlib import Path
p=Path(__file__).resolve().parent
src=(p/'field-neighbors.py').read_text(encoding='utf8')
start=src.index('field=[]')
s=src[:start]+"""outward=read(p/'outward.json')
for conf in outward['staticOverlaps'].values():
    for k in conf:
        n,path=k.split(' / ');target.setdefault(path,set()).add(n)
"""+src[start:src.index("doc=read(base.parent/")]
exec(compile(s,'bounded original field extraction','exec'))
from OCP.BRepTools import BRepTools
folder=p/'field-stock';folder.mkdir(exist_ok=True)
manifest=[]
for i,(key,sh,b) in enumerate(field):
    if np.any(b[0]>[7400,5450,2000]) or np.any(b[1]<[4050,2650,0]):continue
    f=folder/(str(i)+'.brep')
    assert BRepTools.Write_s(sh,str(f))
    manifest.append(dict(path=key,bounds=b.tolist(),file=f.name,sha=env['sha'](f)))
(p/'field-catalog.json').write_text(json.dumps(dict(field=env['expected'],items=manifest),indent=2))
print(len(manifest))


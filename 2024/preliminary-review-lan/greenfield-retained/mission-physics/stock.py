from pathlib import Path
import sys,json,shutil
root=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(root))
from layout import *
from mechanical_motion import bounds,common
original=Path(__file__).resolve().parent/'p3-data.json'
if not original.exists():shutil.copyfile(root.parent/'www/data.json',original)
# Reuse the editable selected assembly and retain the previous issue unchanged.
src=(root/'compare_view.py').read_text(encoding='utf8')
src=src[src.index('remove=[n for n'):src.index('AAB.clear()')]
intent={}
def specify(n,pur,why):intent[n]=dict(purpose=pur,rationale=why)
exec(src)
previous=set(G); additions=[]
def create(n,s,group,purpose,why,material):
    article(n,s,group,purpose,why,material);additions.append(n)
for sign,side in [(1,'left'),(-1,'right')]:
    y=sign*100.88-4
    outer=cyl((180,y,232),Y,25,8)
    rootplate=box(172,y,192,16,8,40)
    bracket=cut(cut(fuse(outer,rootplate),cyl((180,y-1,232),Y,14.5,10)),box(179,y-1,217.5,30,10,29))
    create('Chain hook · '+side,bracket,'Shared cassette lift','Take the chain load on a curved steel seat.','Two seats share the load along the crosspiece. The contact solver uses gated point constraints at these throat centers.','Steel hook plate, 8 mm estimate')
    create('Hook keeper · '+side,box(198,y,217.5,5,8,29),'Shared cassette lift','Close the hook throat once the chain is seated.','Passive keeper envelope. Gate strength, approach and spring travel need verification.','Spring-loaded steel allocation')
    name='Compliant belt tread · '+side
    yc=sign*195.8
    def profile(r):
        return fuse(fuse(box(97,yc-r,32,461,r*2,32),cyl((97,yc,32),Z,r,32)),cyl((558,yc,32),Z,r,32))
    create(name,cut(profile(24),profile(18)),'Cassette drive','Compress against the NOTE to apply traction in either direction.','Six millimeters of soft tread added to the existing belt. Mass, stiffness and installed rotating inertia require measurement.','Compliant foam or molded elastomer')
# Machine a straight outside-return relief in the six side posts, leaving the mounting ends full.
for n in list(previous):
    if n.startswith(('Lid post','Climb yoke post')):
        sign=1 if n.endswith('left') else -1
        pocket=place(box(65,217.8 if sign>0 else -220.8,31,360,3,34),mat(park,*pivot))
        G[n]=cut(G[n],pocket)
        additions.append(n)
previous-=set(additions)
out={n:dict(**viewmesh(G[n]),**meta[n],**intent[n],basis='P4 preliminary stock. Hook gates and compliant tread are idealized in the physics model.') for n in additions}
positive=[]
for i,n in enumerate(additions):
    for b in list(previous)+additions[:i]:
        a,c=bounds(G[n]),bounds(G[b])
        if all(min(a[j+3],c[j+3])-max(a[j],c[j])>1e-5 for j in range(3)):
            v=common(G[n],G[b])
            if v>.001:positive.append(dict(pair=[n,b],volume=v))
p=Path(__file__).resolve().parent
(p/'added-stock.json').write_text(json.dumps(dict(bodies=out,positive=positive),separators=(',',':')))
write_step(p/'selected-cassette.step')
print(positive)


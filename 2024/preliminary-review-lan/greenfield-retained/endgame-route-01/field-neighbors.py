"""Use the original stationary stage solids to narrow conservative swept-box candidates."""
from pathlib import Path
import json,numpy as np,sys,hashlib,math
p=Path(__file__).resolve().parent;base=p.parent
# Load the registered official STEP document and use the recorded coordinate transformation.
env={'__file__':str(base/'mission-physics/fixtures.py')}
src=(base/'mission-physics/fixtures.py').read_text(encoding='utf8')
exec(src[:src.index('packs={}')],env)
from OCP.gp import gp_Trsf
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.XCAFDoc import XCAFDoc_ShapeTool as st
from OCP.TDF import TDF_Label
from OCP.TDataStd import TDataStd_Name
from OCP.OCP.collections import Sequence_TDF_Label as Seq
from OCP.TopLoc import TopLoc_Location
sys.path.insert(0,str(base/'assembly-recheck-01'))
from sweep import load_solids,name
from layout import place,box,distance,common
def read(f):return json.loads(f.read_bytes())
occ=read(p/'hang-occupancy.json');target={}
for k,t in occ['fieldBoxes'].items():
    n,path=k.split(' / ');target.setdefault(path,set()).add(n)
field=[]
rotation=gp_Trsf();rotation.SetValues(-1,0,0,8270.875,0,0,1,4105.275,0,1,0,-4.7752)
def bb(s):
    b=Bnd_Box();BRepBndLib.AddOptimal_s(s,b,False,False);return np.array([b.CornerMin().Coord(),b.CornerMax().Coord()])
def walk(label,loc,path):
    attr=TDataStd_Name();n=attr.Get().ToExtString() if label.FindAttribute(attr.GetID_s(),attr) else ''
    path=path+[n];key='/'.join(path)
    if key in target:
        sh=BRepBuilderAPI_Transform(st.GetShape_s(label).Moved(loc),rotation,True).Shape();bounds=bb(sh)
        if n=='GE-24205_hanging' and np.ptp(bounds[:,0])<100:return
        field.append((key,sh,bounds));return
    if not any(t.startswith(key+'/') for t in target):return
    kids=Seq();st.GetComponents_s(label,kids)
    for j in range(1,kids.Length()+1):
        ch=kids.Value(j);ref=TDF_Label()
        if st.GetReferredShape_s(ch,ref):walk(ref,loc.Multiplied(st.GetLocation_s(ch)),path)
for j in range(1,env['free'].Length()+1):walk(env['free'].Value(j),TopLoc_Location(),[])
assert target.keys()=={k for k,s,b in field}
doc=read(base.parent/'www/data.json');variant=doc['concepts']['field'];bodies={**doc['bodies'],**variant['add']}
robot={name(n):s for n,s in load_solids(base/'tread-underlay-01/field-review.step')[0].items()}
c=read(p/'original-climb.json');srcTrace=read(base/'chain-contact-02/front48.json')['trace'];anchor=np.r_[c['positions'][30],0]
rot={name:np.array(series[30:]).reshape(-1,3,4) for name,series in c['series'].items()}
# Translation applied to each pose at entry. Bounds of source STEP are used for curved geometry.
def boxes(n):
    v=np.array(bodies[n]['v']);m=rot[c['rig'][n]];out=[]
    for a in m:
        vertices=v@a[:,:3].T+a[:,3]+anchor
        out.append([vertices.min(axis=0)-2,vertices.max(axis=0)+2])
    return np.array(out)
names=set.union(*target.values());paths={n:boxes(n) for n in names}
stats=dict(boxes=0,exact=0,leaves=len(field));over=[];errors=[];zero=[];pairs=0
def check(a,b,path,s,fieldbb,n):
    global pairs
    sweep=paths[n][a:b];lo=sweep[:,0].min(axis=0);hi=sweep[:,1].max(axis=0)
    if np.any(hi<fieldbb[0]) or np.any(fieldbb[1]<lo):return
    stats['boxes']+=1;solid=box(*lo,*(hi-lo));gap=distance(s,solid)
    if gap>.005:return
    if b-a>1:
        mid=(a+b)//2;check(a,mid,path,s,fieldbb,n);check(mid,b,path,s,fieldbb,n);return
    stats['exact']+=1;m=np.eye(4);m[:3]=rot[c['rig'][n]][a];m[:3,3]+=anchor
    actual=place(robot[n],m);g=distance(s,actual)
    if g<.005:
        volume=common(s,actual);row=[path,n,float(srcTrace[a]['t']),g,volume];(over if volume>.001 else zero).append(row)
for i,(path,s,b) in enumerate(field):
    for n in target[path]:
        try:check(0,len(srcTrace),path,s,b,n);pairs+=1
        except Exception as e:errors.append([path,n,str(e)])
    (p/'field-narrow.json').write_text(json.dumps(dict(layout=variant['step'],field=env['expected'],occurrencesDone=i+1,occurrences=len(field),pairs=pairs,stats=stats,positive=over,nearZero=zero,errors=errors,samples=len(srcTrace),scope='Original stationary stage solids against occupied mesh bounds expanded 2 mm, recursive pose bounds, then original robot solids where required. The moving working chain is a separate check.'),indent=2))



"""Check actual solved hoist stock versus field, peer and articulated chain paths; keep kernel contacts scoped."""
from pathlib import Path
import sys
p=Path(__file__).resolve().parent;root=p.parent
src=(p/'stop-slot.py').read_text(encoding='utf8');exec(compile(src.split('partsAll=')[0],'guided-pull package','exec'))
sys.path.insert(0,str(root/'endgame-route-01'))
from access import items,ids
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeVertex
from OCP.gp import gp_Pnt
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
old=read(root/'endgame-route-01/original-climb.json');anchor=np.r_[old['positions'][30],0.]
obs=read(root/'endgame-route-01/field-catalog.json');label=active['rig'];cad={**original,**{k:place(s,park) for k,s in parts.items()}}
# Working chain is articulated below; avoid also treating its rest shape as a rigid field rail.
keep=np.array([k!='working chain' for k in ids]);field=items[keep];fieldNames=np.array(ids)[keep]
names=[k for k in cad if k in parts or (k in label and k not in active['removed'] and k!='Chambered NOTE')]
bbs={k:np.array(bounds(s)) for k,s in cad.items() if k in names}
def orient(q):
    w,x,y,z=q;return np.array([[1-2*y*y-2*z*z,2*x*y-2*z*w,2*x*z+2*y*w],[2*x*y+2*z*w,1-2*x*x-2*z*z,2*y*z-2*x*w],[2*x*z-2*y*w,2*y*z+2*x*w,1-2*x*x-2*y*y]])
def movebb(bb,m):
    h=(bb[3:]-bb[:3])/2;c=(bb[3:]+bb[:3])/2;v=c@m[:3,:3].T+m[:3,3];l=h@abs(m[:3,:3]).T;return np.r_[v-l,v+l]
def gapSegment(s,v,w):
    e=BRepBuilderAPI_MakeEdge(gp_Pnt(*v),gp_Pnt(*w)).Edge();query=BRepExtrema_DistShapeShape(s,e);assert query.IsDone();return query.Value()
out={}
for hand in ('front','rear'):
    datum=read(p/('winch-'+hand+'.json'));fixed={};chain={};minimum=10000.;count=0;knots=0
    for r in datum['trace']:
        t=r['t'];z=r['draw']*1000;g=r['gate']*1000
        f=np.eye(4);f[:3,:3]=orient(r['q']);f[:3,3]=anchor+np.array(r['p'])*1000
        arm=f@mat(r['angle'],185,0,501);ar=arm@ip
        slide=arm@offset(-z)@ip;aspin=arm@mat(math.degrees(r['coil']/.0295),0,0,0)@ip
        # Pins and clutch transitions are conservatively scored at their full installed endpoints.
        pins=10.8;dog=8 if z>227 else 0;mx={'frame':f,'slide':f,'pin':f@offset(0,-pins),'arm':ar,'flap':ar,'hookSlide':slide,'keeper':arm@offset(-z,0,g)@ip,'drum':aspin,'clutch':arm@offset(0,-4.2)@mat(math.degrees(r['coil']/.0295),0,0,0)@ip}
        leaf={}
        for n in names:
            rr=role.get(n,'')
            if n=='Common keeper yoke and sliding rack':m=mx['keeper']
            elif rr=='carriage' or n in moving:m=slide
            elif rr=='clutch' or rr=='drum':m=mx[rr]
            elif rr=='stop':m=arm@offset(0,-dog if n.endswith('left') else dog)@ip
            elif rr=='cord' or rr=='cassette':m=ar
            else:m=mx[old['rig'].get(n,label.get(n,'frame'))]
            # A shrinking cord is enclosed by the full source line for the field bounds.
            leaf[n]=m
        boxM=np.array([movebb(bbs[n],leaf[n]) for n in names]);lo=boxM[:,:3]-.3;hi=boxM[:,3:]+.3
        visible=np.flatnonzero(np.all(hi.max(axis=0)>=field[:,0],axis=1)&np.all(lo.min(axis=0)<=field[:,1],axis=1))
        ii,jj=np.where(np.all(hi[:,None]>=field[visible,0],axis=2)&np.all(lo[:,None]<=field[visible,1],axis=2))
        for i,j in zip(ii,jj):
            k=names[i]+' / '+fieldNames[visible[j]];a=fixed.setdefault(k,[t,t,0]);a[1]=t;a[2]+=1
        if t>=datum['cutS']-.000001:minimum=min(minimum,lo[:,2].min())
        raw=np.array(r['chain'])*1000+anchor
        l=np.minimum(raw[:-1],raw[1:])-13.335;h=np.maximum(raw[:-1],raw[1:])+13.335
        ii,jj=np.where(np.all(hi[:,None]>=l,axis=2)&np.all(lo[:,None]<=h,axis=2))
        for i,j in zip(ii,jj):
            n=names[i]
            if n in moving or n.startswith(('Keeper','Chain hook','Common keeper')):continue
            m=leaf[n];seg=(raw[j:j+2]-m[:3,3])@m[:3,:3];da=gapSegment(cad[n],*seg)-13.335;count+=1
            if da<.1:chain.setdefault(n,[]).append([t,int(j),da])
        knots+=1
    out[hand]=dict(receipt=sha256(p/('winch-'+hand+'.json')) if False else digest(p/('winch-'+hand+'.json')),knots=knots,stationaryCandidates=fixed,nonHookChainClose=chain,exactChainComparisons=count,minimumDisabledLowerBoundMm=minimum)
    save(p/'field-check.json',out)
print([(k,v['knots'],len(v['stationaryCandidates']),list(v['nonHookChainClose']),v['minimumDisabledLowerBoundMm']) for k,v in out.items()])


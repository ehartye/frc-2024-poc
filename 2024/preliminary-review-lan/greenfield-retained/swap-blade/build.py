"""P6 interchangeable pickup edge, mm. Parent STEP frame for the pickup interface."""
from pathlib import Path
import json,sys,hashlib,math
here=Path(__file__).resolve().parent
geom=here.parent/'pickup-lip/geometry.py'
ctx={'__file__':str(geom)}
exec(compile(geom.read_text(encoding='utf8').split('shapes={n:place')[0],str(geom),'exec'),ctx)
from layout import *
from mechanical_motion import distance,common,bounds
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCone
from OCP.STEPControl import STEPControl_Writer,STEPControl_Reader
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.TopAbs import TopAbs_FACE
bn='Replaceable pickup lip';host='Pickup lip crossbar'
live=here.parent.parent/'www/data.json'
d=json.loads(live.read_bytes())
parenthash='a8538852a62d749c6057d5e7fb004b7bcc0a6d1db7e5575e308993ea7830d45c'
assert d['step']==parenthash
parentReceipt=d['receipt']
xf=ctx['rest']@ctx['ip']
inv=np.linalg.inv(xf)
top=ctx['top'];height=3.5*6.2/math.hypot(6.2,7)
normal=np.array([7.,0.,6.2])/math.hypot(7,6.2)
def blank(points):
    return prism([(x,-217,z) for x,z in points],(0,434,0))
def cone(pt,axis,r1,r2,h):
    return BRepPrimAPI_MakeCone(gp_Ax2(gp_Pnt(*pt),gp_Dir(*axis)),r1,r2,h).Shape()
def export(sh,path):
    w=STEPControl_Writer();w.Transfer(sh,STEPControl_AsIs);assert w.Write(str(path))==IFSelect_RetDone
def volume(sh):
    props=GProp_GProps();BRepGProp.VolumeProperties_s(sh,props);return props.Mass()
def face_area(a,b):
    s=BRepAlgoAPI_Common(a,b).Shape();props=GProp_GProps();BRepGProp.SurfaceProperties_s(s,props);return props.Mass()
def section2d(rail,tool):
    return cut(rail,tool)
def shift(vec):
    v=np.eye(4);v[:3,3]=vec;return v
def near(a,b):
    x=bounds(a);y=bounds(b)
    return all(min(x[j+3],y[j+3])-max(x[j],y[j])>=-.00001 for j in range(3))
def chk(shape,others):
    return {k:float(v) for k,b in others.items() if near(shape,b) and (v:=common(shape,b))>.0001}
# Profile A preserves the previously simulated external pickup profile.
# Profile B exposes the same rear seat, with a shorter, raised leading edge.
variant=[[593.8,24],[600,17],[607.5,8.9],[611,7.1],[613,6.8],[613.4,6.5],[613.2,6],[611.5,5.9],[606.5,6.3],[600,13.5],[593.8,20.5]]
a,b=blank(top),blank(variant)
base=place(G[host],inv)
oldBaseVol=volume(base);oldBladeVol=volume(a)
labels=[];screws={};tools={};ymarks=(-165,-55,55,165)
for i,y in enumerate(ymarks,1):
    q=np.array([596.9,y,17.]);h=q+normal*(height-.20)
    base=fuse(base,cyl(q-normal*6.6,normal,4.,6.6))
    # Hole is modeled at the nominal screw OD in the mating stock for packaging.
    base=cut(base,cyl(q-normal*6.7,normal,1.5,7.0))
    bore=cyl(q-normal*.1,normal,1.7,height+2)
    countersink=cone(h-normal*1.5,normal,1.5,3.7,2.2)
    for nm in ('a','b'):
        if nm=='a':a=cut(cut(a,bore),countersink)
        else:b=cut(cut(b,bore),countersink)
    screw=fuse(cyl(h-normal*8,normal,1.5,6.5),cone(h-normal*1.5,normal,1.5,3,1.5))
    # 2 mm across-flats hex socket, 1.05 mm deep, for a preliminary driver envelope.
    tangent=np.cross(normal,[0.,1.,0.]);r=2/math.sqrt(3)
    socket=prism([h-normal*1.05+r*(np.cos(k*np.pi/3)*tangent+np.sin(k*np.pi/3)*np.array([0.,1.,0.])) for k in range(6)],normal*1.15)
    screw=cut(screw,socket)
    name=f'Blade retaining screw {i}';labels.append(name);screws[name]=screw
    tools[name]=cyl(h+normal*.3,normal,3,75)
    G[name]=place(screw,xf)
G[host]=place(base,xf);G[bn]=place(a,xf)
assert all(BRepCheck_Analyzer(s).IsValid() for s in (a,b,base,*screws.values()))
service=next(c for c in d['cases'] if c['key']=='supported-service')
def pose(n):
    ser=service['frames'].get(n)
    return np.vstack((np.array(ser[-1]).reshape(3,4),[0,0,0,1])) if ser else np.eye(4)
serviceFromPickup=pose(bn)@xf
serviceBodies={k:place(s,pose(k if k not in labels else bn)) for k,s in G.items() if k not in service['removed']}
fixed={k:v for k,v in serviceBodies.items() if k not in labels+[bn]}
def servicecheck(sh,targets):return chk(place(sh,serviceFromPickup),targets)
def withdraw(poly):
    # The translated boundary edges sweep quadrilaterals. Together with the start
    # and end sections these cover the full straight withdrawal of the solid blank.
    vec=normal[[0,2]]*80;pts=np.array(poly,dtype=float);gauges=[blank(poly),blank(pts+vec)]
    for u,v in zip(pts,np.roll(pts,-1,axis=0)):
        edge=v-u
        if abs(edge[0]*vec[1]-edge[1]*vec[0])>.00001:gauges.append(blank([u,v,v+vec,u+vec]))
    return {n:w for gauge in gauges for n,w in servicecheck(gauge,fixed).items()}
def fitted(edge):
    return {'toRollerMm':distance(edge,place(G['Nose pickup roller'],inv)),
            'toMountMm':distance(edge,base),'overlapWithMountMm3':common(edge,base),
            'screwIntersectionsMm3':[common(edge,s) for s in screws.values()],
            'retainedRemovalNegativeMm3':[common(place(edge,shift(normal*1.5)),s) for s in screws.values()]}
data={'stationYmm':ymarks,'widthMm':434,'normalPickupFrame':normal.tolist(),'headSetdownMm':.20,
      'headToTipMm':8,'nominalInReceivingStockMm':8-height+.20,
      'faceToolLengthMm':75,'withdrawalMm':80,'serviceCase':'supported-service final pose, empty',
      'A':fitted(a),'B':fitted(b),'withdrawalHitsA':withdraw(top),'withdrawalHitsB':withdraw(variant),
      'driverHits':{k:servicecheck(g,dict(fixed,**{bn:place(a,serviceFromPickup)})) for k,g in tools.items()},
      'screwWithdrawalHits':{},'nominalAssemblyHits':{},'parent':parenthash}
for label,s in screws.items():
    # Along its axis the Ø6 head and Ø3 stem create a continuous stepped sweep.
    y=ymarks[labels.index(label)]; q=np.array([596.9,y,17.]);h=q+normal*(height-.20)
    envelope=fuse(s,cyl(h,normal,3,12));envelope=fuse(envelope,cyl(h-normal*8,normal,1.5,20))
    # larger part of cone follows original conic seat as the head is withdrawn.
    data['screwWithdrawalHits'][label]=servicecheck(envelope,dict(fixed,**{bn:place(a,serviceFromPickup)}))
for name in [bn,host,*labels]:
    data['nominalAssemblyHits'][name]=chk(G[name],{k:v for k,v in G.items() if k!=name})
print('fit and service',json.dumps(data),flush=True)
assert all(not row for row in [data['withdrawalHitsA'],data['withdrawalHitsB'],*data['driverHits'].values(),*data['screwWithdrawalHits'].values(),*data['nominalAssemblyHits'].values()])
for test in (data['A'],data['B']):
    assert test['toMountMm']<1e-5 and test['toRollerMm']>2 and min(test['retainedRemovalNegativeMm3'])>1
# Verify the issued assembly and both individual interchangeable trial articles by reopening STEP.
def count(path):
    reader=STEPControl_Reader();assert reader.ReadFile(str(path))==IFSelect_RetDone;reader.TransferRoots()
    it=TopExp_Explorer(reader.OneShape(),TopAbs_SOLID);n=0
    while it.More():n+=1;it.Next()
    return n
out=here/'shared-cassette-swappable.step';write_step(out);data['step']=hashlib.sha256(out.read_bytes()).hexdigest()
data['bodies']=count(out);assert data['bodies']==len(G)
for sh,name in [(a,'A-blunt'),(b,'B-short-raised')]:
    path=here/(name+'.step');export(sh,path);assert count(path)==1
delta=(volume(base)-oldBaseVol)*2.70e-6+(volume(a)-oldBladeVol)*.94e-6+sum(volume(s) for s in screws.values())*7.85e-6
data.update(estimatedMassChangeKg=delta,contourBMinZmm=min(pt[1] for pt in variant))
receipt=here/'interface-check.json';receipt.write_text(json.dumps(data,indent=2))
for n in (bn,host):d['bodies'][n].update(viewmesh(G[n]))
for n in labels:
    d['bodies'][n]=dict(**viewmesh(G[n]),group='Pickup lip',category='mechanism',material='Steel, preliminary M3 x 8 flat-head envelope',purpose='Retain the removable wear blade on its common seat.',rationale='The recessed head leaves the NOTE-facing surface clear. Access all four screws from the exposed face.')
    d['support'][n]=host
    for c in d['cases']:
        if bn in c.get('rig',{}):c['rig'][n]=c['rig'][bn]
        if bn in c.get('frames',{}):c['frames'][n]=c['frames'][bn]
d['bodies'][bn].update(purpose='Lift the arriving rim of the floor NOTE onto the powered center belt.',rationale='Trial A, a replaceable 434 mm UHMW blade. Four recessed screws hold it to the fixed backing. Trial B uses the same seat with a shorter, higher edge. Only A has a pickup simulation.')
d['bodies'][host].update(rationale='Keep the mounting face and four receiving bosses on the cassette when changing the wear profile.')
d['issue']='Shared cassette · swappable pickup lip'
d['focus']='The blunt blade is the starting profile. Compare pickup and carpet strikes on the same mounted nose before choosing the final contour.'
d['notes'].append('Blade trial A is installed. B is a separate interchangeable test piece. Both cleared an 80 mm withdrawal with the unit in its supported service pose. The original pickup trace represents the gross surface of A and omits its four recessed screw heads.')
d['required'][-1][1].extend(labels)
d['mass'][-1]['members'].extend(labels)
# Conservatively add the modeled increment to both ends of the existing allowance.
for k in ('inspection','running'):d[k]=[round(v+delta,4) for v in d[k]]
d['mass'][-1]['kg']=[round(v+delta,4) for v in d['mass'][-1]['kg']]
d['step']=data['step'];d['receipt']=hashlib.sha256(receipt.read_bytes()).hexdigest()
c=next(c for c in d['cases'] if c['key']=='intake')
c['basis']+=' Replay uses the previously solved gross A profile. Four small screw recesses are represented in the updated review geometry.'
d['cases'][d['cases'].index(service)]['results'].append(dict(pair=[bn,host],status='sampled',witness=dict(reason='Empty cassette at final supported service pose. Four face-access screws, 75 mm driver access and continuous 80 mm extraction checked for both profiles.')))
from default_blade import select
select(d)
view=here/'review-data.json';view.write_text(json.dumps(d,separators=(',',':')),encoding='utf8')
assert json.loads(live.read_bytes())['receipt']==parentReceipt
tmp=live.with_suffix('.next');tmp.write_bytes(view.read_bytes());tmp.replace(live)
print('issued',data['bodies'],data['step'],delta,flush=True)



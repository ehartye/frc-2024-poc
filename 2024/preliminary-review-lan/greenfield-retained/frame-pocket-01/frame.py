"""Open-sided machined pockets in four original billets. Leave an integral shear web, bolt islands and both flanges."""
from pathlib import Path
import sys,json,hashlib,numpy as np
p=Path(__file__).resolve().parent;root=p.parent;sys.path.insert(0,str(root/'assembly-recheck-01'))
from sweep import load_solids,name,roles
from layout import box,cyl,cut,fuse,G,write_step,X,Y,Z,common,bounds,distance,viewmesh
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.BRepCheck import BRepCheck_Analyzer
def V(s):
    a=GProp_GProps();BRepGProp.VolumeProperties_s(s,a);return a.Mass()
lab=root/'split-cycle-01';active=lab/'split-drive-layout.step';design={name(n):s for n,s in load_solids(active)[0].items()}
issuer=json.loads((lab/'package.json').read_bytes());assert hashlib.sha256(active.read_bytes()).hexdigest()==issuer['layout']
frame='Chassis perimeter';originals={};subjects={};cuts=[];measure={};before={}
def pocket(t,b,e):
    # Longitudinal side pocket. 4 mm radius matches an 8 mm end mill.
    r=4.;z=119.5;h=23.;x,y,depth,alongX=t
    if alongX:
        b1=box(b+r,y,z,e-b-2*r,depth,h);b2=box(b,y,z+r,e-b,depth,h-2*r)
        arcs=[cyl((s,y,k),Y,r,depth) for s in (b+r,e-r) for k in (z+r,z+h-r)]
    else:
        b1=box(x,b+r,z,depth,e-b-2*r,h);b2=box(x,b,z+r,depth,e-b,h-2*r)
        arcs=[cyl((x,s,k),X,r,depth) for s in (b+r,e-r) for k in (z+r,z+h-r)]
    body=fuse(b1,b2)
    for a in arcs:body=fuse(body,a)
    return body
def intervals(windows,maximum=92):
    rows=[]
    for a,b in windows:
        n=int(np.ceil((b-a+9)/(maximum+9)));span=(b-a-9*(n-1))/n
        for i in range(n):rows.append((a+i*(span+9),a+i*(span+9)+span))
    return rows
for n,s in list(design.items()):
    if not n.startswith(('Chassis sill','Bay crossmember')):continue
    lo=np.array(bounds(s));along=n.startswith('Chassis sill');oldV=V(s);before[n]=oldV
    if along:
        # Rear wall of the frame is X -322. The original sill ended at -320.
        s=box(-322,lo[1],116,644,24,30)
        # Open on outboard side. Closed wall stays at the crossmember interface.
        sideStart=lo[1]+3 if 'left' in n else lo[1]-.5
        tool=(0,sideStart,21.5,True)
        windows=[(-302,-165),(-102,143),(177,244),(297,303)]
    else:
        tool=(lo[0]-.5,0,17.5,False);windows=[(-220,-45),(-27,27),(45,220)]
    opens=[v for v in intervals(windows) if v[1]-v[0]>12]
    for b,e in opens:s=cut(s,pocket(tool,b,e));cuts.append([n,b,e])
    assert BRepCheck_Analyzer(s).IsValid()
    subjects[n]=s;originals[n]=design[n];design[n]=s
    neighbors=[m for m in design if (m!=n and m not in ('Chambered NOTE',) and (m==frame or 'A-frame' in m or 'crossmember' in m))]
    measure[n]=dict(volumeMm3=V(s),machinedFromMm3=oldV,remainingWallMm=3,flangeMm=3.5,cornerRadiusMm=4,interfaceGapsMm={m:distance(s,design[m]) for m in neighbors})
hits=[];tested=0
for i,(n,s) in enumerate(subjects.items()):
    a=np.array(bounds(s))
    for m,t in design.items():
        if m==n or m in list(subjects)[:i] or m=='Chambered NOTE':continue
        b=np.array(bounds(t))
        if np.all(np.minimum(a[3:],b[3:])-np.maximum(a[:3],b[:3])>1e-5):
            v=common(s,t);tested+=1
            if v>.001:hits.append([n,m,v])
G.clear();G.update(design);write_step(p/'frame-and-feeder.step')
vol=V(design[frame])+sum(V(s) for s in subjects.values())
# Mass of all five real solids at nominal 2.70 g/cc. No mass credit outside these five bodies.
solidKg=vol*2.70e-6
allowances={'eight perimeter and crossmember joint pairs incl screws and attachment plates':[.16,.35],'four A-frame foot stacks and positive shear registration':[.12,.25],'finish and machining variance':[.03,.10]}
frameRange=[solidKg*(.99 if i==0 else 1.01)+sum(a[i] for a in allowances.values()) for i in (0,1)]
base=json.loads((lab/'budget.json').read_bytes());total=[base['inspectionKg'][i]-[5,7][i]+frameRange[i] for i in (0,1)]
r=dict(parent=issuer['layout'],step=hashlib.sha256((p/'frame-and-feeder.step').read_bytes()).hexdigest(),cuts=cuts,measure=measure,minOuterWebMm=3.0,changedStockScreenCount=tested,positive=hits,ringVolumeMm3=V(design[frame]),oldFrameSolidKg=(sum(before.values())+V(design[frame]))*2.70e-6,newFrameSolidKg=solidKg,allowancesKg=allowances,newChassisKg=frameRange,combinedInspectionKg=total,upperReserveKg=56.69904625-total[1],method='Side-milled pockets leave full mounting lands and a continuous web. Mass includes the machined five members and allowances. Signed beam and frame torsion, fasteners and proof loading are separate.')
(p/'result.json').write_text(json.dumps(r,indent=2));print(r['oldFrameSolidKg'],solidKg,total,hits)
(p/'mesh.json').write_text(json.dumps({n:viewmesh(s) for n,s in subjects.items()},separators=(',',':')))


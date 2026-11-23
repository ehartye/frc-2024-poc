"""Source-dimensioned eye and straight bury, on a forward anchor that keeps the splice off both sheaves."""
from pathlib import Path
import math
p=Path(__file__).resolve().parent
src=(p/'stop-slot.py').read_text(encoding='utf8')
exec(compile(src.split('partsAll=')[0],'retained guide stock','exec'))
from OCP.BRepPrimAPI import BRepPrimAPI_MakeTorus
from OCP.gp import gp_Ax2,gp_Dir,gp_Pnt
from OCP.GC import GC_MakeArcOfCircle
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge,BRepBuilderAPI_MakeWire
from OCP.BRepOffsetAPI import BRepOffsetAPI_MakePipe
from OCP.gp import gp_Circ
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
for side,h in [(1,'left'),(-1,'right')]:
    def K(n):return n+' · '+h
    def b(x,y,z,dx,dy,dz):return box(x,y if side==1 else -y-dy,z,dx,dy,dz)
    def ax(x,y,z,r,dy):return cyl((x,y if side==1 else -y-dy,z),Y,r,dy)
    def vf(s):return place(s,offset(0,side*-32))
    for n in ('Hoist rear redirect sheave','Hoist rear redirect axle','Hoist redirect ear 194','Hoist redirect ear 211','Pivot cable drum','Hoist rope drum to sheave'):
        update(K(n),vf(parts[K(n)]))
    # The output core is positively keyed at its two new working lands.
    sh=parts['Pivot torque shaft'];update('Pivot torque shaft',cut(sh,b(6.75,160,-10,4,20,20)))
    # Both fixed redirect ears reach a new cross-tie that seats on the underside of the track and is wide in X.
    tie=b(-109,162,144,16,56,7)
    for label in ('Hoist redirect ear 194','Hoist redirect ear 211'):tie=cut(tie,parts[K(label)])
    add(K('Redirect cross-tie'),tie,'cassette',2700)
    # Removable eye pin and a thimble with a 27 mm outside bearing diameter; the loaded eye lies above its support.
    for n in ['Hoist rope anchor','Hoist anchor cross pin']:
        k=K(n);parts.pop(k);role.pop(k);rows.pop(k)
    beam=b(172,150,141,213,14,20);beam=cut(beam,b(171.9,151.6,142.6,213.2,10.8,16.8))
    # Two positive face lands at each end; representative material of the mounting shoe is included.
    bracket=b(172,150,161,16,14,19)
    add(K('Hook anchor compression tube'),beam,'carriage',2700)
    add(K('Hook anchor root'),bracket,'carriage',2700)
    shelf=b(357,150,161,34,30,2.5);shelf=cut(shelf,cyl((376,side*170,160.9),Z,4.1,3))
    pin=cyl((376,side*170,160.8),Z,4,8.2);fl=cyl((376,side*170,169),Z,6,2);pin=fuse(pin,fl)
    add(K('Thimble pin shelf'),shelf,'carriage',2700);add(K('Hoist thimble pin'),pin,'carriage',7850)
    th=ann((376,side*170,163.5),Z,13.5,4.05,4)
    add(K('Hoist eye thimble'),th,'carriage',2700)
    # Plan-view eye, 82 mm between the pin center and the throat. Curve follows a semicircle around the far face.
    y=side*170.;A=(376,y-15.,165.5);B=(391,y,165.5);C=(376,y+15,165.5)
    profile=BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(gp_Circ(gp_Ax2(gp_Pnt(*A),gp_Dir(1,0,0)),1.5)).Edge()).Wire()).Face()
    arc=GC_MakeArcOfCircle(gp_Pnt(*A),gp_Pnt(*B),gp_Pnt(*C)).Value()
    rounded=BRepOffsetAPI_MakePipe(BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(arc).Edge()).Wire(),profile).Shape()
    # The 3 mm line doubles back before the full tapered bury ends. Separate bounding cylinder reserves 6 mm over the full bury.
    eye=fuse(bar([294,y,165.5],A,1.5),rounded);eye=fuse(eye,bar(C,[294,y,165.5],1.5))
    add(K('Spliced hoist eye'),eye,'carriage',970)
    add(K('Hoist splice allowance'),bar([114,y,165.5],[294,y,165.5],3),'carriage',970)
    update(K('Hoist rope working span'),bar([-131,y,165.5],[114,y,165.5],1.5))
def posed(k,z,pin,release):
    if role.get(k)=='cord':
        y=170 if k.endswith('left') else -170
        return bar([-131,y,165.5],[114-z,y,165.5],1.5)
    return placedAt(k,z,pin,release)
partsAll={**members,**parts};checks=0;errors=[];seats={}
for z,pin,release in [(0,0,0),(0,0,4.2),(55,0,4.2),(110,0,4.2),(165,0,4.2),(220,0,4.2),(230,8,4.2),(233,8,4.2)]:
    ar={k:posed(k,z,pin,release) for k in partsAll};bb={k:np.array(bounds(s)) for k,s in ar.items()}
    for i,(a,sa) in enumerate(ar.items()):
        for j,(b,sb) in enumerate(ar.items()):
            if j<=i or not (a in parts or b in parts) or not candidate(bb[a],bb[b]):continue
            checks+=1;v=common(sa,sb)
            if v>.02:errors.append([z,pin,a,b,v])
    if not z:
        for side in ['left','right']:
            for a,bn in [('Hook anchor root','Shared lift crosspiece'),('Hook anchor compression tube','Hook anchor root'),('Redirect cross-tie','Hook guide channel')]:
                k=a+' · '+side;other=bn+' · '+side if bn!='Shared lift crosspiece' else bn
                seats[k+' / '+other]=distance(ar[k],ar[other])
G.clear();G.update(original);G.update({k:place(s,park) for k,s in parts.items()});write_step(p/'spliced-guide.step')
out=dict(whole=digest(p/'spliced-guide.step'),parent=digest(source),netStockKg=sum(v['kg'] for v in rows.values())-subtracted,rows=rows,role=role,states=8,comparisons=checks,intersections=errors,landsGapMm=seats,buryLengthMm=180,minimumStraightReserveMm=114-233+131,ropeDiameterMm=3,rollingSurfaceDiameterMm=26,eyeDmMm=27,flatEyeMm=82,source='Marlow D12 MAX 99 3 mm spliced minimum 16.2 kN. Published 60d splice allowance and minimum 2.7d eye. Finished bury diameter represented by a conservative 6 mm envelope; loaded construction and installation remain unmeasured.')
save(p/'splice.json',out);print(checks,sorted(set((a,b) for _,_,a,b,v in errors)),seats)


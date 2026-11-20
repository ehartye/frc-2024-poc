"""Check a measurable boot receiving envelope against installed neighboring parts."""
from math import cos,radians,sin,sqrt
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox,BRepPrimAPI_MakeCylinder
from OCP.gp import gp_Ax2,gp_Dir,gp_Pnt
from breaker_contacts import bounds
from breaker_layout import PLATFORMS,ROUTES,SHELF_Z,xy


def measure(rows):
    output=[]
    for idx,r in enumerate(ROUTES,1):
        x,y,angle,_=r
        direction=gp_Dir(cos(radians(angle)),sin(radians(angle)),0)
        # The post cover is bounded on a vertical stud; the tapered cable exit
        # has a separate transverse gauge and a flat plane at 27 mm from the stud.
        cap=BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x,y,258.7),gp_Dir(0,0,1)),15.,27.8).Shape()
        origin=xy(r,5.)
        neck=BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(*origin,262),direction),8.5,22.).Shape()
        components=[('upright cap',cap),('outlet',neck)]
        nearest=[]
        for name,guard in components:
            gb=bounds(guard)
            candidates=[]
            for item in rows:
                body=item['body'];bb=item['boundsMm']
                if body=='R7 149 main-breaker' or (body.startswith(f'Lead {idx} ') and any(
                    s in body for s in ('Panduit','Pacer','Southwire'))):continue
                if body.startswith(('R7 205 battery-strap-leg','R7 212 battery-strap-leg')):
                    a,b,c,A,B,C=bb;mid=-286.5 if '286.5' in body else -193.5
                    boxes=[[a,b,c,A,B,c+3],[a,b,C-3,A,B,C],[a,mid-1.5,c+3,A,mid+1.5,C-3]]
                elif body.endswith('one-piece raised clamp saddle'):
                    j=int(body.split()[1])-1
                    x0,y0,x1,y1=PLATFORMS[j]
                    corners=[xy(ROUTES[j],u,v) for u in (38.,52.) for v in (-17.,17.)]
                    xx,yy=zip(*corners)
                    boxes=[[x0,y0,SHELF_Z,x1,y1,252.],[min(xx),min(yy),252.,max(xx),max(yy),261.85]]
                else:boxes=[bb]
                for ob in boxes:
                    gap=sqrt(sum(max(ob[i]-gb[i+3],gb[i]-ob[i+3],0)**2 for i in range(3)))
                    if gap<=12:candidates.append((body,ob))
            measured=[]
            for body,b in candidates:
                solid=BRepPrimAPI_MakeBox(gp_Pnt(*b[:3]),gp_Pnt(*b[3:])).Shape()
                measured.append((BRepExtrema_DistShapeShape(guard,solid).Value(),body))
            distance,part=min(measured)
            nearest.append(dict(gauge=name,distanceToNeighborBoxMm=distance,nearest=part))
        output.append(dict(lead=idx,capGaugeDiameterMm=30,capZlimitsMm=[258.7,286.5],neckGaugeDiameterMm=17,
                           neckStartAndStopMm=[5,27],measurements=nearest))
    return output

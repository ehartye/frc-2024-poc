"""Check a nominal lug at each actual open-shell factory terminal, in mm."""
import math
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common,BRepAlgoAPI_Cut,BRepAlgoAPI_Fuse
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.TopAbs import TopAbs_FACE,TopAbs_SHELL
from OCP.gp import gp_Pnt,gp_Trsf
from breaker_contacts import area,bounds,cyl,explore,flange
from breaker_layout import ROUTES


def measure(factory):
    shells=list(explore(factory,TopAbs_SHELL))
    if len(shells)!=16:raise ValueError('Breaker shell register')
    result=[]
    for i,(x,y,angle,_) in enumerate(ROUTES):
        washer,nut,stud=(2,3,7) if i==0 else (1,4,5)
        old_tip=bounds(shells[stud])[5]
        seat=bounds(shells[washer])[2]
        step=2.032
        a=math.radians(angle)
        def registered(shape):
            m=gp_Trsf()
            m.SetValues(math.cos(a),-math.sin(a),0,x,math.sin(a),math.cos(a),0,y,0,0,1,0)
            return BRepBuilderAPI_Transform(shape,m,True).Shape()
        def tongue(z,h):
            disk=cyl(0,0,z,6.096,h)
            tab=BRepPrimAPI_MakeBox(gp_Pnt(0,-6.096,z),gp_Pnt(16,6.096,z+h)).Shape()
            return registered(BRepAlgoAPI_Cut(BRepAlgoAPI_Fuse(disk,tab).Shape(),cyl(0,0,z-.1,3.43,h+.2)).Shape())
        pad=tongue(seat,step)
        def mated(lower,upper,z):
            return sum(area(BRepAlgoAPI_Common(f,g).Shape()) for f in flange(lower,z) for g in flange(upper,z))
        landed=mated(shells[15],pad,seat)
        interior=tongue(seat+.01,step-.02)
        interior_area=sum(area(BRepAlgoAPI_Common(f,interior).Shape()) for j in (15,stud) for f in explore(shells[j],TopAbs_FACE))
        move=gp_Trsf();move.SetValues(1,0,0,0,0,1,0,0,0,0,1,step)
        factory_nut=BRepBuilderAPI_Transform(shells[nut],move,True).Shape()
        factory_washer=BRepBuilderAPI_Transform(shells[washer],move,True).Shape()
        # The spring washer's lower contact is formed by curved edges; its
        # zero planar land is recorded independently of the broad housing pad.
        r=dict(studXYmm=[x,y],factoryPadZmm=seat,lugThicknessMm=step,
               padToLugMm2=landed,washerBottomZmm=bounds(factory_washer)[2],
               distanceBeyondNutMm=old_tip-bounds(factory_nut)[5],
               housingOrStudSurfaceInsideTongueMm2=interior_area)
        if landed<60 or r['distanceBeyondNutMm']<3 or interior_area>.03:
            raise ValueError('Nominal terminal seating '+str(r))
        result.append(r)
    return result

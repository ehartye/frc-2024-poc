"""Measure contact as trimmed surface area on the factory's open-shell breaker STEP."""
import argparse
import hashlib
import json
from pathlib import Path

from OCP.Bnd import Bnd_Box
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common,BRepAlgoAPI_Cut
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepGProp import BRepGProp
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox,BRepPrimAPI_MakeCylinder
from OCP.GeomAbs import GeomAbs_Plane
from OCP.GProp import GProp_GProps
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_FACE,TopAbs_SHELL,TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.gp import gp_Ax2,gp_Dir,gp_Pnt,gp_Trsf


def explore(shape,kind):
    ex=TopExp_Explorer(shape,kind)
    while ex.More():
        yield ex.Current();ex.Next()


def bounds(s):
    b=Bnd_Box();BRepBndLib.AddOptimal_s(s,b,False,False)
    return [v for p in (b.CornerMin(),b.CornerMax()) for v in p.Coord()]


def area(s):
    g=GProp_GProps();BRepGProp.SurfaceProperties_s(s,g)
    return g.Mass()


def cyl(x,y,z,r,h):
    return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x,y,z),gp_Dir(0,0,1)),r,h).Shape()


def flange(shape,z):
    for f in explore(shape,TopAbs_FACE):
        bb=bounds(f)
        if max(abs(bb[k]-z) for k in (2,5))<.0001:
            yield f


def contact(step,native):
    root=Path(__file__).resolve().parents[2]
    ev=json.loads((root/'cad/electrical/breaker-mount-evidence.json').read_bytes())
    if hashlib.sha256(step.read_bytes()).hexdigest()!=ev['stepSha256']:raise ValueError('STEP identity')
    model=STEPControl_Reader();model.ReadFile(str(step));model.TransferRoots();source=model.OneShape()
    native=json.loads(native.read_bytes())
    obs={r["body"]:r for r in native["rows"]}
    shift=round(obs["R7 149 main-breaker"]["boundsMm"][1]+199.13)
    if shift not in (0,12):raise ValueError("Unknown placement")
    m=gp_Trsf();m.SetValues(1,0,0,-280,0,0,-1,-175+shift,0,1,0,ev['zShiftMm']+40)
    shape=BRepBuilderAPI_Transform(source,m,True).Shape()
    bb=bounds(shape)
    if max(abs(a-b) for a,b in zip(obs['R7 149 main-breaker']['boundsMm'],bb))>.02: raise ValueError('Supplier registration')
    faces=[(f,bounds(f)) for f in explore(shape,TopAbs_FACE)]
    def near(a,b):
        return all(a[i+3]+.0001>=b[i] and b[i+3]+.0001>=a[i] for i in range(3))
    def interface(s,z):
        return sum(area(BRepAlgoAPI_Common(sf,p).Shape()) for p in flange(s,z) for sf,fb in faces
                   if abs(fb[2]-z)<.0001 and abs(fb[5]-z)<.0001 and near(fb,bounds(p)))
    def crossed(s):
        sb=bounds(s)
        return sum(area(BRepAlgoAPI_Common(f,s).Shape()) for f,b in faces if near(b,sb))
    base=obs['breaker-pedestal']['boundsMm'];z=ev['baseZ']+40
    if abs(z-base[5])>.0001:raise ValueError('Pedestal datum')
    top=BRepPrimAPI_MakeBox(gp_Pnt(base[0],base[1],z-3),gp_Pnt(base[3],base[4],z)).Shape()
    for x,y in ev['worldHoleCentersXY']:top=BRepAlgoAPI_Cut(top,cyl(x,y+shift,z-4,2.75,5)).Shape()
    result=dict(sourceSha256=ev['stepSha256'],archiveSha256=native['archiveSha256'],rawBoundsMm=bounds(source),
        installedBoundsMm=bb,sourceShells=len(list(explore(source,TopAbs_SHELL))),sourceSolids=len(list(explore(source,TopAbs_SOLID))),
        sourceFaces=len(faces),pedestalLandMm2=interface(top,z),washers=[],terminalAccess=[])
    for x,y in ev['worldHoleCentersXY']:
        source_y=y
        y+=shift
        row=obs[f'JTD FW0010-375031-SS at {x}'];cz=ev['earSeatZ']+40
        if abs(cz-row['boundsMm'][2])>.0001:raise ValueError('Washer datum')
        washer=BRepAlgoAPI_Cut(cyl(x,y,cz,.375*25.4/2,.031*25.4),cyl(x,y,cz-.1,.196*25.4/2,1.1)).Shape()
        a=interface(washer,cz)
        # Omit a 0.01 mm contact layer to test unexpected supplier faces in the washer material.
        interior=BRepAlgoAPI_Cut(cyl(x,y,cz+.01,.375*25.4/2,.031*25.4-.01),
                                 cyl(x,y,cz-.1,.196*25.4/2,1.1)).Shape()
        start=cz+.031*25.4+4.826
        driver=cyl(x,y,start,3.175,48)
        def other_boxes(tool,skip):
            b=bounds(tool)
            return [n for n,v in obs.items() if n not in skip and near(b,v['boundsMm'])]
        result['washers'].append(dict(xyMm=[x,y],landMm2=a,supplierFaceAreaInsideWasherMm2=crossed(interior),
            driverGaugeMm=[6.35,48],driverSupplierCrossingMm2=crossed(driver),driverOtherBoxes=other_boxes(driver,
            {'R7 149 main-breaker',f'breaker-mount-bolt-{x}-{source_y}-head'})))
    # Opposite diagonal contains the 1/4-28 terminal studs. Start above the
    # measured factory stud tips. A bus lug and boot need their own seated sweep.
    for x,y in [(-308.575,-161.03),(-251.425,-188.97)]:
        y+=shift
        tool=cyl(x,y,ev['zShiftMm']+40+17.78+.05,6,42)
        occupied=other_boxes(tool,{'R7 149 main-breaker'})
        boot='Lead '+('1' if x<-280 else '2')+' Pacer TBCA2-R boot envelope'
        result['terminalAccess'].append(dict(xyMm=[x,y],gaugeMm=[12,42],supplierCrossingMm2=crossed(tool),
            installedOtherBoxes=occupied,removeBeforeAccess=[boot] if boot in occupied else [],
            otherBoxesAfterRemovingBoot=[name for name in occupied if name!=boot]))
    if result['pedestalLandMm2']<395 or min(r['landMm2'] for r in result['washers'])<32:
        raise ValueError(f'Incomplete actual trimmed surface support: {result}')
    if shift==12:
        from terminal_seats import measure
        result['nominalLugSeats']=measure(shape)
    return result


if __name__=='__main__':
    a=argparse.ArgumentParser(description=__doc__)
    for n in ('step','native','output'):a.add_argument(n,type=Path)
    args=a.parse_args()
    args.output.write_text(json.dumps(contact(args.step,args.native),indent=2)+'\n',encoding='utf8',newline='\n')

"""Original handed entry rails, machined carriers and nominal fastener envelopes in robot mm."""
import hashlib
import json
import math
import re
import sys
from pathlib import Path

from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Fuse
from OCP.BRepPrimAPI import BRepPrimAPI_MakeSphere
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parents[1]/"cad"))
import a2_primitives as h
D=json.loads((HERE/"part.json").read_bytes())


def fuse(a,b):
    return BRepAlgoAPI_Fuse(a,b).Shape()


def cut(a,b):
    return h.BRepAlgoAPI_Cut(a,b).Shape()


def volume(s):
    p=GProp_GProps()
    BRepGProp.VolumeProperties_s(s,p)
    return p.Mass()


def capsule(a,b,r):
    d=[v-u for u,v in zip(a,b)]
    s=h.cyl(a,d,r,math.dist(a,b))
    for p in (a,b):
        s=fuse(s,BRepPrimAPI_MakeSphere(h.gp_Pnt(*p),r).Shape())
    return s


def articles(side):
    Z=D["railZmm"]
    def stock(x,lo,z,length,width,height):
        return h.box(x,lo if side==1 else -lo-width,z,length,width,height)
    # A single machined carrier. The low forward section clears the bumper.
    carrier=stock(275,211.4,4,230,6.35,11)
    carrier=fuse(carrier,stock(275,211.4,14,36,6.35,27.275))
    carrier=fuse(carrier,stock(275,203.825,38.1,36,13.925,3.175))
    rails=[(x,side*y,Z) for x,y in D["railPathMm"]]
    core=capsule(rails[0],rails[1],2)
    core=fuse(core,capsule(rails[1],rails[2],2))
    sleeve=fuse(capsule(rails[0],rails[1],7),capsule(rails[1],rails[2],7))
    parts=[]
    for x in D["mountXmm"]:
        y=side*D["mountYmm"]
        carrier=cut(carrier,h.cyl((x,y,37),(0,0,1),1.7,6))
        screw=fuse(h.cyl((x,y,33.275),(0,0,1),1.5,8),h.cyl((x,y,41.275),(0,0,1),2.75,3))
        parts.append(("M3 cap "+str(x),screw,7.9e-3))
    for end in (rails[0],rails[-1]):
        x,y,z=end
        pin=h.cyl(end,(0,side,0),2,218-abs(y))
        # Flat for the set screw; the shoulder at each end retains the overmold.
        pin=cut(pin,stock(x-2.5,212.2,9.8,5,4.8,3))
        core=fuse(core,pin)
        carrier=cut(carrier,h.cyl((x,side*210,z),(0,side,0),2.05,10))
        carrier=cut(carrier,h.cyl((x,side*214.6,9.7),(0,0,1),1.25,7))
        parts.append(("M3 set "+str(x),h.cyl((x,side*214.6,9.8),(0,0,1),1.5,6),7.9e-3))
    sleeve=cut(sleeve,core)
    parts.extend([("carrier",carrier,2.7e-3),("welded spine",core,7.9e-3),("molded tread",sleeve,1.04e-3)])
    return parts


def build(out):
    writer=h.STEPControl_Writer()
    rows=[]
    for side in (-1,1):
        for name,shape,density in articles(side):
            assert h.BRepCheck_Analyzer(shape).IsValid(),name
            solids=h.TopExp_Explorer(shape,h.TopAbs_SOLID)
            assert solids.More(),name
            solids.Next()
            assert not solids.More(),name
            writer.Transfer(shape,h.STEPControl_AsIs)
            rows.append({"side":side,"name":name,"volumeMm3":round(volume(shape),2),
                         "estimatedG":round(volume(shape)*density,2)})
    dest=out/"paired-entry.step"
    assert writer.Write(str(dest))==h.IFSelect_RetDone
    raw=dest.read_bytes()
    raw,count=re.subn(rb"(FILE_NAME\('Open CASCADE Shape Model',')[0-9T:-]+(')",rb"\g<1>2026-11-20T00:00:00\2",raw,count=1)
    assert count==1
    dest.write_bytes(raw)
    reader=h.STEPControl_Reader()
    assert reader.ReadFile(str(dest))==h.IFSelect_RetDone
    reader.TransferRoots()
    assert reader.NbRootsForTransfer()==len(rows)==16
    for i, row in enumerate(rows,1):
        assert abs(volume(reader.Shape(i))-row["volumeMm3"])<.05,row
    (out/"part-receipt.json").write_bytes((json.dumps({"articles":rows,"totalG":round(sum(r["estimatedG"] for r in rows),2),
        "roots":16,"stepSha256":hashlib.sha256(dest.read_bytes()).hexdigest()},indent=2)+"\n").encode())


if __name__=="__main__":
    build(HERE if len(sys.argv)==1 else Path(sys.argv[1]))

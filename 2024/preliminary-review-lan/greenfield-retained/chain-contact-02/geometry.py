"""Build convex prisms from actual CAD cross-sections of the hook and guide lands."""
from pathlib import Path
import sys,numpy as np,json,hashlib,math
p=Path(__file__).resolve().parent;root=p.parent
sys.path.insert(0,str(root));from layout import *
from showcase_contacts import load_solids
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
from OCP.TopoDS import TopoDS
from OCP.TopLoc import TopLoc_Location
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
def area(v):
    return .5*abs(sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(v,v[1:]+v[:1])))
def hull(v):
    pts=sorted(set(v))
    def half(pts):
        out=[]
        for x in pts:
            while len(out)>=2:
                a,b=out[-2:]
                if (b[0]-a[0])*(x[1]-a[1])-(b[1]-a[1])*(x[0]-a[0])>1e-7:break
                out.pop()
            out.append(x)
        return out
    return half(pts)[:-1]+half(pts[::-1])[:-1]
def merge(polys):
    while True:
        changed=False
        for i,a in enumerate(polys):
            for j in range(i):
                b=polys[j]
                if len(set(a)&set(b))<2:continue
                h=hull(a+b)
                if abs(area(h)-area(a)-area(b))>1.e-6:continue
                polys[j]=h;del polys[i];changed=True;break
            if changed:break
        if not changed:return polys
src=root/'chain-entry-01/hook-and-guided-keepers.step';solids,_=load_solids(src);s=solids['left hook'];y=100.88;parts=[];Gprops=GProp_GProps();BRepGProp.VolumeProperties_s(s,Gprops)
for low,high in [(-6,-4.2),(-4.2,-4),(-4,4),(4,4.2),(4.2,6)]:
    a,b=y+low,y+high
    sl=BRepAlgoAPI_Common(s,box(120,a,185,110,b-a,100)).Shape()
    BRepMesh_IncrementalMesh(sl,.025,False,.11,True).Perform();it=TopExp_Explorer(sl,TopAbs_FACE);polys=[]
    while it.More():
        face=TopoDS.Face(it.Current());it.Next();loc=TopLoc_Location();tri=BRep_Tool.Triangulation_s(face,loc)
        if tri is None:continue
        v=np.array([tri.Node(k).Transformed(loc.Transformation()).Coord() for k in range(1,tri.NbNodes()+1)])
        if max(abs(v[:,1]-a))>1.e-5:continue
        for k in range(1,tri.NbTriangles()+1):
            pts=[tuple(np.round(v[j-1,[0,2]],6)) for j in tri.Triangle(k).Get()]
            if area(pts)>1e-5:polys.append(pts)
    for v in merge(polys):parts.append(dict(y=[low,high],polygon=v))
approx=sum((v['y'][1]-v['y'][0])*area(v['polygon']) for v in parts)
r=dict(source=hashlib.sha256(src.read_bytes()).hexdigest(),sourceVolumeMm3=Gprops.Mass(),prismVolumeMm3=approx,parts=parts,scope='Convex prisms at all five axial material bands from the installed hook sectional faces. Curves meshed at 0.025 mm chordal deflection.')
assert len(parts)>8 and abs(approx-Gprops.Mass())/Gprops.Mass()<.005,(len(parts),approx,Gprops.Mass())
(p/'collision-hook.json').write_text(json.dumps(r));print(len(parts),approx,Gprops.Mass())


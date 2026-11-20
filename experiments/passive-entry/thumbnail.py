"""Draw the as-exported positive-Y cassette by tessellating its reopened STEP solids."""
import json
import hashlib
from pathlib import Path
from parts import HERE,h,GProp_GProps,BRepGProp
from OCP.TopAbs import TopAbs_REVERSED


def build():
    receipt=json.loads((HERE/"part-receipt.json").read_bytes())
    src=HERE/"paired-entry.step"
    assert hashlib.sha256(src.read_bytes()).hexdigest()==receipt["stepSha256"]
    reader=h.STEPControl_Reader()
    assert reader.ReadFile(str(src))==h.IFSelect_RetDone
    reader.TransferRoots()
    groups=[]
    for index,row in enumerate(receipt["articles"],1):
        if row["side"]!=1:continue
        shape=reader.Shape(index)
        h.BRepMesh_IncrementalMesh(shape,.15,False,.30,True)
        explorer=h.TopExp_Explorer(shape,h.TopAbs_FACE)
        verts=[];faces=[]
        while explorer.More():
            face=h.TopoDS.Face(explorer.Current())
            loc=h.TopLoc_Location()
            mesh=h.BRep_Tool.Triangulation_s(face,loc)
            if mesh is None:
                area=GProp_GProps()
                BRepGProp.SurfaceProperties_s(face,area)
                assert area.Mass()<1e-8
                explorer.Next()
                continue
            base=len(verts)
            for i in range(1,mesh.NbNodes()+1):
                p=mesh.Node(i).Transformed(loc.Transformation())
                verts.append([round(x,2) for x in p.Coord()])
            for i in range(1,mesh.NbTriangles()+1):
                a,b,c=mesh.Triangle(i).Get()
                if face.Orientation()==TopAbs_REVERSED:b,c=c,b
                faces.extend([base+a-1,base+b-1,base+c-1])
            explorer.Next()
        groups.append({"name":row["name"],"v":verts,"f":faces})
    path=HERE.parents[1]/"docs/passive-cassette.json"
    path.write_bytes((json.dumps({"stepSha256":receipt["stepSha256"],"groups":groups},separators=(",",":"))+"\n").encode())
    # Only this individually validated solid is eligible for student display.
    carrier = [g for g in groups if g["name"] == "carrier"]
    assert len(carrier) == 1
    path.with_name("passive-carrier.json").write_bytes((json.dumps({"stepSha256":receipt["stepSha256"],"groups":carrier},separators=(",",":"))+"\n").encode())


if __name__=="__main__":build()

"""Extract the actual shaft, pilot and mounting coordinates from the purchased motor."""
from pathlib import Path
import sys,math,json,hashlib,numpy as np
p=Path(__file__).resolve().parent;root=p.parent
sys.path.insert(0,str(root/'assembly-recheck-01'));from sweep import load_solids,name,place,bounds,common,distance,mat
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
from OCP.TopoDS import TopoDS
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder,GeomAbs_Plane
source=Path('C:/Users/ehart/Documents/FRC Native/plugin-qa/toe-actuator-20261121/REV-21-1651.STEP')
bodies=list(load_solids(source)[0].values());print('count',len(bodies))
rows=[]
for body in bodies:
    it=TopExp_Explorer(body,TopAbs_FACE)
    while it.More():
        face=TopoDS.Face(it.Current());surface=BRepAdaptor_Surface(face);kind=surface.GetType()
        if kind in (GeomAbs_Cylinder,GeomAbs_Plane):
            s=surface.Cylinder() if kind==GeomAbs_Cylinder else surface.Plane()
            if abs(s.Axis().Direction().X())>.99999:
                B=bounds(face);rows.append(dict(type='cyl' if kind==GeomAbs_Cylinder else 'plane',radius=round(s.Radius(),3) if kind==GeomAbs_Cylinder else None,Y=round(s.Location().Y(),3),Z=round(s.Location().Z(),3),minX=round(B[0],3),maxX=round(B[3],3),spanY=round(B[4]-B[1],2),spanZ=round(B[5]-B[2],2)))
        it.Next()
if __name__=='__main__':
    (p/'purchased-motor.json').write_text(json.dumps(dict(source=hashlib.sha256(source.read_bytes()).hexdigest(),faces=rows),indent=2))
    for r in rows:
        if r['minX']<13 and ((r['radius'] or 0)>1 or max(r['spanY'],r['spanZ'])>24):print(r)


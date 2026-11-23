"""Extract commissioned motor barrel axes and cylindrical spans from the saved assembly."""
import sys,json,hashlib
from pathlib import Path
import numpy as np
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE,TopAbs_VERTEX
from OCP.TopoDS import TopoDS
from OCP.BRep import BRep_Tool
root=Path(__file__).parent; sys.path.insert(0,str(Path(sys.argv[1])/'scripts'))
from showcase_contacts import load_solids
from mechanical_motion import bounds
review=Path(sys.argv[2]);step=review.parents[1]/'linked-side-drive'/'first-party-lineage'/'assembly.step'
sha=hashlib.file_digest(step.open('rb'),'sha256').hexdigest()
assert sha.startswith('7a1e46c5')
ss,_=load_solids(step)
eligible=[(n,s) for n,s in ss.items() if ('REV-21-1650' in n or 'REV-21-1651' in n or 'neo' in n.lower() and 'motor' in n.lower())]
def rows(s):
    it=TopExp_Explorer(s,TopAbs_FACE);out=[]
    while it.More():
        f=(getattr(TopoDS,'Face_s',None) or TopoDS.Face)(it.Current());surf=BRepAdaptor_Surface(f)
        if surf.GetType()==GeomAbs_Cylinder:
            cy=surf.Cylinder();r=cy.Radius();axis=np.array(cy.Axis().Direction().Coord());p=np.array(cy.Location().Coord())
            if 2<r<36:
                lo=surf.FirstVParameter();hi=surf.LastVParameter()
                if hi-lo>.2:out.append(dict(r=r,p=(p+axis*lo).tolist(),axis=axis.tolist(),length=hi-lo,arc=surf.LastUParameter()-surf.FirstUParameter()))
        it.Next()
    return out
output={}
for name,s in eligible:
    spans=rows(s)
    if not spans:continue
    main=max(spans,key=lambda v:v['r']*v['arc']*v['length'])
    origin=np.array(main['p']);u=np.array(main['axis']);ret=[]
    for v in spans:
        vaxis=np.array(v['axis']);delta=np.array(v['p'])-origin
        if abs(np.dot(u,vaxis))<.99999 or np.linalg.norm(np.cross(delta,u))>.05 or v['arc']<2.9:continue
        a=delta.dot(u);b=a+np.dot(u,vaxis)*v['length'];ret.append([min(a,b),max(a,b),v['r']])
    output[name]=dict(origin=origin.tolist(),axis=u.tolist(),spans=ret,box=bounds(s))
(root/'motors.json').write_text(json.dumps(dict(step=sha,rows=output)))
print([(n,len(e['spans'])) for n,e in output.items()],flush=True)

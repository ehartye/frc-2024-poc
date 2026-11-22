from pathlib import Path
import sys,numpy as np
q=Path(__file__).parent;sys.path.insert(0,sys.argv[1])
from mechanical_motion import bounds
from showcase_contacts import load_solids
from stock_sections import entities
from OCP.TopAbs import TopAbs_VERTEX
from OCP.BRep import BRep_Tool
from OCP.TopoDS import TopoDS
for fn in ['2311-0001-0118.step','REV-41-1665.STEP']:
 a,other=load_solids(q/fn);print(fn,list(a),len(other))
 body=next(iter(a.values()));print(bounds(body));pts=np.array([BRep_Tool.Pnt_s(TopoDS.Vertex(v)).Coord()for v in entities(body,TopAbs_VERTEX)]);np.save(q/(fn+'.vertices.npy'),pts)
 for k in range(3):
  vals,c=np.unique(np.round(pts[:,k],4),return_counts=True);print(k,[(vals[j],c[j])for j in np.argsort(c)[-8:][::-1]])

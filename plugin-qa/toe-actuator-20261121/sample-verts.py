from pathlib import Path
import sys,numpy as np
q=Path(__file__).parent;sys.path.insert(0,str(Path.home()/'repos/agent-frc/scripts'))
from stock_sections import entities
from showcase_contacts import load_solids
from OCP.BRep import BRep_Tool
from OCP.TopAbs import TopAbs_VERTEX
from OCP.TopoDS import TopoDS
for nm in ['REV-21-1651','REV-41-1602','REV-41-1615','REV-41-1607','REV-41-1608','REV-41-1503','REV-41-1332']:
 path=next(q.glob(nm+'.[sS][tT][eE][pP]'));a,_=load_solids(path);p=np.unique(np.round([BRep_Tool.Pnt_s(TopoDS.Vertex(e)).Coord() for s in a.values() for e in entities(s,TopAbs_VERTEX)],6),axis=0);np.save(q/(nm+'-points.npy'),p)
 k=0 if nm=='REV-21-1651' else (1 if nm=='REV-41-1607' else 2)
 x,n=np.unique(p[:,k].round(2),return_counts=True);v={}
 for at in x[np.argsort(n)[-12:]]:
  sub=p[p[:,k].round(2)==at];v[float(at)]=np.amax(np.linalg.norm(np.delete(sub,k,axis=1),axis=1)).round(2).item()
 print(nm,sorted(v.items()),flush=True)

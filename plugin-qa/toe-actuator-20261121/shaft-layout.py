from pathlib import Path
import numpy as np,math,sys,json,runpy
from OCP.gp import gp_Ax2,gp_Trsf,gp_Pnt,gp_Dir
from OCP.TopLoc import TopLoc_Location
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
q=Path(__file__).parent;b=runpy.run_path(str(q/'bounds-fit.py'));w=b['w'];sys.path.insert(0,str(Path.home()/'repos/agent-frc/scripts'))
from mechanical_motion import distance,common
from showcase_contacts import load_solids
shaft=BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(-70,-218,195),gp_Dir(0,1,0)),6.35,436).Shape()
rack=next(iter(load_solids(q/'REV-41-1503.step')[0].values())); out={}
for rx,rz in [(-92,185),(-106,200),(-106,198),(-106,203)]:
 P=w['make'](rx=rx,rz=rz);mins={};controls=[]
 for p in P:
  lo,hi=b['localBounds'](p)
  if p.k=='rear':lo=lo-np.array([34.15,0,0]);hi=hi+np.array([53.15,0,0])
  d=math.hypot(max(lo[0]+70,-70-hi[0],0),max(lo[2]-195,195-hi[2],0))-6.35;mins[p.name]=d
  if rx==-92 and p.s=='REV-41-1503' and p.k=='rear':
   for stroke in [0,34,-53]:
    g=gp_Trsf();g.SetValues(*np.column_stack([p.A,p.t+[-stroke,0,0]]).ravel());sh=rack.Moved(TopLoc_Location(g));controls.append([p.name,stroke,common(sh,shaft)])
 out[str((rx,rz))]={'minShaftBoundsMm':sorted(mins.items(),key=lambda x:x[1])[:6],'exact':controls}
print(json.dumps(out,indent=1));(q/'wrist-shaft-check.json').write_text(json.dumps(out,indent=2))


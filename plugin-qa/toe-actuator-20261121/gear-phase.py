from pathlib import Path
import sys,numpy as np,runpy,math,json
q=Path(__file__).parent;env=runpy.run_path(str(q/'rack-fit.py'));sys.path.insert(0,str(Path.home()/'repos/agent-frc/scripts'))
from OCP.gp import gp_Trsf
from OCP.TopLoc import TopLoc_Location
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from mechanical_motion import common,distance
from showcase_contacts import load_solids
def tr(s,A,t):
 g=gp_Trsf();g.SetValues(*np.column_stack([A,t]).ravel());return s.Moved(TopLoc_Location(g))
def rotx(deg):
 a=math.radians(deg);s=np.sin(a);c=np.cos(a);return np.array([[1,0,0],[0,c,-s],[0,s,c]])
rack=next(iter(load_solids(q/'REV-41-1503.step')[0].values()))
gear=next(iter(load_solids(q/'REV-41-1332.step')[0].values()))
P=env['make'](rz=185);gearPlace=P[0];rs=[p for p in P if p.name.startswith('left seg')]
def calc(qs,phi):
 g=tr(gear,rotx(phi)@gearPlace.A,gearPlace.t)
 placed=[tr(rack,p.A,p.t+np.array([0,qs,0]))for p in rs]
 return sum(common(g,r) for r in placed)
if __name__=='__main__':
 data={}
 for ph in np.arange(0,12,1):
  v=calc(0,ph);data[float(ph)]=v;print(ph,v,flush=True)
 best=min(data,key=data.get);w={}
 for joint in [0,34,-23]:
  # Positive rack Y on upper pinion Z needs negative about +X.
  gearphi=best-math.degrees(joint/11.25)
  w[joint]=[gearphi,calc(joint,gearphi)]
  print(joint,w[joint],flush=True)
 (q/'gear-phase.json').write_text(json.dumps(dict(home=data,selectedHomePhaseDeg=best,endpoints=w),indent=2))


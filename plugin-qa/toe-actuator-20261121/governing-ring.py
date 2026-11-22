from pathlib import Path
import numpy as np,runpy,math,sys,json
q=Path(__file__).parent;w=runpy.run_path(str(q/'rack-fit.py'));sys.path.insert(0,str(Path.home()/'repos/agent-frc/scripts'))
from showcase_contacts import load_solids
from mechanical_motion import distance,common
from OCP.BRepPrimAPI import BRepPrimAPI_MakeTorus
from OCP.TopLoc import TopLoc_Location
from OCP.gp import gp_Ax2,gp_Dir,gp_Pnt,gp_Trsf
n=int(sys.argv[1]);fp=q/f'free-ring-{n}.json';read=json.loads(fp.read_bytes());P=w['make'](rz=185);rows=sorted(read['rows'].values(),key=lambda r:r['certifiedSampledLowerBoundMm'])
for row in rows[:1]:
 p=next(p for p in P if p.name==row['part']);b=next(iter(load_solids(next(q.glob(p.s+'.[sS][tT][eE][pP]')))[0].values()));g=gp_Trsf();g.SetValues(*np.column_stack([p.A,p.t+np.array(row['translatePartHead'])]).ravel());shape=b.Moved(TopLoc_Location(g))
 tor=BRepPrimAPI_MakeTorus(gp_Ax2(gp_Pnt(*row['ringCenterHead']),gp_Dir(*row['ringNormalHead'])),152.4,25.4).Shape()
 row.update(exactTorusToSolidMm=distance(tor,shape),exactCommonMm3=common(tor,shape))
read['limitingEnvelopePoseCheck']=rows[0];fp.write_text(json.dumps(read,indent=2));print(rows[0]['part'],rows[0]['ring'],rows[0]['t'],rows[0]['certifiedSampledLowerBoundMm'],rows[0]['exactTorusToSolidMm'],rows[0]['exactCommonMm3'])


from pathlib import Path
import sys,math,json,runpy
import numpy as np
q=Path(__file__).parent;w=runpy.run_path(str(q/'rack-fit.py'));b=runpy.run_path(str(q/'bounds-fit.py'));sys.path.insert(0,str(Path.home()/'repos/agent-frc/scripts'))
from OCP.gp import gp_Pnt,gp_Trsf
from OCP.TopLoc import TopLoc_Location
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from mechanical_motion import common,distance
from showcase_contacts import load_solids
def tr(body,A,t):
 g=gp_Trsf();m=np.column_stack([A,t]);g.SetValues(*m.ravel());return body.Moved(TopLoc_Location(g))
def pick(name):return next(iter(load_solids(next(q.glob(name+'.[sS][tT][eE][pP]')))[0].values()))
stock=pick('REV-41-1503');cover=BRepPrimAPI_MakeBox(gp_Pnt(419.5,-350.9,57.4),84.55,701.8,129.2).Shape()
samples={}
for n in [96,192]:
 row=json.loads((q/f'fit-untrimmed-{n}.json').read_bytes())['rows'][-1]['witness']['cover'];rows,joints=b['trace'](n);i=row['sample'];t=rows[i];wRotation=w['rot'](t['toolAngleRad']);xyz=np.array([t['toolX']*1000,0,t['toolMm']]);cases={}
 for z in [130,185]:
  part=w['make'](rz=z)[-1];body=tr(stock,wRotation@part.A,xyz+wRotation@(part.t+b['shift']('rear',[0,0,joints['rear'][i]])));cases[z]=dict(volumeMm3=common(cover,body),distanceMm=distance(cover,body))
 samples[n]=dict(t=t['t'],pose=t['toolAngleRad'],rearMm=joints['rear'][i],cases=cases)
print(samples,flush=True);(q/'tail-brep.json').write_text(json.dumps(samples,indent=2))


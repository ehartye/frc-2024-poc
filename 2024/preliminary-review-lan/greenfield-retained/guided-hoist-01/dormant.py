"""Bound nearest separation between moving bumper stock and the free outer mark over the pickup segment."""
from pathlib import Path
import sys,math,numpy as np,json,csv
p=Path(__file__).resolve().parent;root=p.parent;sys.path.insert(0,str(root/'assembly-recheck-01'))
from sweep import load_solids,place
from layout import distance
from OCP.BRepPrimAPI import BRepPrimAPI_MakeTorus
from OCP.gp import gp_Ax2,gp_Pnt,gp_Dir
plan=json.loads((root/'split-field-02/plan.json').read_bytes());leg=plan['plan'][0]
q=leg['yaw'];m=np.eye(4);m[:2,:2]=[[math.cos(q),-math.sin(q)],[math.sin(q),math.cos(q)]]
start=1000*np.array(leg['out'][-1]);end=1000*np.array(leg['loaded'][0]);outer=1000*np.array(plan['plan'][1]['mark'])
bumper=load_solids(p/'guided-cassette.step')[0]['Blue bumper set']
ring=BRepPrimAPI_MakeTorus(gp_Ax2(gp_Pnt(*outer,25.4),gp_Dir(0,0,1)),152.4,25.4).Shape()
vals=[]
for a in np.linspace(0,1,45):
    m[:2,3]=start+a*(end-start);vals.append(distance(place(bumper,m),ring))
roster=list(csv.DictReader(Path('C:/Users/ehart/repos/frc-2024-poc/sim/pressure/evidence/speaker-repeatability-starts.csv').open()));maxShift=max(math.hypot(float(r['outer_x_mm']),float(r['outer_y_mm'])) for r in roster)
result=dict(closestMeasuredMm=min(vals),interSampleMm=np.linalg.norm(end-start)/88,fullRosterOffsetMm=maxShift,clearanceLowerBoundMm=min(vals)-np.linalg.norm(end-start)/88-maxShift,scope='Exact torus-to-bumper distance, half-knot translational bound plus largest prespecified dormant-piece setup offset.')
(p/'dormant.json').write_text(json.dumps(result,indent=2));print(result)


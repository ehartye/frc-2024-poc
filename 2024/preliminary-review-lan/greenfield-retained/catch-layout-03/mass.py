from pathlib import Path
import sys,json,numpy as np
p=Path(__file__).resolve().parent;sys.path.insert(0,str(p.parent))
from layout import viewmesh
from showcase_contacts import load_solids
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
d=json.loads((p/'parent-review.json').read_bytes())
r=json.loads((p/'layout-results.json').read_bytes())
part,_=load_solids(p/'catch-install-trial.step')
part={n.replace(chr(194)+chr(183),chr(183)):s for n,s in part.items()}
rows={}
for n,a in r['attr'].items():
    prop=GProp_GProps();BRepGProp.VolumeProperties_s(part[n],prop);volume=prop.Mass()
    kg=volume*(2.70e-6 if a['material']=='Aluminum' else 7.85e-6)
    low,high=(.050,.160) if 'actuator' in n.lower() else (kg*.97,kg*1.05)
    rows[n]=dict(volumeMm3=volume,cm=list(prop.CentreOfMass().Coord()),low=low,high=high,role=a['role'])
rows['Catch sensing, wiring and connections allowance']=dict(low=.025,high=.09,role='slide',cm=[142,280,535])
low=sum(t['low'] for t in rows.values());hi=sum(t['high'] for t in rows.values())
out=dict(rows=rows,increaseKg=[low,hi],totalRunningHighKg=d['running'][1]+hi,totalInspectionHighKg=d['inspection'][1]+hi,shaftGroupHighKg=sum(t['high'] for t in rows.values() if t['role']=='arm'),fixedGroupHighKg=sum(t['high'] for t in rows.values() if t['role']!='arm'))
(p/'mass-allowance.json').write_text(json.dumps(out,indent=2));print(out['increaseKg'],out['totalInspectionHighKg'],out['totalRunningHighKg'])


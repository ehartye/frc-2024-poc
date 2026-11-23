"""Calculate design-change mass instead of counting reused motors and their wiring twice."""
from pathlib import Path
import json,sys,hashlib
p=Path(__file__).resolve().parent;sys.path.insert(0,str(p.parent/'assembly-recheck-01'))
from sweep import shapes
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
def volume(s):
    a=GProp_GProps();BRepGProp.VolumeProperties_s(s,a);return a.Mass()
d=json.loads((p/'package.json').read_bytes());old=json.loads((p.parent/'follower-01/review-data.json').read_bytes())
base=sum(d['added'].values());credit={}
for side in ('left','right'):
    for prefix,rho in [('Opposed traction belt',1.2e-6),('Compliant belt tread',.25e-6),('Nose pulley shelf',2.7e-6),('Idler axle',7.85e-6)]:
        n=prefix+' · '+side;credit[n]=volume(shapes[n])*rho
    for k in ['drive','idler']:credit['Belt pulley · '+side+' '+k]=.022
mech=base-sum(credit.values())
items={'solid package delta':[.9*mech,1.18*mech],'incremental fasteners, guards and strain relief':[.035,.095],'additional controller, conductors and connector':[.052,.13]}
delta=[sum(a[i] for a in items.values()) for i in (0,1)]
inspection=[old['inspection'][i]+delta[i] for i in (0,1)]
# Existing drive and shot use actual simulated weak-pack power; added current and weight require a new route check.
r=dict(geometry=d['layout'],baseInspection=old['inspection'],retained='Both original launcher motors, drive shafts and shelves relocated with their previous allocation.',replacementEstimatedKg=base,creditKg=credit,deltaKg=delta,estimates=items,inspectionKg=inspection,headroomKg=56.699-inspection[1],runningKg=[old['running'][i]+delta[i] for i in (0,1)],scope='Planning estimates. New shafts, controller, mounts and transmission require selected articles and tolerances.')
(p/'budget.json').write_text(json.dumps(r,indent=2));print(delta,inspection,r['headroomKg'])


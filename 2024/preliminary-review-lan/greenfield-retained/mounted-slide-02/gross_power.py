"""Charge the entire service package again to the chassis in the loaded sizing case."""
from pathlib import Path
import json,sys
here=Path(__file__).resolve().parent
root=here.parent
sys.path[:0]=[str(root/'loaded-actuation-01'),str(root/'follower-01')]
from power import *
here=Path(__file__).resolve().parent
src=root/'loaded-actuation-01/purchased_power.py'
code=src.read_text().split('cases=[motor_run(',1)[0].replace("(here/'power.py')","(root/'loaded-actuation-01/power.py')")
exec(compile(code,'matched motor curves','exec'))
mass=read(here/'service-view-receipt.json')['grossKg'][1]
original=driveMass;driveMass+=mass
# Conservatively charge the new mass at the farthest bumper corner for yaw inertia.
body=body.replace('(10./4)*alpha','((10.+mass*.7**2)/4)*alpha')
assert 'mass*.7**2' in body
exec(compile(body,'charged chassis inertia','exec'))
rr=[motor_run(leg,.16,aux,2.,4.8) for aux in [25,39] for leg in plan['plan']]
def stats(aux):
    sel=[v for v in rr if v['auxA']==aux]
    return dict(bus=min(x['weakBusMinV'] for x in sel),motorReserve=min(x['minimumMotorVoltageReserveV'] for x in sel))
out=dict(extraFrameKg=mass,runningKg=driveMass,inspectionHighKg=g['inspection'][1]+mass,inspectionMarginKg=56.699-g['inspection'][1]-mass,upperFrameInertiaAddition=mass*.7**2,loaded25A=stats(25),negative39A=stats(39),cases=rr,scope='New service package stays on chassis. Charge its full gross mass on top of original high running mass and yaw inertia, retaining previous motor mass and 2 kg adverse arm allocation from existing contingency.')
(here/'gross-power.json').write_text(json.dumps(out,indent=2))
print(out['loaded25A'],out['negative39A'],driveMass)


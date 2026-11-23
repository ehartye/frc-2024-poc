"""Stationary unloaded reverse stroke after field release, preserving signed torque and weak-pack loads."""
from pathlib import Path
import sys,json,hashlib
p=Path(__file__).resolve().parent
root=p.parent
sys.path.insert(0,str(root/'loaded-actuation-01'))
import numpy as np
from power import *
here=root/'bounded-lift-01'
# Reuse the same dynamics and separate reserve hypothesis. Supply allowance covers the measured firing tail.
code=(root/'loaded-actuation-01/power.py').read_text().split('def run(',1)[1].split("if __name__==",1)[0]
code='def reverse('+code
needle='profileClock=clock*(3.4/liftS)'
assert code.count(needle)==1;code=code.replace(needle,'profileClock=3.4-clock*(3.4/liftS)')
code=code.replace("if m<=0:continue","if m<=0 or name=='Loaded NOTE':continue")
exec(compile(code,'unloaded return with same drive limits','exec'))
out=[]
for aux in [30.,39.,92.]:
    r=reverse(dict(loadedT=[4.8],loaded=[[1.39,5.871],[1.39,5.871]],id='parked-reset',yaw=0),.16,aux,2.,4.8)
    out.append(r)
def h(f):return hashlib.sha256(f.read_bytes()).hexdigest()
report=dict(source=h(root/'loaded-actuation-01/power.py'),mass=h(root/'loaded-actuation-01/power.json'),geometry=g['step'],cases=out,scope='Robot stationary during return, empty cassette, fixed 30 A battery-side residual allowance. Measured firing traces must support that allowance. Applied friction bounds treat both motion signs conservatively. Candidate symmetric 35 A pivot mode for pickup/reset only; climb and its 25 A negative cap remain separately specified.')
(here/'reset-load.json').write_text(json.dumps(report,indent=2))
for r in out:print(r['auxA'],r['minPitchCurrentA'],r['maxPitchCurrentA'],r['maxSlidePhaseA'],r['weakBusMinV'],r['minimumMotorVoltageReserveV'])


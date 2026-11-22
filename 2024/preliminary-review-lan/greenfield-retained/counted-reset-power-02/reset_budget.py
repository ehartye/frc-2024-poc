"""Bound the stationary empty-cassette reset using the matched motor and simultaneous feeder braking."""
from pathlib import Path
import sys,json,hashlib
here=Path(__file__).resolve().parent;root=here.parent
sys.path[:0]=[str(root/'loaded-actuation-01'),str(root/'follower-01')]
from power import *
here=Path(__file__).resolve().parent
fn=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
body=(root/'loaded-actuation-01/purchased_power.py').read_text(encoding='utf8').split("cases=[motor_run(",1)[0]
body=body.replace("exec(compile(body,'matched NEO 550 terminal curve','exec'))","")
body=body.replace("(here/'power.py')","(root/'loaded-actuation-01/power.py')");exec(compile(body,'purchase-bound demand construction','exec'))
assert body.count('profileClock=clock*(3.4/liftS)')==1
body=body.replace('profileClock=clock*(3.4/liftS)','profileClock=3.4-clock*(3.4/liftS)').replace('if m<=0:continue',"if m<=0 or name=='Loaded NOTE':continue")
body=body.replace("return dict(piece=leg['id']", "return dict(traceTimes=t.tolist(),watts=watts.tolist(),requiredV=np.maximum.reduce((pv,sv,motorPeak)).tolist(),piece=leg['id']")
exec(compile(body,'stationary empty reverse with purchase-bound curves','exec'))
leg=dict(id='stationary',loadedT=[4.8],loaded=[[1.39,5.871],[1.39,5.871]],yaw=0)
res={}
for aux in [30.,35.,39.,120.]:
    q=motor_run(leg,.16,aux,2.,4.8)
    if aux==35.:
        arrays={k:q[k] for k in ['traceTimes','watts','requiredV']}
        np.savez(here/'reset-waveform.npz',**{k:np.asarray(v) for k,v in arrays.items()})
    res[int(aux)]={k:v for k,v in q.items() if k not in ['traceTimes','watts','requiredV']}
q=res[35];assert q['weakBusMinV']>=8 and q['minimumMotorVoltageReserveV']>0
raw=(root/'bounded-risk-01/power.py').read_text(encoding='utf8');split=(root/'scorer-contact-01/split.py').read_text(encoding='utf8')
assert 'active and release is None else 0.;Fmotor=' in raw and 'requestAccel=bool(t<.4 or (.55<t<.70)' in raw
assert 'feedI=np.clip((command-feedSpeed)*18,-8,8)' in split and 'currents.append(abs(feedI)*min(1.' in split
r=dict(step=g['step'],curve=sha(root/'loaded-actuation-01/purchased_power.py'),motionSource=sha(root/'loaded-actuation-01/power.py'),firingArbiter=fn(root/'bounded-risk-01/power.py'),feedLoad=fn(root/'scorer-contact-01/split.py'),standby=27,feedCurrentCap=8,upperBatteryA=35,maximumScrewRPM=read(root/'loaded-actuation-01/slide-demand.json')['rows'][-1]['maxScrewRPM8mmLead']*3.4/4.8,cases=res,waveform=fn(here/'reset-waveform.npz'),scope='After the scored NOTE leaves the robot and the 0.7 s acceleration window ends, both launcher motors coast. Total non-reset load is bounded by 27 A auxiliaries and 8 A feeder command. Include the complete bound concurrently through the 4.8 s reset, while stationary. No regenerative credit.')
(here/'reset-budget.json').write_text(json.dumps(r,indent=2))
print([(a,v['weakBusMinV'],v['minimumMotorVoltageReserveV']) for a,v in res.items()])



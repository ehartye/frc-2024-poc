"""Replace the provisional slide motor curve with the original purchased motor's published 12 V endpoints."""
from power import *
from pathlib import Path
here=Path(__file__).resolve().parent
ds=Path('C:/Users/ehart/Documents/FRC Native/plugin-qa/toe-actuator-20261121/REV-21-1651-DS.pdf')
body=(here/'power.py').read_text().split('def run(',1)[1].split("if __name__==",1)[0]
body='def motor_run('+body
for a,b in [(')/.0097',')/.0097+1.4'),("math.tau/lead*.010+slideI*.13","math.tau/lead*((12.-.12*1.4)/(11000*math.tau/60))+slideI*.12")]:
    assert body.count(a)==1;body=body.replace(a,b)
needle="peakWatts=float(max(watts)),"
assert body.count(needle)==1
body=body.replace(needle,needle+"batteryBreakdownAtPeakA=dict(accessories=aux,fourDrives=float(power[idx]/bus[idx]) if bus[idx]>0 else None,pivot=float((pitchI*pv)[idx]/(.95*bus[idx])) if bus[idx]>0 else None,slide=float((slideI*sv)[idx]/(.95*bus[idx])) if bus[idx]>0 else None),")
exec(compile(body,'matched NEO 550 terminal curve','exec'))
cases=[motor_run(leg,.16,aux,2,4.8) for aux in [25,39,92] for leg in plan['plan']]
out=dict(motor='REV-21-1651 NEO 550',supplierDataSheetSha256=sha(ds),purchasedSTEP=read(here/'purchased-motor.json')['source'],geometry=g['step'],massLedger=sha(here/'power.json'),motorCurve='12 V endpoints, stall 100 A and 0.97 N m, 11000 rpm at 1.4 A; budget a further 1.4 A as constant running drag on the ideal torque calculation. Temperature and long-duration thermal rating unmeasured.',leadMmPerRev=8,efficiency=.65,cases=cases)
(here/'purchased-power.json').write_text(json.dumps(out,indent=2))
for aux in [25,39,92]:
    q=[r for r in cases if r['auxA']==aux];print(aux,min(r['weakBusMinV'] or 0 for r in q),min(r['minimumMotorVoltageReserveV'] if r['minimumMotorVoltageReserveV'] is not None else -99 for r in q),max(r['maxSlidePhaseA'] for r in q))


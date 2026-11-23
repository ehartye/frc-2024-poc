from power import *
cases=[]
for seconds in (3.4,4.,4.4,4.8,5.2):
    group=[run(leg,.16,25,2,seconds) for leg in plan['plan']]
    cases.extend(group)
    print(seconds,*(round(min(r[k] for r in group),3) for k in ['weakBusMinV','minPivotTerminalReserveV','minSlideTerminalReserveV']),round(max(r['maxPitchPhaseA'] for r in group),2),round(max(r['maxSlidePhaseA'] for r in group),2))
(p/'retime.json').write_text(json.dumps(dict(powerMethod=sha(p/'power.py'),sourceReceipt=sha(p/'power.json'),cases=cases),indent=2))


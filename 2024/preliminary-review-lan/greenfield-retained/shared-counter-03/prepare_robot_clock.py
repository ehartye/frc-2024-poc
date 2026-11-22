"""Prepare a separate on-robot schedule which does not need access to the field score display."""
from make_generator import read,sha,p,old
from shutil import copyfile
import json
out=p.parent/'robot-clock-01';out.mkdir(exist_ok=True)
for name in ['carry.py','counter_tail.py','shot.py','lane.py','empty_return.py','reset-budget.json','reset-waveform.npz','clearance.json','plan.json']:
    dst=out/name
    if dst.exists():assert sha(dst)==sha(old/name)
    else:copyfile(old/name,dst)
s=(old/'mission.py').read_text().split("if __name__=='__main__':")[0]
def sub(a,b):
    global s
    assert s.count(a)==1,a
    s=s.replace(a,b)
sub('depart=0 if i==0 else max(end,reset) if parallel else end+empty','depart=0 if i==0 else reset')
sub("s['opening']['fullyThroughByS']<=trigger","s['opening'] is not None and s['opening']['fullyThroughByS']<=trigger")
sub("parallel=bool(s['passShot'] and trigger","parallel=bool(trigger")
sub("good=good and s['passShot'] and s['minBusV']>=8 and end<=120","good=good and parallel and s['minBusV']>=8 and shotStart+trigger<=120")
sub("passMission=bool","passSchedule=bool")
sub("elapsedS=end,serialElapsedS","elapsedS=end,lastOnboardReleaseCheckS=(shotStart+trigger if 'shot' in part else None),serialElapsedS")
sub("freeze=sha(p/'freeze.json')","freeze=sha(p/'protocol.json')")
s += """
if __name__=='__main__':
    name=sys.argv[1]
    row=next(v for v in csv.DictReader(roster.open()) if v['attempt']==name)
    trial(row,96)
"""
fn=out/'mission.py'
if fn.exists():assert fn.read_text()==s
else:fn.write_text(s)
protocol=dict(parent=sha(old/'freeze.json'),source={n:sha(out/n) for n in ['mission.py','carry.py','empty_return.py','shot.py','lane.py','counter_tail.py','plan.json','reset-budget.json','reset-waveform.npz']},actions='Depart after the on-robot feed timer (1.0 s) and completed 4.8 s reset. Score display cannot hold robot. Reproduce carry/power/separation evidence on every leg; on an adverse safety screen terminate qualification. Report six verified shared-counter events separately at the 120 s deadline.',roster=read(old/'freeze.json')['roster'])
pf=out/'protocol.json'
if pf.exists():assert read(pf)==protocol
else:pf.write_text(json.dumps(protocol,indent=2))
# Copy only the first inbound and shot from the same original run. Confirm all data lineage on load.
k='R06-middle-96'
src=read(old/'run-R06-96.json')['parts'][0]
for name,pin in [(f'{k}.json',src['carry']),(f'split-tail_.25_0-{k}-6.88.json',src['shot'])]:
    assert sha(old/name)==pin
    if (out/name).exists():assert sha(out/name)==pin
    else:copyfile(old/name,out/name)
print(out,sha(pf))


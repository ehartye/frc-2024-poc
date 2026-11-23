"""Freeze one new shooter setting after matched trials, then run the full carried-state sequence."""
from pathlib import Path
from shutil import copyfile
from make_generator import old,p,sha,read
import json
parent=old.parent/'robot-clock-01'
out=old.parent/'shooter-command-696';out.mkdir(exist_ok=True)
refs=['R02-inner','R01-inner','R03-center-low']
screen=[]
for tag in refs:
    pair=[read(p/f'speed-{tag}-{s}.json') for s in ('6.80','6.96')]
    assert pair[0]['source']==pair[1]['source'] and all(x['opening'] and x['both'] and x['minimumBusV']>8 for x in pair)
    assert pair[1]['both']<pair[0]['both']
    screen.extend([dict(tag=tag,speed=x['speed'],both=x['both'],minOpeningMm=x['opening']['minimumMarginM']*1000,minBusV=x['minimumBusV']) for x in pair])
for n in ['carry.py','lane.py','counter_tail.py','empty_return.py','plan.json','clearance.json','reset-budget.json','reset-waveform.npz']:
    dst=out/n
    if dst.exists():assert sha(dst)==sha(parent/n)
    else:copyfile(parent/n,dst)
def write_once(f,s):
    if f.exists():assert f.read_text()==s
    else:f.write_text(s)
s=(parent/'shot.py').read_text()
assert s.count("'6.88'")==1
write_once(out/'shot.py',s.replace("'6.88'","'6.96'"))
s=(parent/'mission.py').read_text()
assert s.count('-6.88.json')==1
s=s.replace('-6.88.json','-6.96.json')
s=s.replace("row=next(v for v in csv.DictReader(roster.open()) if v['attempt']==name)","row={'attempt':'dev'} if name=='dev' else next(v for v in csv.DictReader(roster.open()) if v['attempt']==name)")
s=s.replace('trial(row,96)','trial(row,int(sys.argv[2]))')
write_once(out/'mission.py',s)
src=['carry.py','lane.py','counter_tail.py','empty_return.py','plan.json','clearance.json','reset-budget.json','reset-waveform.npz','shot.py','mission.py']
plan=dict(parent=sha(parent/'protocol.json'),source={n:sha(out/n) for n in src},speedMps=6.96,
    basis='Both prospective commands recovered the rebounding R02 inner and retained the passing and late-count control. The faster command reached both beams earlier in all three trials; both had at least 19 mm opening margin and 8.34 V loaded. Test 6.96 unchanged on two nominal full routines and fresh roster starts. Never count development runs toward 19/20.',
    cases=screen,scope='Same physical mechanism, original field, returning belt state, on-robot reset. Individual passage followed by shared-field replay. Original failed shot returned into fixed electronics. Any candidate with a similar crossing is held regardless of apparent field score.', gate='original 19/20 and 120 s; compare before/after; if either nominal fails review cause before full campaign.')
write_once(out/'protocol.json',json.dumps(plan,indent=2))
print(sha(out/'protocol.json'),flush=True)


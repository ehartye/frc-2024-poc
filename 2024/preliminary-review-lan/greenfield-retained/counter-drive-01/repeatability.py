"""Reproducible six-piece development and frozen-roster campaign, with a receipt after every leg."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import csv, hashlib, json, math, subprocess, sys
from paired_cycle import read,sha,save
p=Path(__file__).resolve().parent
base=p.parent/'split-field-02';plan=read(base/'plan.json')
roster=Path('C:/Users/ehart/repos/frc-2024-poc/sim/pressure/evidence/speaker-repeatability-starts.csv')
assert sha(roster)==plan['roster']
def call(script,args,log):
    with log.open('w') as f: subprocess.run([sys.executable,script,*args],cwd=p,check=True,stdout=f,stderr=f)
def trial(case):
    row,n=case;name=row['attempt'];out=p/f'mission-receipt-{name}-{n}.json';parts=[];clock=0
    for i,leg in enumerate(plan['plan']):
        k=leg['id'];tag=f'mission-working-{name}-{k}';a=p/f'{tag}-{n}.json'
        if not a.exists():call('mission_carry.py',[name,k,str(n),'working'],a.with_suffix('.log'))
        c=read(a)
        assert c['id']==k and c['attempt']==name and c['n']==n
        travel=sum(leg['outT'])
        if i==0 and name!='dev':
            start,dock=leg['out'];L=math.hypot(dock[0]-start[0]-float(row['robot_x_mm'])/1000,dock[1]-start[1]-float(row['robot_y_mm'])/1000)
            travel=max(1.5*L/2.7,math.sqrt(6*L/2.6),math.sqrt(6*abs(math.radians(float(row['robot_heading_deg'])))/2.8))
        clock+=travel+(1.15 if i==0 else 3.4)+c['finish']
        # Confirm retention from the first authorized departure; extra pickup dwell precedes transport.
        points=[r for r in c['trajectory'] if r[0]>=2.82+c['seatDelay']]
        held=bool(points) and all(.12<r[8]<.35 and .018<r[9]<.09 and abs(r[2])<.05 for r in points)
        captured=c['seated'] and c['beamProofS'] is not None and not c['failedGate'] and held
        part=dict(id=k,carry=sha(a),gateS=c['beamProofS'],seated=c['seated'],held=held,passShot=False,endCarryClock=clock)
        if captured:
            b=p/f'split-paired_.75_.75-{tag}-{n}-6.88.json'
            if not b.exists():call('fired.py',[tag,str(n),'6.88','.75','.75'],b.with_suffix('.log'))
            s=read(b);assert s['sourceCarry']==sha(a) and s['targetSpeed']==6.88
            clock+=s['both'] if s['passShot'] else s['trace'][-1]['t']
            part.update(shot=sha(b),passShot=s['passShot'],both=s['both'],opening=s['opening'],bus=s['minBusV'])
        parts.append(dict(**part,clock=clock));score=sum(x['passShot'] for x in parts)
        ok=captured and part['passShot'] and part['bus']>=8 and clock<=120
        result=dict(attempt=name,n=n,parts=parts,score=score,passMission=bool(len(parts)==6 and ok),terminal=bool(len(parts)==6 or not ok),freeze=sha(p/'mission-freeze.json'))
        save(out,result)
        if not ok:break
    print(name,n,score,round(clock,3),flush=True);return result
if __name__=='__main__':
    cfg=dict(roster=sha(roster),baseline=sha(base/'freeze.json'),code={k:sha(p/k) for k in ['counter.py','fired.py','capture.py','mission_carry.py','repeatability.py']},originalBeam=sha(p.parent/'seat-interlock-01/recovery.py'),field=sha(p/'transmission.json'),pads=76,segmentMass='hold full moving-pad mass constant',robotMps=6.88,fieldMotorFractions=[.75,.75],required='19 of 20 six-piece starts by 120 s at >=8 V during shots; independently close remaining robot power, service and field checks',timing='retain original conservative wait for both field sensors before departure',scope='Fixed field-speed assumption. Correction informed by nominal, R01 and R08. Robot mount motion is prescribed and stationary drive model initializes on each approach.')
    f=p/'mission-freeze.json'
    if f.exists():assert read(f)==cfg
    else:save(f,cfg)
    rows=list(csv.DictReader(roster.open()))
    with ThreadPoolExecutor(max_workers=4) as pool:
        commission=list(pool.map(trial,[({'attempt':'dev'},48),({'attempt':'dev'},96),(rows[0],96),(rows[7],96)]))
        out=[r for r in commission if r['attempt']!='dev']
        save(p/'mission-status.json',dict(commission=[(r['attempt'],r['n'],r['passMission']) for r in commission],roster=out,unrun=20-len(out)))
        if all(r['passMission'] for r in commission[:2]):
            remain=[r for r in rows if r['attempt'] not in {x['attempt'] for x in out}]
            while remain and sum(not x['passMission'] for x in out)<2:
                group=remain[:3];remain=remain[3:]
                out.extend(pool.map(trial,[(r,96) for r in group]))
                save(p/'mission-status.json',dict(commission=[(r['attempt'],r['n'],r['passMission']) for r in commission],roster=out,unrun=20-len(out)))


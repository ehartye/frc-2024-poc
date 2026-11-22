"""Two nominal discretizations followed by the 20 predeclared six-piece setup variations."""
from pathlib import Path
import hashlib,json,sys,subprocess,math,csv
from concurrent.futures import ThreadPoolExecutor
p=Path(__file__).resolve().parent;root=p.parent
def read(f):return json.loads(f.read_bytes())
def sha(f):return hashlib.sha256(f.read_bytes()).hexdigest()
def write(file,data):
    tmp=file.with_suffix('.tmp');tmp.write_text(json.dumps(data,indent=2));tmp.replace(file)
plan=read(p/'plan.json')
roster=Path('C:/Users/ehart/repos/frc-2024-poc/sim/pressure/evidence/speaker-repeatability-starts.csv')
assert sha(roster)==plan['roster']
sources=[p/f for f in ('run.py','carry.py','fire.py','route.py','plan.json')]+[root/'split-cycle-01'/f for f in ('split_intake.py','split_fire.py','package.json')]+[root/'scorer-contact-01'/f for f in ('split.py','patch.py')]+[root/'six-piece-02'/f for f in ('route.py','loaded.py')]+[root/'follower-01'/f for f in ('carry_cam.py','fire.py','stroke.py','response.json')]+[root/'bounded-risk-01'/f for f in ('pickup.py','power.py')]+[root/'pickup-lip'/f for f in ('pickup_trial.py','shot_replay.py','layout.json')]+[root/'pickup_physics.py',root/'mission-physics/shot.py']
cfg=dict(inputs={str(f.relative_to(root)):sha(f) for f in sources},setupRoster=sha(roster),field=plan['field'],commandMps=6.88,stationM=[1.54,5.55],required=[19,20],limitS=120,scoring='The full ring crossed the opening and fully shadowed both counter pairs. A failed or late leg fails that six-NOTE attempt.',cutoff='Stop after second failed run. Tests after first failure on a given run cannot restore its credit.',scope='Two start cases informed development. This is regression on the prespecified roster. Independent unseen setup validation remains separate.')
freeze=p/'freeze.json'
if freeze.exists():assert read(freeze)==cfg
else:write(freeze,cfg)
def run(case):
    row,n=case;attempt=row['attempt'];clock=0.;parts=[];score=0
    for i,leg in enumerate(plan['plan']):
        k=leg['id'];tag=f'{attempt}-{k}-{n}';a=p/(tag+'.json');b=p/f'split-ready-{tag}-6.88.json'
        with (p/(tag+'.log')).open('a') as log:
            if not a.exists():subprocess.run([sys.executable,'carry.py',k,attempt,str(n)],stdout=log,stderr=log,check=True)
            carry=read(a);ok=carry['seated'] and carry['held']
            if ok and not b.exists():subprocess.run([sys.executable,'fire.py',attempt+'-'+k,str(n),'ready','6.88'],stdout=log,stderr=log,check=True)
        travel=sum(leg['outT'])
        if i==0 and attempt!='dev':
            start,dock=leg['out'];L=math.hypot(dock[0]-start[0]-float(row['robot_x_mm'])/1000,dock[1]-start[1]-float(row['robot_y_mm'])/1000)
            travel=max(1.5*L/2.7,math.sqrt(6*L/2.6),math.sqrt(6*abs(math.radians(float(row['robot_heading_deg'])))/2.8))
        clock+=travel+(1.15 if i==0 else 3.4)+carry['finish']
        record=dict(id=k,carry=sha(a),held=carry['held'],seat=carry['seated'],endCarryClock=clock)
        if ok:
            sh=read(b);assert sh['sourceCarry']==record['carry'] and sh['robotX']==1.54
            ok=sh['passShot'];clock+=(sh['both'] if ok else sh['trace'][-1]['t'])
            record.update(shot=sha(b),wholeOpening=sh['opening'],both=sh['both'],bus=sh['minBusV'])
        score+=int(ok);record.update(credited=bool(ok),clock=clock);parts.append(record)
        if attempt!='dev' and (not ok or clock>120):break
    result=dict(attempt=attempt,n=n,score=score,passAttempt=bool(score==6 and clock<=120),elapsed=clock,parts=parts,config=sha(freeze))
    write(p/f'attempt-{attempt}-{n}.json',result);print(attempt,n,score,clock,flush=True);return result
rows=list(csv.DictReader(roster.open()))
with ThreadPoolExecutor(max_workers=4) as pool:
    # Run nominal and the first two commissioning starts together before allocating further compute.
    jobs=[pool.submit(run,c) for c in [({'attempt':'dev'},48),({'attempt':'dev'},96),(rows[0],96),(rows[1],96)]]
    nominal=[j.result() for j in jobs[:2]];out=[j.result() for j in jobs[2:]]
    while len(out)<20 and sum(not r['passAttempt'] for r in out)<2:
        batch=rows[len(out):len(out)+2];out.extend(pool.map(run,[(x,96) for x in batch]))
        write(p/'progress.json',dict(count=len(out),passes=sum(r['passAttempt'] for r in out)))
    write(p/'result.json',dict(nominal=nominal,attempts=out,untested=20-len(out),possible=20-sum(not r['passAttempt'] for r in out)))


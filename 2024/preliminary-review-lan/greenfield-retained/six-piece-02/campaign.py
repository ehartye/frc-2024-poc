"""Predeclared 20-start campaign, with mathematical early rejection after two failed attempts."""
from pathlib import Path
import hashlib,json,csv,sys,subprocess,time,math
from concurrent.futures import ThreadPoolExecutor,as_completed
p=Path(__file__).resolve().parent
def read(f):return json.loads(f.read_bytes())
def sha(f):return hashlib.sha256(f.read_bytes()).hexdigest()
enc=Path('C:/Users/ehart/repos/frc-2024-poc/sim/pressure/evidence/speaker-repeatability-starts.csv')
roster=list(csv.DictReader(enc.open()));plan=read(p/'plan.json')
assert len(roster)==20 and sha(enc)==plan['roster']
sources=[p/k for k in ['route.py','loaded.py','shoot.py','index_shoot.py','plan.json','campaign.py']]
sources+=[p.parent/'follower-01'/k for k in ['carry_cam.py','stroke.py','fire.py','fit.json','response.json']]
sources+=[p.parent/k for k in ['bounded-risk-01/pickup.py','bounded-risk-01/power.py','mission-physics/shot.py','pickup-lip/shot_replay.py','pickup-lip/pickup_trial.py','pickup_physics.py']]
config=dict(target='At least 19 of the 20 frozen setups score all six IDs within 120 s',pieces=[x['id'] for x in plan['plan']],order=[r['attempt'] for r in roster],roster=sha(enc),controller={k.name+':'+k.parent.name:sha(k) for k in sources},segments=96,station=[1.53,5.55],voltageFloor=8,stop='Two failed completed attempts rule out 19/20. Stop an attempted run at the first missed required piece.',scope='Preliminary commanded chassis tracking and ideal field localization. Measured NOTE pose, velocity, gravity and contact; 0.25 ms time step. Empty robots travel from their actual perturbed initial pose. Only the initial mark/robot offsets enter the world. Score sensors determine results.')
freeze=p/'campaign-freeze.json'
if freeze.exists():assert read(freeze)==config
else:freeze.write_text(json.dumps(config,indent=2))
def run(row):
    attempt=row['attempt'];rows=[];clock=0.;ok=True
    for i,leg in enumerate(plan['plan']):
        k=leg['id'];name=attempt+'-'+k+'-96';log=p/(name+'.log')
        with log.open('a') as f:
            for script,file in [('loaded.py',p/(name+'.json')),('index_shoot.py',p/('indexed-'+name+'.json'))]:
                if script=='index_shoot.py' and not (read(p/(name+'.json')).get('held') and read(p/(name+'.json')).get('seated')):break
                if not file.exists():
                    job=subprocess.run([sys.executable,script,k,attempt,'96'],cwd=p,stdout=f,stderr=f)
                    if job.returncode:raise RuntimeError(str(log))
        carry=read(p/(name+'.json'));shot=p/('indexed-'+name+'.json')
        if not i:
            a,b=leg['out'];dx=(b[0]-a[0])-float(row['robot_x_mm'])/1000;dy=(b[1]-a[1])-float(row['robot_y_mm'])/1000
            L=math.hypot(dx,dy);travel=max(1.5*L/2.7,math.sqrt(6*L/2.6),math.sqrt(6*abs(math.radians(float(row['robot_heading_deg'])))/2.8))
        else:travel=sum(leg['outT'])
        clock+=travel+(1.15 if not i else 3.4)+carry['finish']
        bind=dict(id=k,carry=sha(p/(name+'.json')),seat=carry['seated'],held=carry['held'],reachedStation=clock,sourceClock='commanded route with finite mark offsets')
        if shot.exists():
            r=read(shot);assert r['sourceCarry']==bind['carry'] and r['robotX']==1.53
            ok=bool(r['passShot']);clock+=r['both'] if ok else r['trace'][-1]['t']
            bind.update(shot=sha(shot),opening=r['opening'],both=r['both'],minBusV=r['minBusV'])
        else:ok=False
        rows.append(dict(**bind,clock=clock,credited=ok))
        if not ok or clock>120:break
    result=dict(id=attempt,freeze=sha(freeze),pieces=rows,passAttempt=bool(len(rows)==6 and all(x['credited'] for x in rows) and clock<=120),endS=clock)
    (p/('attempt-'+attempt+'.json')).write_text(json.dumps(result,indent=2));return result
stats=[];failed=0
for at in range(0,len(roster),2):
    with ThreadPoolExecutor(max_workers=2) as pool:batch=list(pool.map(run,roster[at:at+2]))
    for row in batch:
        stats.append(row['id']);failed+=not row['passAttempt'];print(row['id'],row['passAttempt'],[(k['id'],k['credited']) for k in row['pieces']],flush=True)
    (p/'campaign-status.json').write_text(json.dumps(dict(freeze=sha(freeze),attempts=stats,passed=len(stats)-failed,failed=failed,untested=20-len(stats),upperBound=20-failed,terminal=failed>=2 or len(stats)==20),indent=2))
    if failed>=2:break


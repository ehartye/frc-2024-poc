"""Drive the bounded command through the same six finite floor marks and field counter."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import hashlib,json,subprocess,sys,math,csv,numpy as np
p=Path(__file__).resolve().parent;root=p.parent
def read(f):return json.loads(f.read_bytes())
def sha(f):return hashlib.sha256(f.read_bytes()).hexdigest()
def save(f,r):
    tmp=f.with_suffix('.tmp');tmp.write_text(json.dumps(r,indent=2));tmp.replace(f)
roster=Path('C:/Users/ehart/repos/frc-2024-poc/sim/pressure/evidence/speaker-repeatability-starts.csv')
plan=read(p/'plan.json');assert sha(roster)==plan['roster']
def call(script,args,log):
    with log.open('w') as out:subprocess.run([sys.executable,script,*args],cwd=p,stdout=out,stderr=out,check=True)
def trial(row,n):
    name=row['attempt'];out=p/f'run-{name}-{n}.json';parts=[];end=0.;serial=0.;reset=0.;parallel=False
    for i,leg in enumerate(plan['plan']):
        k=leg['id'];tag=f'{name}-{k}';a=p/f'{tag}-{n}.json'
        if not a.exists():call('carry.py',[k,name,str(n),'working'],a.with_suffix('.log'))
        c=read(a);assert c['liftDurationS']==4.8 and abs(c['slideSmoothingS']-.16*4.8/3.4)<1e-8 and c['id']==k and c['attempt']==name
        travel=sum(leg['outT'])
        if i==0 and name!='dev':
            start,dock=leg['out'];L=math.hypot(dock[0]-start[0]-float(row['robot_x_mm'])/1000,dock[1]-start[1]-float(row['robot_y_mm'])/1000)
            travel=max(1.5*L/2.7,math.sqrt(6*L/2.6),math.sqrt(6*abs(math.radians(float(row['robot_heading_deg'])))/2.8))
        empty=1.15 if i==0 else 4.8
        depart=0 if i==0 else max(end,reset) if parallel else end+empty
        # First setup and each fresh pickup remain at the original free state. The return is physically replayed.
        shotStart=depart+travel+c['finish']+(1.15 if i==0 else 0.)
        serial+=empty+travel+c['finish']
        good=c['held'] and c['seated'] and c['beamProofS'] is not None and not c['failedGate']
        part=dict(id=k,depart=depart,shotStart=shotStart,carry=sha(a),gateTime=c['beamProofS'],held=c['held'],scored=False)
        if good:
            b=p/f'split-outerlane-{tag}-{n}-6.88.json'
            if not b.exists():call('lane.py',[tag,str(n)],b.with_suffix('.log'))
            s=read(b);assert s['sourceCarry']==sha(a) and s['pivot']==[1.575,5.871,.501] and s['angle']==122
            dur=s['both'] if s['passShot'] else s['trace'][-1]['t'];end=shotStart+dur;serial+=dur
            # Fixed on-robot delay from the feed command. Assess separation and electrical overlap independently.
            trigger=(s['feedTime']+1.) if s['feedTime'] is not None else math.inf
            samples=[r for j,r in enumerate(s['trace']) if j+1<len(s['trace']) and s['trace'][j+1]['t']>=trigger and r['t']<=trigger+4.8]
            tmax=max(np.diff([r['t'] for r in s['trace']]))
            # Screen with a full circumsphere and stated 12 m/s between-frame speed allowance.
            gap=(1.39+plan['envelope'][0][0]-max(r['q'][0] for r in samples)-.1778-12*tmax) if samples else None
            postA=max((11.8-r['busV'])/.027 for r in samples) if samples else None
            parallel=bool(s['passShot'] and trigger+4.8<=s['trace'][-1]['t'] and s['opening']['fullyThroughByS']<=trigger and gap is not None and gap>0 and postA<=30)
            reset=shotStart+trigger+4.8
            part.update(scored=s['passShot'],shot=sha(b),both=s['both'],opening=s['opening'],minBus=s['minBusV'],serialEnd=serial,counterClock=end,fieldClearanceDuringResetM=gap,afterFeedResidualBatteryA=postA,resetParallelEligible=parallel,resetEnd=reset)
            good=good and s['passShot'] and s['minBusV']>=8 and end<=120
        else:end=shotStart
        parts.append(part)
        verdict=dict(name=name,n=n,parts=parts,count=sum(x['scored'] for x in parts),passMission=bool(len(parts)==6 and good),terminal=bool(not good or len(parts)==6),elapsedS=end,serialElapsedS=serial,freeze=sha(p/'freeze.json'))
        save(out,verdict)
        if not good:break
    print(name,n,verdict['count'],round(end,2),flush=True);return verdict
if __name__=='__main__':
    cfg=dict(step=read(p/'clearance.json')['step'],roster=sha(roster),sources={str(x.relative_to(root)):sha(x) for x in [p/i for i in ('carry.py','shot.py','lane.py','mission.py','plan.json','clearance.json')]+[root/'loaded-actuation-01/smooth.py',root/'counter-drive-01/transmission.json',root/'counter-drive-01/counter.py',root/'seat-interlock-01/recovery.py']},firingPose=[1.39,5.871,122],fieldDriveFraction=.75,targetSpeedMps=6.88,liftS=4.8,resetS=4.8,resetTrigger='1.0 s after robot feed command; require swept X separation and <=30 A residual stationary shot load throughout reset before overlapping elapsed time',passCriterion='six unique NOTES, full opening passage, both field counters by 120 s, firing bus >=8 V, 19 of 20 starts',scope='Unloaded approach and reset still use prescribed tracking. Belt initialization at each pickup, chassis accuracy, loaded voltage, physical field transport calibration and active feedback require separate review.')
    f=p/'freeze.json'
    if f.exists():assert read(f)==cfg
    else:save(f,cfg)
    rosterRows=list(csv.DictReader(roster.open()))
    with ThreadPoolExecutor(max_workers=3) as ex:
        dev=list(ex.map(lambda n:trial({'attempt':'dev'},n),[48,96]))
        tested=[];save(p/'status.json',dict(dev=dev,roster=tested,unrun=20))
        if all(r['passMission'] for r in dev):
            while len(tested)<20 and sum(not r['passMission'] for r in tested)<2:
                team=rosterRows[len(tested):len(tested)+3];tested.extend(ex.map(lambda row:trial(row,96),team));save(p/'status.json',dict(dev=dev,roster=tested,unrun=20-len(tested)))


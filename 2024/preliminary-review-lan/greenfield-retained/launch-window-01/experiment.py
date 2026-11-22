"""Freeze a single-factor diagnostic on the scored mission before running any new setpoint."""
from pathlib import Path
import shutil,json,hashlib,sys,subprocess,concurrent.futures
p=Path(__file__).resolve().parent;old=p.parent/'split-field-02'
def sha(f):return hashlib.sha256(f.read_bytes()).hexdigest()
read=lambda f:json.loads(f.read_bytes())
cases=['R01-center-low-96','dev-center-low-96','dev-center-low-48']
matrix=[6.74,6.88,7.02]
sources={k:sha(old/(k+'.json')) for k in cases}
cfg=dict(predecessor=sha(old/'freeze.json'),carry=sources,stationM=[1.54,5.55],pitchDeg=125,setpointsMps=matrix,changed='Only the launch-belt setpoint changes. Each shot starts from the same saved incoming free NOTE and rotor states for that row.',readout='Full opening margin, both official detector pairs, release and credited time, minimum weak-pack voltage. Exclude a setting if it loses either nominal configuration, fails the known miss or falls below 8 V. A surviving setting is a candidate for a new full campaign, never retroactive credit.',baseline='Reuse exact 6.88 m/s issued shots for all three rows.',scope='One calibration case with two matched nominal controls. The frozen 19/20 result remains intact.')
freeze=p/'freeze.json'
if freeze.exists():assert read(freeze)==cfg
else:freeze.write_text(json.dumps(cfg,indent=2))
for tag in cases:
    f=p/(tag+'.json')
    if not f.exists():shutil.copyfile(old/f.name,f)
    assert sha(f)==sources[tag]
script=p/'shot.py'
if not script.exists():shutil.copyfile(old/'fire.py',script)
assert sha(script)==sha(old/'fire.py')
def run(pair):
    tag,v=pair;key,n=tag.rsplit('-',1);name=f'split-ready-{tag}-{v}.json'
    f=(old if v==6.88 else p)/name
    if not f.exists():
        with (p/(name+'.log')).open('w') as log:subprocess.run([sys.executable,str(script),key,n,'ready',str(v)],cwd=p,stdout=log,stderr=log,check=True)
    r=read(f);assert r['sourceCarry']==sources[tag] and r['targetSpeed']==v and r['robotX']==1.54 and r['n']==int(n)
    return dict(start=tag,setpoint=v,sha=sha(f),passShot=r['passShot'],opening=r['opening'],beams=r['beams'],both=r['both'],minBusV=r['minBusV'],release=r['release'])
with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
    verdict=list(pool.map(run,[(k,v) for k in cases for v in matrix]))
(p/'results.json').write_text(json.dumps(verdict,indent=2))
print([(r['start'],r['setpoint'],r['passShot'],r['both']) for r in verdict])

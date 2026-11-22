"""Compare the launch window across complete release states, including the failed full mission."""
from pathlib import Path
import json,hashlib
p=Path(__file__).resolve().parent
src=(p.parent/'six-piece-02/aim_window.py').read_text()
defs=src.split('out={}')[0].replace('dh=station-1.43','dh=station-1.54')
exec(compile(defs,'rigid torus launch window','exec'))
def read(f):return json.loads(f.read_bytes())
def sha(f):return hashlib.sha256(f.read_bytes()).hexdigest()
sources=sorted((p.parent/'split-field-02').glob('split-ready-*-6.88.json'))+sorted(p.glob('split-paired_.75_.75-mission-working-*-6.88.json'))
states={};aliases={}
for f in sources:
    d=read(f)
    if not d.get('release') or abs(d['robotX']-1.54)>1e-8:continue
    key=json.dumps(d['release'],sort_keys=True)
    if key in aliases:continue
    a=margin(d['release'])
    states[f.name]=dict(fileHash=sha(f),release=d['release'],score=d['passShot'],wholeOpening=d['opening'],marginsM=a.tolist())
    aliases[key]=f.name
grid=np.array([v['marginsM'] for v in states.values()]);worst=grid.min(0);best=np.argmax(worst)
old=np.argmin(abs(stations-1.54));name=list(states)[np.argmin(grid[:,old])]
res=dict(method='Gravity and torque-free ring rotation from measured six-DOF releases. Sampled 1 ms before the top of flight. Ring arc and between-sample motion conservatively inflate its required aperture. Contact with a field wall invalidates the subsequent free flight.',windowCode=sha(p.parent/'six-piece-02/aim_window.py'),stationsM=stations.tolist(),cases=states,count=len(states),minMarginAtOriginalM=float(worst[old]),limiterAtOriginal=name,bestXM=float(stations[best]),bestWorstMarginM=float(worst[best]),predictedClearAtBest=int((grid[:,best]>0).sum()),windowM=stations[worst>0].tolist())
(p/'station-window.json').write_text(json.dumps(res));print(res['count'],name,res['minMarginAtOriginalM'],res['bestXM'],res['bestWorstMarginM'],res['windowM'])


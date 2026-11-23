"""Predict common free-flight opening margin for measured slower releases. Replay the fitted station against the field."""
from pathlib import Path
import json,sys,subprocess,concurrent.futures
p=Path(__file__).resolve().parent
s=(p.parent/'six-piece-02/aim_window.py').read_text().split('out={}')[0]
s=s.replace('1.19,1.76','1.21,2.02').replace('station-1.43','station-1.87')
exec(compile(s,'finite torus passage','exec'))
files=sorted(p.glob('apex-1.87-*.json'));assert len(files)==4
rows=np.array([margin(json.loads(f.read_bytes())['release']) for f in files]);worst=rows.min(axis=0);x=round(stations[int(np.argmax(worst))],2)
print('station',x,'worst',worst.max(),'range',stations[worst>0],flush=True)
def run(r):
    d=json.loads(r.read_bytes());label=d['name'].split('ready-')[-1].rsplit('-',2)[0];n=d['n']
    with (p/f'geometry-aim-{label}-{n}.log').open('w') as log:subprocess.run([sys.executable,'apex_probe.py',label,str(n),'6.88',str(x)],stdout=log,stderr=log,check=True)
if worst.max()>0:
    with concurrent.futures.ThreadPoolExecutor(4) as pool:list(pool.map(run,files))


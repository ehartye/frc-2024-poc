"""Screen fixed elevation changes against every measured release before changing the physical controller."""
from pathlib import Path
import numpy as np,json,math
from scipy.spatial.transform import Rotation
p=Path(__file__).resolve().parent
prior=json.loads((p/'station-window.json').read_bytes())
exec(compile((p/'station-window.py').read_text().split('sources=sorted')[0],'analytic finite ring','exec'))
cases=prior['cases'];pivot=np.array([1.725,5.55,.501])
def rotate_release(row,delta):
    q=np.array(row['q']);v=np.array(row['v']);d=Rotation.from_euler('y',-delta,degrees=True);current=Rotation.from_quat([*q[4:],q[3]])
    qq=(d*current).as_quat();return dict(q=[*list(pivot+d.apply(q[:3]-pivot)),qq[3],*qq[:3]],v=[*list(d.apply(v[:3])),*list(v[3:])])
rows=[]
for angle in [121,122,123,124,125,127,129]:
    obs={name:margin(rotate_release(row['release'],angle-125)) for name,row in cases.items()}
    worst=np.array(list(obs.values())).min(axis=0);i=int(np.argmax(worst));j=int(np.argmin(abs(stations-1.54)))
    rows.append(dict(angle=angle,stationM=float(stations[i]),worstMarginMm=float(worst[i]*1000),marginAtOriginalMm=float(worst[j]*1000),above20mmM=stations[worst>.020].tolist(),limiters=sorted(obs,key=lambda n:obs[n][i])[:3]))
ans=dict(parent=sha(p/'station-window.json'),n=len(cases),rows=rows,scope='Rigidly rotate observed releases, then run ballistic motion under gravity. Candidate screening only. The real carry and launcher must be rerun at the selected angle, including opening, both field sensors and bus voltage.')
(p/'elevation-window.json').write_text(json.dumps(ans,indent=2));print([(x['angle'],x['stationM'],round(x['worstMarginMm'],1),round(x['marginAtOriginalMm'],1)) for x in rows])


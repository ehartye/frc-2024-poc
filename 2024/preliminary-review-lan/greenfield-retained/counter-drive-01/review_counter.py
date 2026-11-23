"""Publish two matched field-counter replays using the issued robot rig and free-piece trace."""
from pathlib import Path
import hashlib,json,copy
import numpy as np
from scipy.spatial.transform import Rotation
from paired_cycle import read,sha,verify
p=Path(__file__).resolve().parent;root=p.parent;web=root.parent/'www/data.json'
baseline=root/'split-field-02/split-ready-R01-center-low-96-6.88.json'
source=p/'split-paired_.75_.75-R01-center-low-96-6.88.json';other=p/'split-paired_.75_-.75-R01-center-low-96-6.88.json'
review=[]
for name in sorted(p.glob('split-paired_*center-*.json')):
    if 'R01-center-low' not in name.name and 'dev-center-high' not in name.name:continue
    old=(root/('launch-window-01' if '6.74' in name.name else 'split-field-02')/('split-ready-'+name.name[min(i for i in (name.name.find('-R01-'),name.name.find('-dev-')) if i>=0)+1:]))
    d=verify(name,old)
    review.append(dict(raw=name.name,hash=sha(name),old=sha(old),originalBoth=read(old)['both'],**d))
(p/'paired-trials.json').write_text(json.dumps(review,indent=2))
doc=read(web);g=doc['concepts']['guided'];template=next(c for c in doc['concepts']['field']['cases'] if c['key']=='field-96')
def mat(angle,x=0,y=0,z=0):
    m=np.eye(4);m[:3,:3]=Rotation.from_euler('y',-angle,degrees=True).as_matrix();m[:3,3]=[x,y,z];return m
atseat=mat(125,185,0,501)@mat(0,215,0,50.4)
flap=mat(125,185,0,501)@mat(0,457.5,0,96)@mat(90)@mat(0,-457.5,0,-96)@np.linalg.inv(mat(125,185,0,501))
for key,title,field in [('counter-driven','Speaker counter · driven surfaces',source),('counter-reversed','Speaker counter · reverse upper drive',other)]:
    shot=read(field);s=verify(field,baseline)
    xy=[1540,5550];frames=[];times=[];positions=[];status=[]
    for v in shot['trace']:
        t=v['t'];q=v['q']
        pose=np.eye(4);pose[:3,:3]=Rotation.from_quat([*q[4:],q[3]]).as_matrix();pose[:3,3]=np.array(q[:3])*1000-[*xy,0]
        frames.append((pose@np.linalg.inv(atseat))[:3].round(7).flatten().tolist());times.append(t);positions.append(xy)
        count=sum(b is not None and t>=b for b in s['beams'])
        event='Wheel spin-up' if t<shot['feedTime'] else 'Feeding' if t<shot['release']['t'] else 'Free flight' if t<s['opening']['fullyThroughByS'] else 'Inside the speaker'
        status.append(f'{event} · {count}/2 modeled counter pairs crossed · {t:.2f} s')
    rig={**template['rig'],**{n:'arm' for n in g['add']}}
    series={k:([np.eye(4)[:3].flatten().tolist()]*len(times)) for k in ['arm','slide','frame']}
    series.update(piece=frames,flap=[flap[:3].round(7).flatten().tolist()]*len(times))
    proved=s['passShot'];info='Both counter pairs crossed at %.3f s.'%s['both'] if proved else 'The NOTE passed the opening and missed the completed count.'
    case=dict(key=key,id=title,stage='Mission rehearsal',gap=None,removed=[],positions=positions,status='sampled' if proved else 'fail',basis='Same R01 final NOTE, incoming state, 6.88 m/s robot command and 106 mm full-opening margin. The drawing couples the upper shaft and front rollers in opposite directions. Nominal 75% of no-load speed is a sensitivity assumption. Only the upper motor sign changes between these two views. '+info,frames={},results=[dict(pair=['R01 final NOTE','Speaker counter'],status='sampled' if proved else 'fail',witness=dict(reason=info))],time=times,measure=status,rig=rig,series=series,narrow=0,fixture='shot')
    # The renderer displays a clearance line only when an actual clearance is available.
    g['cases']=[x for x in g['cases'] if x['key']!=key]+[case]
tmp=web.with_suffix('.counter-next');tmp.write_text(json.dumps(doc,separators=(',',':')),encoding='utf8');tmp.replace(web)


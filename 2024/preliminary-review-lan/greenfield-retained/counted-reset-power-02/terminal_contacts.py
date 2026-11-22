"""Identify the supporting official field components at each open counter outcome."""
from pathlib import Path
from types import SimpleNamespace
import json,sys,math
import numpy as np
from scipy.spatial.transform import Rotation
p=Path(__file__).resolve().parent;field=Path('C:/Users/ehart/Documents/FRC Native/2024/six-speaker-20261118/field-source/blue-speaker')
sys.path[:0]=[str(p.parent/'counter-drive-01'),str(field.parent.parent/'field-query-libs'),'C:/Users/ehart/repos/frc-2024-poc/sim/pressure']
from speaker_surfaces import SpeakerSurfaces
import run_speaker_field as mod
from counter import bind
trans,surfaceVelocity=bind(mod,.75,.75)
surfaces=SpeakerSurfaces(field);parts={part['key']:part for part,*_ in surfaces.parts}
read=lambda f:json.loads(f.read_bytes());report=[]
for r in read(p/'counter-clear.json'):
    if r['sustainedBothBeamsClearFromSampleS'] is not None:continue
    n=r['n'];k=r['id'];s=read(p/f'split-tail_.25_0-dev-{k}-{n}-6.88.json')
    tr=s['trace'];q=np.array(tr[-1]['q']);prev=np.array(tr[-2]['q']);dt=tr[-1]['t']-tr[-2]['t']
    rotation=lambda q:Rotation.from_quat(q[[4,5,6,3]])
    a=rotation(q);b=rotation(prev);W=(a*b.inv()).as_rotvec()/dt;v=(q[:3]-prev[:3])/dt
    ring=np.linspace(0,2*np.pi,n,endpoint=False);points=q[:3]+a.apply(.1524*np.c_[np.cos(ring),np.sin(ring),np.zeros(n)])
    fw,contact,depth=mod.field_wrench(surfaces,points,q[:3],v,W,SimpleNamespace(stiffness=1.2e6,damping=450.,friction=.45,belt_speed=1.5))
    rows=[dict(key=id,article=parts[id]['name'],fullPath=parts[id].get('path'),normalN=N) for id,N in sorted(contact.items(),key=lambda kv:-kv[1])]
    report.append(dict(case=k,n=n,near7sBodySpeedMps=float(np.linalg.norm(v)),sevenSecondsXYZ=q[:3].tolist(),wrenchN=fw[:3].tolist(),contacts=rows,depthM=depth))
    print(n,k,[(a['article'],round(a['normalN'],1)) for a in rows])
surfaces.close();(p/'terminal-contacts.json').write_text(json.dumps(report,indent=2))


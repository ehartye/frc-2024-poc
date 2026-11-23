"""Released six-DOF NOTE in stationary, powerless cassette. No grip/friction credit."""
import json, math as m
from pathlib import Path
import numpy as np
import mujoco as mj
here=Path(__file__).resolve().parent
def run(n,dt,angle,closed):
    def box(name,p,h):
        return f'<geom name="{name}" type="box" pos="{p}" size="{h}" contype="2" conaffinity="1"/>'
    solids=box('stop','.0326 0 .0415','.0046 .048 .0225')
    for y in [-.1225,.1225]:solids+=box('runner'+str(y),f'.32 {y} .022','.289 .0225 .003')
    for y in [-.1958,.1958]:solids+=box('belt'+str(y),f'.3275 {y} .048','.2305 .018 .016')
    if closed:
        solids+=box('roof','.130 0 .083','.009 .226 .003')+'<geom name="pin" type="cylinder" pos=".130 .055 .056" size=".0035 .042" contype="2" conaffinity="1"/>'
    geom=''
    for i in range(n):
        a=i*2*m.pi/n;b=(i+1)*2*m.pi/n
        geom+=f'<geom type="capsule" fromto="{.1524*m.cos(a)} {.1524*m.sin(a)} 0 {.1524*m.cos(b)} {.1524*m.sin(b)} 0" size=".0254" mass="{.25/n}" contype="1" conaffinity="2"/>'
    s=-9.80665*m.sin(m.radians(angle));h=-9.80665*m.cos(m.radians(angle))
    xml=f'<mujoco><option timestep="{dt}" gravity="{s} 0 {h}" iterations="90"/><default><geom condim="1" solref=".004 1" solimp=".95 .99 .0005"/></default><worldbody>{solids}<body pos=".215 0 .0504"><freejoint/>{geom}</body></worldbody></mujoco>'
    model=mj.MjModel.from_xml_string(xml);d=mj.MjData(model);peak=0;poses=[]
    for step in range(round(1.5/dt)+1):
        mj.mj_step(model,d);dist=np.linalg.norm(d.qpos[:3]-[.215,0,.0504]);peak=max(peak,float(dist))
        if step%round(.025/dt)==0:poses.append(d.qpos.tolist())
        if dist>1:break
    return dict(angleDeg=angle,n=n,dt=dt,latchAndRoof=closed,maximumOffsetMm=1000*peak,endSpeedMps=np.linalg.norm(d.qvel[:3]),finalMm=(d.qpos[:3]*1000).tolist(),poses=poses)
rows=[run(n,dt,a,c) for n,dt in [(48,.001),(96,.0005)] for a in [-50,0,45,90,125] for c in (False,True)]
for r in rows:print(r['angleDeg'],r['n'],r['latchAndRoof'],round(r['maximumOffsetMm'],2),round(r['endSpeedMps'],3))
(here/'free-hold.json').write_text(json.dumps(dict(mujoco=mj.__version__,scope='Frictionless contact. Same initial centered ring in each paired run; rigid segmented torus with compliant numerical contact. Independent hold at each angle, no imposed game-piece position.',cases=rows),separators=(',',':')))


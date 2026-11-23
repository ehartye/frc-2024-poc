"""Free chassis with unilateral tire/bumper contacts and a two-anchor articulated chain."""
import json,sys,math
from pathlib import Path
import mujoco as mj
import numpy as np
dt=float(sys.argv[2]);enabled=int(sys.argv[3]);hold=int(sys.argv[4]);n=24
span=1.210569
ys=np.linspace(-span,span,n+1)
z=.717505+1.65855*(np.cosh(ys/1.65855)-1)
k1,k2=n//2-1,n//2+1
start=np.pi-np.arcsin((z[k1]-.501)/np.hypot(.18,.232))-np.arctan2(.232,.18)
cx=.185+.18*np.cos(start)-.232*np.sin(start)
nodes=np.column_stack([np.full(n+1,cx),ys,z])
def fmt(vals):return ' '.join(str(float(x)) for x in vals)
chain=''
for i,(a,b) in enumerate(zip(nodes,nodes[1:])):
    off=a if i==0 else a-nodes[i-1]
    delta=b-a
    chain+=f'<body name="c{i}" pos="{fmt(off)}"><joint type="ball" damping=".005"/><geom type="capsule" fromto="0 0 0 {fmt(delta)}" size=".013335" mass=".085" contype="0" conaffinity="0"/><site name="s{i}"/>'
chain+=f'<site name="last" pos="{fmt(nodes[-1]-nodes[-2])}"/>'+n*'</body>'
tires=''.join(f'<geom name="tire{i}" type="sphere" size=".0508" pos="{x} {y} .0508"/>' for i,(x,y) in enumerate([(x,y) for x in (-.27,.27) for y in (-.27,.27)]))
xCG=float(sys.argv[5]) if len(sys.argv)>5 else 0.
xml=f'''<mujoco><option timestep="{dt}" integrator="implicitfast" iterations="80"/><default><geom friction=".8 .001 .001" solref=".006 1"/></default>
<worldbody><geom name="floor" type="plane" size="3 3 .05"/><site name="anchor" pos="{fmt(nodes[-1])}"/>{chain}
<body name="frame"><freejoint/><inertial mass="62.56" pos="{xCG} 0 .30" diaginertia="4.5 4.5 6.5"/>{tires}
<geom name="bumper" type="box" size=".437 .436 .064" pos="0 0 .122"/>
<body name="cassette" pos=".185 0 .501"><joint name="pivot" axis="0 -1 0" damping=".5"/><inertial mass="6" pos=".28 0 .05" diaginertia=".13 .30 .32"/>
<geom name="deck" type="box" pos=".32 0 .022" size=".30 .21 .008" contype="2" conaffinity="1"/>
<site name="h1" pos=".18 {ys[k1]} .232"/><site name="h2" pos=".18 {ys[k2]} .232"/></body></body></worldbody>
<equality><connect site1="last" site2="anchor" solref=".004 1"/>
<connect name="left" site1="h1" site2="s{k1}" active="false" solref=".01 1"/><connect name="right" site1="h2" site2="s{k2}" active="false" solref=".01 1"/>
<joint name="brake" joint1="pivot" active="false" polycoef="{math.radians(165)} 0 0 0 0" solref=".008 1"/></equality></mujoco>'''
m=mj.MjModel.from_xml_string(xml);d=mj.MjData(m)
jid=m.joint('pivot');qi=jid.qposadr[0];vi=jid.dofadr[0];d.qpos[qi]=start
fr=m.body('frame').id
chainids=[m.site('s'+str(i)).id for i in range(n)]+[m.site('last').id]
grasp=[];trace=[];torques=[];holds=[];cut=4.
for i in range(int(9/dt)+1):
    mj.mj_forward(m,d)
    t=d.time;angle=d.qpos[qi]
    if t>=.25 and t<.5 and enabled:
        for eq,hid,idx in [(1,'h1',k1),(2,'h2',k2)]:
            gap=np.linalg.norm(d.site_xpos[m.site(hid).id]-d.site_xpos[chainids[idx]])
            if not d.eq_active[eq] and gap<.012:d.eq_active[eq]=1;grasp.append([t,eq,gap])
    u=np.clip((t-.55)/2.4,0,1);target=start+(np.deg2rad(165)-start)*u*u*(3-2*u)
    demand=3000*(target-angle)-150*d.qvel[vi]
    torque=float(np.clip(demand,-215.,215.)) if t<cut else 0.
    if t>=cut and hold and not d.eq_active[3] and abs(target-angle)<np.deg2rad(3): 
        m.eq_data[3,0]=angle;d.eq_active[3]=1
    d.qfrc_applied[vi]=torque;torques.append(torque)
    if i%round(.025/dt)==0:
        ground=0.;force=np.zeros(6)
        for j in range(d.ncon):
            c=d.contact[j]
            if m.geom('floor').id in (c.geom1,c.geom2):mj.mj_contactForce(m,d,j,force);ground+=force[0]
        corners=[d.xpos[fr]+d.xmat[fr].reshape(3,3)@np.array([x,y,.058]) for x in (-.437,.437) for y in (-.436,.436)]
        zmin=min([p[2] for p in corners]+[d.geom_xpos[m.geom('tire'+str(j)).id,2]-.0508 for j in range(4)])
        trace.append(dict(t=t,p=d.xpos[fr].tolist(),q=d.xquat[fr].tolist(),angle=float(np.rad2deg(angle)),chain=d.site_xpos[chainids].tolist(),floorN=ground,minFloorM=zmin,torqueNm=torque,driveA=abs(torque)/(.0198*250*.72),hold=bool(d.eq_active[3])))
        if t>=cut+.1:holds.append(trace[-1])
    mj.mj_step(m,d)
r=dict(trace=trace,dt=dt,gates=grasp,contactDeg=float(np.rad2deg(start)),chainX=cx,peakNm=max(abs(np.array(torques))),minHeldFloorM=min(s['minFloorM'] for s in holds),maxHeldFloorN=max(s['floorN'] for s in holds),held=all(s['hold'] for s in holds),disabledS=5,activeMassKg=68.56,cgX=xCG)
Path(sys.argv[1]+'.json').write_text(json.dumps(r,separators=(',',':')))
print({k:v for k,v in r.items() if k!='trace'})


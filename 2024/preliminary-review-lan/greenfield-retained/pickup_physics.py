"""Free six-DOF ring, gravity and compliant contacts. Cassette held at pickup angle.
Circular torus approximated with equal-mass capsules. Belt pads follow a closed
stadium path with prescribed surface velocities. Only robot travel and drives are commanded.
Units m, s, kg. CSV-equivalent sparse world trajectories retained per run."""
import math as m, json, sys
from pathlib import Path
import numpy as np
import mujoco as mj
HERE=Path(__file__).resolve().parent
C=m.cos(m.radians(-50)); S=m.sin(m.radians(-50)); X=.1693625892825837; Z=.501
beater=False;phase=0;shoe=0;press=False;stroke=.095;angle=50;topGeom=(.624,.075,.025)
def run(n,dt,enabled,upper=True,offset=0,mu=.75):
    points=[]; joints=[]; speed=.84 if enabled else 0
    def geom(kind,size,attr=''):
        return f'<geom type="{kind}" {size} {attr}/>'
    def body(pos,child,more=''):
        return f'<body pos="{pos}" {more}>{child}</body>'
    def rotor(name,s,y,h,r,w):
        shaft=geom('cylinder',f'size="{r if name!="top" or not beater else .022} {w/2}" quat=".7071068 .7071068 0 0"','mass=".16" contype="2" conaffinity="1"')
        if name=='top' and beater:
            for i in range(6):
                phi=m.radians(phase+i*60);x=r*m.cos(phi);z=r*m.sin(phi)
                for lane in [-.095,0,.095]:
                    shaft+=geom('capsule',f'fromto="{.15*x} {lane} {.15*z} {x} {lane} {z}" size=".009"','mass=".014" contype="2" conaffinity="1"')
        return body(f'{s} {y} {h}',f'<joint name="{name}" type="hinge" axis="0 1 0"/>'+shaft)
    tape=''
    count=38; a=.091;b=.625;radius=.011;length=2*(b-a)+2*m.pi*radius;spacing=length/count
    for i in range(count):
        tape+=body('0 0 0',f'<joint name="s{i}" type="slide" axis="1 0 0"/><joint type="slide" axis="0 0 1"/><joint type="hinge" axis="0 -1 0"/>'+geom('box',f'size="{spacing*.48} .0225 .001"','mass=".07" contype="2" conaffinity="1"'))
    fixed=''
    def block(x,y,z,hx,hy,hz):
        return geom('box',f'pos="{x} {y} {z}" size="{hx} {hy} {hz}"','contype="2" conaffinity="1"')
    for side in (-1,1):
        fixed+=block(.320,side*.1225,.022,.289,.0225,.003)
    if shoe:
        for y in (-.115,.115):fixed+=block(.640,y,.02375,.0315,shoe,.00125)
    fixed+=block(.0326,0,.0415,.0046,.048,.0225)
    for y in [-.12,.12]:fixed+=rotor('nose'+str(int(y*100)),.625,y,.013,.012,.19)
    if upper: fixed+=rotor('top',topGeom[0],0,topGeom[1],topGeom[2],.26)
    if press:
        for i in range(7):fixed+=rotor(f'pressure{i}',topGeom[0]-(i+1)*.060,0,(.084 if beater else topGeom[1]),.029,.285)
    ring=''
    for i in range(n):
        x,y=np.array([m.cos(i*2*m.pi/n),m.sin(i*2*m.pi/n)])*.1524
        u,v=np.array([m.cos((i+1)*2*m.pi/n),m.sin((i+1)*2*m.pi/n)])*.1524
        ring+=geom('capsule',f'fromto="{x} {y} 0 {u} {v} 0" size=".0254"',f'mass="{.25/n}" contype="1" conaffinity="2" friction="{mu} .003 .001"')
    arm=body(f'{X} 0 {Z}',fixed+tape,f'euler="0 {angle} 0"')
    robot=body('0 0 0','<joint name="drive" type="slide" axis="1 0 0"/><inertial pos="0 0 0" mass="50" diaginertia="1 1 1"/>'+arm)
    xml=f'<mujoco><compiler angle="degree"/><option timestep="{dt}" gravity="0 0 -9.80665" integrator="implicitfast" cone="elliptic" iterations="70"/><default><geom friction="{mu} .003 .001" solref=".008 1" solimp=".9 .98 .002" /></default><worldbody><geom type="plane" size="3 3 .1" contype="2" conaffinity="1" friction=".72 .003 .001"/>{robot}'+body(f'.7866309 {offset} .0254','<freejoint name="piece"/>'+ring)+'</worldbody></mujoco>'
    model=mj.MjModel.from_xml_string(xml); d=mj.MjData(model)
    jid=lambda name:mj.mj_name2id(model,mj.mjtObj.mjOBJ_JOINT,name)
    ids=[jid(f's{i}') for i in range(count)];qs=model.jnt_qposadr[ids];vs=model.jnt_dofadr[ids]
    rj=[(jid('nose-12'),-speed/.012),(jid('nose12'),-speed/.012)]
    if upper:rj.append((jid('top'),speed/topGeom[2]))
    if press:
        for i in range(7):rj.append((jid(f'pressure{i}'),speed/.029))
    qi=model.jnt_qposadr[jid('piece')]; drive=jid('drive'); near=100; seated=0; elapsed=0
    def track(u):
        # Increasing arc length moves inward along the upper face.
        L=b-a;u%=length
        if u<L:return (b-u,.013+radius,0,-1,0,0)
        u-=L
        if u<m.pi*radius:
            t=u/radius;return (a-radius*m.sin(t),.013+radius*m.cos(t),t,-m.cos(t),-m.sin(t),1/radius)
        u-=m.pi*radius
        if u<L:return (a+u,.013-radius,m.pi,1,0,0)
        t=(u-L)/radius;return (b+radius*m.sin(t),.013-radius*m.cos(t),m.pi+t,m.cos(t),m.sin(t),1/radius)
    for step in range(round(2.0/dt)+1):
        t=step*dt
        # Same 95 mm chassis advance with full drive enabled or disabled.
        advance=min(stroke,t*.25);d.qpos[model.jnt_qposadr[drive]]=advance;d.qvel[model.jnt_dofadr[drive]]=.25 if t<stroke/.25 else 0
        vals=np.array([track(i*spacing+speed*t) for i in range(count)])
        for k in range(3):d.qpos[qs+k]=vals[:,k];d.qvel[vs+k]=vals[:,k+3]*speed
        for j,w in rj:d.qvel[model.jnt_dofadr[j]]=w
        mj.mj_step(model,d)
        pos=d.qpos[qi:qi+3].copy();dx=pos[0]-advance-X;dz=pos[2]-Z
        s=C*dx+S*dz; h=-S*dx+C*dz
        near=min(near,s); elapsed=elapsed+dt if abs(s-.215)<.018 and abs(h-.0504)<.014 and abs(pos[1])<.035 else 0
        seated=max(elapsed,seated)
        if step%round(.01/dt)==0:points.append([round(t,5),*np.round(pos-[advance,0,0],7),*np.round(d.qpos[qi+3:qi+7],7),round(s,6),round(h,6)])
    return dict(n=n,dt=dt,enabled=enabled,upper=upper,mu=mu,offset=offset,minimumCenterS=near,seatResidenceS=seated,seated=seated>=.04,trajectory=points)
if __name__=='__main__':
    args=sys.argv[1:];n=int(args[0]);dt=float(args[1]); enabled=bool(int(args[2])); upper=bool(int(args[3]));offset=float(args[4]) if len(args)>4 else 0;mu=float(args[5]) if len(args)>5 else .75
    result=run(n,dt,enabled,upper,offset,mu);name=f'pickup-{n}-{dt}-{int(enabled)}-{int(upper)}-{offset}-{mu}.json'
    (HERE/name).write_text(json.dumps(result,separators=(',',':')));print(name,{k:v for k,v in result.items() if k!='trajectory'},result['trajectory'][-1])


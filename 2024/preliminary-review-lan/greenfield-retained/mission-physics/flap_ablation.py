"""Power-off cassette retention. Paired mouth geometry on the selected P4 belt dimensions."""
from pathlib import Path
import ast, hashlib, json, math, sys
import mujoco as mj
import numpy as np
here=Path(__file__).resolve().parent
source=(here/'shot.py').read_text()
def path(t,case):
    angle=-50 if case!='aim' else 125
    if case=='raise':
        u=np.clip(t/1.5,0,1);angle+=175*u*u*(3-2*u)
    a=math.radians(angle);co,si=math.cos(a),math.sin(a)
    reach=max(-.018*co,.643*co)+max(.044*si,-.067*si)
    return a,min(.185,.634-reach)
def run(case,gate,brake,dt,n):
    angle,x=path(0,case);pivot=np.array([x,0,.501])
    def rot(a):
        c,s=math.cos(a),math.sin(a);return np.array([[c,0,-s],[0,1,0],[s,0,c]])
    R=rot(angle);initial=pivot+R@np.array([.215,0,.0504])
    env=dict(n=n,major=.1524,minor=.0254,mass=.2413,math=math)
    start=source.index('    def box(');end=source.index('    xml=f')
    import textwrap
    exec(textwrap.dedent(source[start:end]),env)
    stock=env['stock'].split('<geom name="flapBarrel"')[0]
    if gate:stock+=env['box'](.3995,0,.058,.0015,.037,.026,'closedFlap')
    fmt=lambda x:' '.join(str(y) for y in x)
    xml=f'''<mujoco><option timestep="{dt}" gravity="0 0 -9.80665" integrator="implicitfast"/><default><geom solref=".008 1" solimp=".94 .99 .001"/></default><worldbody>
    <geom type="plane" size="3 3 .05" name="floor" contype="2" conaffinity="1"/>
    <body pos="0 0 .501" gravcomp="1"><joint type="slide" axis="1 0 0"/><inertial pos="0 0 0" mass="10000" diaginertia="1000 1000 1000"/>
    <body gravcomp="1"><joint type="hinge" axis="0 -1 0"/><inertial pos="0 0 0" mass="10000" diaginertia="1000 1000 1000"/>{stock}</body></body>
    <body name="note" pos="{fmt(initial)}" quat="{fmt([math.cos(angle/2),0,-math.sin(angle/2),0])}"><freejoint/><inertial pos="0 0 0" mass=".2413" diaginertia="{env['ix']} {env['ix']} {env['iz']}"/>{env['bead']}</body></worldbody></mujoco>'''
    model=mj.MjModel.from_xml_string(xml);d=mj.MjData(model);b=model.body('note').id
    phi=np.arange(n)*2*np.pi/n;beads=.1524*np.stack([np.cos(phi),np.sin(phi),np.zeros(n)],axis=1);belts=np.zeros(2)
    maxOut=0;firstMouth=None;floor=None;last=0;maxGate=0
    for step in range(round(2/dt)+1):
        t=step*dt;a,x=path(t,case);an,xn=path(t+dt,case);R=rot(a);pivot[0]=x
        d.qpos[:2]=[x,a];d.qvel[:2]=[(xn-x)/dt,(an-a)/dt];mj.mj_step1(model,d)
        world=d.xmat[b].reshape(3,3);center=d.qpos[2:5];v=d.qvel[2:5];omega=world@d.qvel[5:8];pts=beads@world.T+center
        local=(pts-pivot)@R;wr=np.zeros(6);axle=np.array([0,-d.qvel[1],0])
        for j,side in enumerate((-1,1)):
            nearest=np.column_stack([np.clip(local[:,0],.097,.558),np.full(n,side*.1898),np.clip(local[:,2],.032,.064)])
            diff=local-nearest;length=np.linalg.norm(diff,axis=1);i=length.argmin();depth=.0434-length[i];F=0.
            if depth>0:
                normal=R@(diff[i]/length[i]);site=pts[i]-.0254*normal;arm=site-center;rel=v+np.cross(omega,arm)-np.array([d.qvel[0],0,0])-np.cross(axle,site-pivot)
                N=max(0.,4500*depth-5*np.dot(rel,normal));direction=R[:,0]-normal*np.dot(R[:,0],normal);direction/=max(.01,np.linalg.norm(direction))
                F=np.clip(45*(belts[j]-rel@direction),-min(.78*N,18),min(.78*N,18));force=N*normal+F*direction;wr[:3]+=force;wr[3:]+=np.cross(arm,force)
            # A shorted motor provides velocity-dependent torque, zero at rest.
            # Coast includes only the baseline speed-dependent running drag; no dry breakaway credit.
            drag=.4+(.0198*.0202/(.115*.018**2) if brake=='shorted' else 0)
            belts[j]+=dt*(-F-drag*belts[j])/.20
            if brake=='locked':belts[j]=0.
        d.xfrc_applied[b]=wr;s=(center-pivot)@R[:,0];last=(s-.215)*1000;maxOut=max(maxOut,last)
        if firstMouth is None and s>.3995:firstMouth=t
        for ct in d.contact:
            if model.geom('floor').id in (ct.geom1,ct.geom2) and floor is None:floor=t
        if floor is not None:break
        mj.mj_step2(model,d)
    return dict(case=case,flap=gate,belt=brake,dt=dt,segments=n,peakOutwardMm=float(maxOut),endShiftMm=float(last),centerPastFlapS=firstMouth,carpetContactS=floor)
if __name__=='__main__':
    targets=[('pickup',False,'coast'),('pickup',True,'coast'),('pickup',False,'shorted'),('pickup',False,'locked'),('aim',False,'coast'),('raise',False,'coast'),('raise',True,'coast')]
    n=int(sys.argv[1]);dt=.024/n
    rows=[run(*r,dt,n) for r in targets]
    print(json.dumps(rows,indent=1));(here/f'flap-ablation-{n}.json').write_text(json.dumps(dict(source=hashlib.sha256(source.encode()).hexdigest(),scope='Seated piece, two-second hold or 1.5 s smooth lift, level stationary chassis. Closed flap is an ideal unilateral obstruction. No frictional breakaway credited to idle motors. Locked bounds the effect of an ideal physical shaft lock.',trials=rows),indent=2))


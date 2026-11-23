"""Return the passive flap and drive states from the previous shot to the actual next pickup."""
from pathlib import Path
import math,sys,json
import numpy as np,mujoco as mj
here=Path(__file__).resolve().parent
sys.path[:0]=[str(here.parent/'follower-01'),str(here.parent/'loaded-actuation-01')]
from smooth import table,times
from stroke import scheduled
layout=json.loads((here.parent/'pickup-lip/layout.json').read_bytes())
low=json.loads((here.parent/'bounded-lift-01/dev-middle-48.json').read_bytes())['pickAngle'];x0=layout['pickPivot'][0]
tb,_=table(low,122,x0)
def smooth(u):return u*u*(3-2*u)
def transport(leg,travel,initialYaw,firstOffset,prev,wait,n):
    first=prev is None
    dt=.0005 if n==48 else .00025
    end=travel+1.15 if first else wait+4.8+travel
    t=np.arange(int(math.ceil(end/dt))+1)*dt
    if first:
        u=np.clip((t-travel)/1.15,0,1);deg=low*smooth(u);truck=np.array([scheduled(float(a))*.001 for a in deg])
        driveStart=0.
    else:
        u=np.clip((t-wait)/4.8,0,1);deg=122+(low-122)*smooth(u)
        truck=np.interp(3.4*(1-u),times,tb)
        driveStart=end-travel
    th=np.radians(deg)
    ax=np.gradient(np.gradient(truck+.4575*np.cos(th)-.096*np.sin(th),dt),dt)
    az=np.gradient(np.gradient(.501+.4575*np.sin(th)+.096*np.cos(th),dt),dt)
    angacc=np.gradient(np.gradient(th,dt),dt)
    pts=np.array(leg['out']).copy();pts[0]+=np.array(firstOffset)
    elapsed=driveStart
    # Match the outbound cubic segment acceleration in the pickup heading.
    for j,rawD in enumerate(leg['outT']):
        D=travel if len(pts)==2 else rawD
        uu=(t-elapsed)/D;use=(uu>=0)&(uu<1);uu=uu[use]
        yaw=leg['yaw']+(initialYaw-leg['yaw'])*(1-smooth(uu)) if first else np.zeros_like(uu)
        dx,dy=pts[j+1]-pts[j];ax[use]+=(dx*np.cos(yaw)+dy*np.sin(yaw))*(6-12*uu)/D**2
        elapsed+=D
    mod=mj.MjModel.from_xml_string('<mujoco><option gravity="0 0 0" timestep="'+str(dt)+'"/><worldbody><body><joint name="f" axis="0 -1 0" limited="true" range="-92 90" stiffness=".018" damping=".004" frictionloss=".002" solreflimit=".002 1"/><geom type="box" pos="0 0 -.026" size=".0015 .037 .026" mass=".02"/></body></worldbody></mujoco>')
    d=mj.MjData(mod)
    if not first:
        c,s=prev
        d.qpos[0]=c['endFlapRad'];d.qvel[0]=c['endFlapVelocity']
    camangles=np.pi/2*np.clip((deg-102)/18,0,1)
    inertia=.02*((.052**2+.003**2)/12+.026**2);trace=[];err=0.
    for j in range(len(t)-1):
        f=d.qpos[0];om=d.qvel[0];cam=camangles[j];angle=th[j]+f
        mod.jnt_range[0,1]=cam
        push=max(0.,min(.25,8*(cam-f)-.09*om)) if deg[j]>102 else 0.
        d.qfrc_applied[0]=push-.02*.026*(ax[j]*math.cos(angle)+(9.80665+az[j])*math.sin(angle))-inertia*angacc[j]
        err=max(err,f-cam);mj.mj_step(mod,d)
        if j%round(.025/dt)==0:trace.append([t[j],deg[j],float(f),float(om)])
    if first:nose=phase=0.;zones=[0.,0.,0.]
    else:
        c,s=prev;v=c['endBeltSpeed'];decay=.5/.16;nose=v*math.exp(-decay*end)
        phase=c['noseTravelM']+v*(-math.expm1(-decay*end))/decay
        last=s['trace'][-1];tail=end-last['t'];assert tail>=0,(end,last['t'])
        assert abs(last['feedMps'])<8/18
        zones=[last['feedMps']*math.exp(-(.5+18*.01*4*.7/.012)/.18*tail),*[v*math.exp(-.4/.20*tail) for v in last['beltMps']]]
    return dict(flapRad=float(d.qpos[0]),flapVelocityRadS=float(d.qvel[0]),zonesMps=zones,noseSpeedMps=nose,noseTravelM=phase),dict(duration=end,dt=dt,peakStopViolationRad=err,trace=trace,assumption='Unloaded prescribed support with straight-leg translational acceleration; chassis yaw-induced hinge acceleration omitted.')


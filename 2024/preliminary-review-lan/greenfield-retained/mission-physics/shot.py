"""Free torus driven in the selected cassette, then observed on the registered blue field.
SI units; no imposed NOTE pose, launch impulse or assigned exit velocity."""
from pathlib import Path
import sys,json,math,hashlib
from types import SimpleNamespace
import numpy as np,mujoco as mj
here=Path(__file__).resolve().parent
repo=next(d for d in here.parents if (d/'sim/pressure').is_dir());root=repo/'2024/six-speaker-20261118'
field=root/'field-source'/'blue-speaker'
sys.path[:0]=[str(root/'field-query-libs'),str(repo/'sim/pressure')]
from speaker_surfaces import SpeakerSurfaces,digest
from run_speaker_field import field_wrench
from speaker_score_observer import opening_passage,shadow_margin,beam_samples
from wing_speaker_passage import rotation
target=json.loads((field/'opening-and-counter.json').read_bytes());sensor=target['sensorCaseEnds']
def shot(name,angle=125,robotX=1.4,drive=True,dt=.0005,n=48,mu=.78,k=4500,withfield=True):
    a=math.radians(angle); C,S=math.cos(a),math.sin(a);R=np.array([[C,0,-S],[0,1,0],[S,0,C]])
    robotY=float(sys.argv[9]) if len(sys.argv)>9 else 5.5458275; pivot=np.array([robotX+.185,robotY,.501]); q0=[math.cos(a/2),0,-math.sin(a/2),0]
    initial=pivot+R@np.array([.215,0,.0504]);mass=.2413;major=.1524;minor=.0254
    fmt=lambda v:' '.join(str(float(x)) for x in v)
    def box(s,y,h,ds,dy,dh,label):
        return f'<geom name="{label}" type="box" pos="{s} {y} {h}" size="{ds} {dy} {dh}" friction=".12 .001 .0001" contype="2" conaffinity="1"/>'
    stock=box(.0326,0,.0415,.0046,.048,.0225,'backstop')
    for side in (-1,1):
        stock+=box(.32,.1225*side,.022,.289,.0225,.003,f'runner{side}')
        stock+=box(.222,.1315*side,.081,.186,.0865,.001,f'windowRail{side}')
    stock+=box(.048,0,.081,.012,.045,.001,'panelReturn')+box(.358,0,.024,.267,.0225,.001,'centralBelt')
    stock+=f'<geom name="flapBarrel" type="cylinder" pos=".3995 0 .084" size=".003 .037" euler="90 0 0" contype="2" conaffinity="1"/>'
    stock+=box(.4255,0,.084,.026,.037,.0015,'camOpenedFlap')
    bead=''
    for i in range(n):
        w=2*math.pi*i/n;u=2*math.pi*(i+1)/n
        bead+=f'<geom type="capsule" fromto="{major*math.cos(w)} {major*math.sin(w)} 0 {major*math.cos(u)} {major*math.sin(u)} 0" size="{minor}" contype="1" conaffinity="2"/>'
    ix=mass*(major**2/2+5*minor**2/8);iz=mass*(major**2+3*minor**2/4)
    xml=f'<mujoco><option timestep="{dt}" integrator="implicitfast" gravity="0 0 -9.80665"/><default><geom solref=".008 1" solimp=".94 .99 .001"/></default><worldbody><body pos="{fmt(pivot)}" quat="{fmt(q0)}">{stock}</body><body name="note" pos="{fmt(initial)}" quat="{fmt(q0)}"><freejoint/><inertial pos="0 0 0" mass="{mass}" diaginertia="{ix} {ix} {iz}"/>{bead}</body></worldbody></mujoco>'
    model=mj.MjModel.from_xml_string(xml);d=mj.MjData(model);b=mj.mj_name2id(model,mj.mjtObj.mjOBJ_BODY,'note')
    phi=np.arange(n)*2*np.pi/n;beads=major*np.stack([np.cos(phi),np.sin(phi),np.zeros(n)],axis=1)
    # Soft belt surface 6 mm inward of the tangent envelope per side.
    # Resolve normal and tangential wrenches at the deepest bead on each finite rail.
    fargs=SimpleNamespace(stiffness=1.2e6,damping=450.,friction=.45,belt_speed=1.5)
    fieldparts=SpeakerSurfaces(field) if withfield else None
    rows=[];coarse=[];release=None;peak=0.;exit=None;touches=set();maxfloor=0;maxCurrent=0
    ratio=float(sys.argv[10]) if len(sys.argv)>10 else 1.4; limit=float(sys.argv[11]) if len(sys.argv)>11 else 35.
    inertia=float(sys.argv[8]) if len(sys.argv)>8 else .14
    belts=np.zeros(2);bus=11.8-.027*25;peakBelt=0.;minBus=bus
    for step in range(int((7 if drive else .55)/dt)+1):
        mj.mj_step1(model,d);q=d.qpos.copy();world=d.xmat[b].reshape(3,3);center=q[:3];v=d.qvel[:3];omega=world@d.qvel[3:]
        pts=beads@world.T+center;local=(pts-pivot)@R;wr=np.zeros(6);normal=0.;work=0
        currents=[]
        for jj,side in enumerate([-1,1]):
            speed=belts[jj];w=speed/(.018*ratio);I=np.clip((bus-.0202*w)/.115,0,limit) if drive else 0.;Fmotor=I*.0198*.87/(ratio*.018);contactF=0.; currents.append(I*min(1.,(.0202*w+.115*I)/bus))
            nearest=np.column_stack((np.clip(local[:,0],.097,.558),np.full(n,side*.1898),np.clip(local[:,2],.032,.064)))
            diff=local-nearest;ds=np.linalg.norm(diff,axis=1);idx=ds.argmin();dep=(minor+.018)-ds[idx]
            if dep<=0:
                belts[jj]+=dt*(Fmotor-.4*speed)/inertia;continue
            direction=R@(diff[idx]/ds[idx]);site=pts[idx]-direction*minor;arm=site-center;velocity=v+np.cross(omega,arm)
            N=max(0.,k*dep-5*np.dot(velocity,direction));normal+=N
            # Friction-limited contact plus two motor current/voltage limits, starting from rest.
            tangent=R[:,0]-direction*np.dot(R[:,0],direction);tangent/=max(.01,np.linalg.norm(tangent))
            targetV=speed
            slip=targetV-np.dot(velocity,tangent)
            F=np.clip(45*slip,-mu*N,mu*N);F=np.clip(F,-18.,18.) if drive else np.clip(F,-1.,1.)
            force=N*direction+F*tangent;wr[:3]+=force;wr[3:]+=np.cross(arm,force)
            belts[jj]+=dt*(Fmotor-F-.4*speed)/inertia
            work+=abs(F)*targetV;peak=max(peak,abs(F))
        maxCurrent=max(maxCurrent,sum(currents));bus=11.8-.027*(25+sum(currents));minBus=min(minBus,bus);peakBelt=max(peakBelt,max(belts))
        if withfield and center[0]<.8:
            fw,touch,_=field_wrench(fieldparts,pts,center,v,omega,fargs);wr+=fw;touches.update(touch)
        if exit is None and (center-pivot)@R[:,0]>.558 and normal<.01:exit=dict(t=d.time,q=q.tolist(),velocity=v.copy().tolist())
        if release is None and exit and (center-pivot)@R[:,0]>.64:release=dict(t=d.time,q=q.tolist(),v=d.qvel.copy().tolist())
        row=dict(t=d.time,q=q.tolist(),beltMps=belts.tolist(),busV=bus);rows.append(row)
        if step%max(1,round(.012/dt))==0:coarse.append(row)
        maxfloor=max(maxfloor,center[2]);d.xfrc_applied[b]=wr;mj.mj_step2(model,d)
        if center[2]<.17 or (not withfield and center[2]<1 and release and d.qvel[2]<0):break
    if fieldparts:fieldparts.close()
    opening=opening_passage(rows)
    pairs=[[beam_samples(sensor[j]['caseEndCornersM'],sensor[j+1]['caseEndCornersM'])] for j in []]
    first=[None,None];both=None
    if opening and withfield:
        ends={row['key']:np.array(row['caseEndCornersM']) for row in sensor}
        beamgroups=[beam_samples(ends[a],ends[b]) for a,b in target['beamPairs']]
        for row in rows:
            if row['t']<opening['fullyThroughByS']:continue
            qq=np.array(row['q']);marg=[shadow_margin(qq,sm) for sm in beamgroups]
            for j,vv in enumerate(marg):
                if vv>=0 and first[j] is None:first[j]=row['t']
            if min(marg)>=0 and both is None:both=row['t']
    coarse.append(rows[-1]) if coarse[-1]['t']!=rows[-1]['t'] else None
    result=dict(name=name,dt=dt,n=n,mu=mu,k=k,overdrive=ratio,motorLimitA=limit,beltEquivalentKg=inertia,angle=angle,robotX=robotX,pivot=pivot.tolist(),powered=drive,exit=exit,release=release,opening=opening,beams=first,both=both,peakTractionN=peak,estimatedPeakDriveA=maxCurrent,minBusV=minBus,peakBeltMps=peakBelt,peakZ=maxfloor,contacts=sorted(touches),trace=coarse)
    (here/(name+'.json')).write_text(json.dumps(result,separators=(',',':')));print(name,exit and exit['velocity'],opening,first,both,maxfloor,flush=True)
    return result
if __name__=='__main__':
    shot(sys.argv[1],float(sys.argv[2]),float(sys.argv[3]),bool(int(sys.argv[4])),float(sys.argv[5]),int(sys.argv[6]),withfield=bool(int(sys.argv[7])))


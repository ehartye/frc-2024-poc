"""Run the Profile A contact model with a finite-speed, current-limited intake nose.
Loads from the solved NOTE/roller and NOTE/pad contact wrenches feed the next motor step.
"""
from pathlib import Path
import sys,math as m,json,numpy as np,mujoco as mj
out=Path(__file__).resolve().parent
p=out.parent/'pickup-lip';sys.path.insert(0,str(p))
from gap import mat,quat,clearance
label,n,stop,err,height,offset,grip,cap=sys.argv[1:];n=int(n)
stop,err,height,offset,grip,cap=map(float,(stop,err,height,offset,grip,cap))
g=json.loads((p/'layout.json').read_bytes())
deploy=g['pickAngle']+stop+err;Z=(g['pickPivot'][2]+height)/1000
floorMm=min(1000*Z+x*m.sin(m.radians(deploy))+z*m.cos(m.radians(deploy)) for x,z in g['profiles']['Replaceable pickup lip'])
loc=out/(label+'.json')
base=dict(label=label,pitchDegrees=deploy,addedHeightMm=height,offsetM=offset,noseMu=grip,phaseCurrentLimitA=cap,capSegments=n,minimumBladeHeightMm=floorMm)
if floorMm<1:
    base.update(passScreen=False,skipReason='Fails 1 mm rigid-floor geometric minimum');loc.write_text(json.dumps(base));print(label,base);sys.exit()
# Reuse the contact geometry, passive flap, capped side-belt law and 6-DOF free body.
parent=(p/'pickup_trial.py').read_text()
header=parent[parent.index('src=(p.parent'):parent.index("# Keep the original")]
exec(header)
def sub(a,b):
    global src
    assert src.count(a)==1,a
    src=src.replace(a,b)
sub('speed=.84 if enabled else 0','speed=0.; commanded=.84 if enabled else 0.; travel=0.;motorLog=[]; peakI=0.;minLoaded=10.;regen=0.')
sub('round(2.0/dt)','round(2.82/dt)')
sub('.7866309 {offset}', '.8666309 {offset}')
sub('mass=".07" contype="2"',f'mass=".07" priority="1" friction="{grip} .003 .001" contype="2"')
sub('mass=".16" contype="2"',f'mass=".16" priority="1" friction="{grip} .003 .001" contype="2"')
sub('track(i*spacing+speed*t)','track(i*spacing+travel)')
sub('for j,w in rj:d.qvel[model.jnt_dofadr[j]]=w','for j,w in rj:d.qvel[model.jnt_dofadr[j]]=-speed/.012')
sub('45*(-speed-np.dot(vel,tangent))','45*(-commanded-np.dot(vel,tangent))')
sub('        mj.mj_step2(model,d)', '''        mj.mj_step2(model,d)
        load=0.;tmp=np.zeros(6)
        for j,cnt in enumerate(d.contact):
            aa,bb=cnt.geom1,cnt.geom2
            if model.geom_bodyid[aa]==bi:oth=bb;sign=1.
            elif model.geom_bodyid[bb]==bi:oth=aa;sign=-1.
            else:continue
            nm=model.geom(oth).name
            if not(nm.startswith('pad') or nm.startswith('nose')):continue
            mj.mj_contactForce(model,d,j,tmp);axes=cnt.frame.reshape(3,3)
            force=sign*(axes.T@tmp[:3]);moment=sign*(axes.T@tmp[3:])
            if nm.startswith('pad'):
                ix=int(nm[3:]);vx,vz,omega=vals[ix,3:]
                vper=R@np.array([vx,0,vz]);angular=np.array([0,-omega,0])
            else:
                vper=np.zeros(3);angular=np.array([0,-1/.012,0])
            vper+=np.cross(angular,cnt.pos-d.xpos[model.geom_bodyid[oth]])
            load+=np.dot(force,vper)+np.dot(moment,angular)
        # Sign-preserving current-limited speed regulator; 4:1 assumed reduction, 12 mm pitch radius.
        # 8.5 V at the controller, 0.01 Nm/A, 0.01 V s/rad, 0.35 ohm, 70% drive efficiency.
        amps=max(0.,min(cap,22.*(commanded-speed),(8.5-.01*4*speed/.012)/.35)) if enabled else 0.
        motor=amps*.01*4*.70/.012
        peakI=max(peakI,amps)
        if 'center belt' in inj or 'nose roller' in inj:minLoaded=min(speed,minLoaded)
        # 0.16 kg equivalent translational inertia; signed load, no speed imposed on the NOTE.
        ds=(motor+load-.5*speed)*dt/.16
        travel+=speed*dt;speed+=ds
        if step%round(.01/dt)==0:motorLog.append([round(t,5),speed,load,amps])
        if not np.isfinite(speed) or abs(speed)>5:raise ValueError(('unstable motor step',speed))
''')
sub('return dict(lip=LIP','return dict(minimumContactSpeedMps=minLoaded,peakPhaseA=peakI,motorTrace=motorLog,lip=LIP')
sub('pos=d.qpos[qi:qi+3].copy()', '''pos=d.qpos[qi:qi+3].copy(); fm=mat(-angle,X*1000,0,Z*1000)@mat(0,457.5,0,96)@mat(m.degrees(d.qpos[model.jnt_qposadr[jid('flap')]])); qp=quat(d.qpos[qi+3:qi+7],(pos-[advance,0,0])*1000);checks.append(clearance(qp,fm))''')
beater=False;press=False;shoe=0;phase=0;topGeom=(.624,.075,.025);angle=-deploy
X=g['pickPivot'][0]/1000;C=m.cos(m.radians(angle));S=-m.sin(m.radians(angle))
LIP=True;LIP_MU=.15;stroke=.44;checks=[]
exec(src);r=run(n,.024/n,cap>0,False,offset,.75);r.update(base)
r['flapClearanceMm']=np.min(checks,axis=0).tolist()
r['passScreen']=bool(r['seated'] and min(r['flapClearanceMm'])>0)
loc.write_text(json.dumps(r,separators=(',',':')))
print(label,r['passScreen'],r['seated'],floorMm,r['minimumContactSpeedMps'],r['flapClearanceMm'])


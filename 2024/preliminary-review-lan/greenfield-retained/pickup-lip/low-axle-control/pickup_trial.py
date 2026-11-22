"""Source-bound comparison on a free NOTE at the actual deployed pitch. SI units."""
from pathlib import Path
import math as m,hashlib,json,sys
import numpy as np
import mujoco as mj
p=Path(__file__).resolve().parent
g=json.loads((p/'layout.json').read_bytes())
src=(p.parent/'pickup_physics.py').read_text(encoding='utf8')
src=src[src.index('def run('):src.index("if __name__")]
def replace(a,b):
    global src
    assert a in src,a
    src=src.replace(a,b)
replace("fixed+=block(.0326", """assets=''
    for side in [-.1315,.1315]:fixed+=block(.222,side,.081,.186,.0865,.001)
    fixed+=block(.048,0,.081,.012,.045,.001)
    fixed+=body('.4575 0 .084','<joint name="flap" type="hinge" axis="0 -1 0" limited="true" range="-92 0" stiffness=".018" damping=".004" frictionloss=".002"/>'+geom('box','pos="0 0 -.026" size=".0015 .037 .026"','name="flapLeaf" margin=".003" solref=".003 1" mass=".02" contype="2" conaffinity="1"'))
    fixed+=geom('cylinder','name="sleeve" pos=".4575 0 .084" size=".003 .037" quat=".70710678 .70710678 0 0"','margin=".003" contype="2" conaffinity="1"')
    fixed+=geom('cylinder','name="axle" pos=".4575 0 .084" size=".002 .226" quat=".70710678 .70710678 0 0"','margin=".003" contype="2" conaffinity="1"')
    if LIP:
        for nm,width,tag in [('Replaceable pickup lip',.434,'wear'),('Pickup lip crossbar',.452,'back')]:
            # Conforming convex strips share the exact profile extrema, including the bevel.
            prof=np.asarray(g['profiles'][nm])/1000
            # Triangulate the plane polygon so the engine cannot fill a concave relief.
            uv=prof.tolist();inds=list(range(len(uv)));tris=[]
            def cross(a,b,c):return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
            if sum(uv[j][0]*uv[(j+1)%len(uv)][1]-uv[(j+1)%len(uv)][0]*uv[j][1] for j in inds)<0:inds.reverse()
            while len(inds)>3:
                for jj,vv in enumerate(inds):
                    tri=[inds[jj-1],vv,inds[(jj+1)%len(inds)]];av,bv,cv=[uv[j] for j in tri]
                    if cross(av,bv,cv)<=1e-12:continue
                    if any(min(cross(av,bv,uv[k]),cross(bv,cv,uv[k]),cross(cv,av,uv[k]))>=-1e-11 for k in inds if k not in tri):continue
                    tris.append(tri);inds.pop(jj);break
                else:raise ValueError('invalid lip cross-section')
            tris.append(inds)
            for j,tri in enumerate(tris):
                verts=' '.join(str(val) for yy in [-width/2,width/2] for k in tri for val in [uv[k][0],yy,uv[k][1]])
                key=tag+str(j);assets+=f'<mesh name="{key}" vertex="{verts}"/>'
                fixed+=geom('mesh',f'mesh="{key}" name="{key}"',f'priority="1" friction="{LIP_MU} .001 .0001" contype="2" conaffinity="1"')
    fixed+=block(.0326""")
replace('<worldbody><geom type="plane"', '<asset>{assets}</asset><worldbody><geom name="floor" type="plane"')
replace("shaft=geom('cylinder',f'size=",'''shaft=geom('cylinder',f'name="{name}" size=''')
replace("geom('box',f'size=\"{spacing","geom('box',f'name=\"pad{i}\" size=\"{spacing")
replace('.25/n','.2413/n')
replace("qi=model.jnt_qposadr[jid('piece')]","""bi=model.jnt_bodyid[jid('piece')];angles=np.arange(n)*2*np.pi/n;beads=.1524*np.stack([np.cos(angles),np.sin(angles),np.zeros(n)],axis=1)
    v0=model.jnt_dofadr[jid('piece')]
    touched={};floorLast=0.;peakRise=0.;takeover=None;maxLiftAtLip=0.;contacts={};maxF=0.
    qi=model.jnt_qposadr[jid('piece')]""")
replace('mj.mj_step(model,d)','''mj.mj_step1(model,d);q=d.qpos[qi:qi+7];wr=np.zeros(6);R=np.array([[C,0,-S],[0,1,0],[S,0,C]])
        world=d.xmat[bi].reshape(3,3);pts=beads@world.T+q[:3];local=(pts-[X+advance,0,Z])@R
        # Actual finite soft belt reaches and friction law from the mounted P4 shot.
        for side in (-1,1):
            nearest=np.column_stack((np.clip(local[:,0],.097,.558),np.full(n,side*.1898),np.clip(local[:,2],.032,.064)))
            diff=local-nearest;radial=np.linalg.norm(diff,axis=1);i=radial.argmin();depth=.0434-radial[i]
            if depth<=0:continue
            normal=R@(diff[i]/radial[i]);arm=pts[i]-q[:3]-normal*.0254
            vel=d.qvel[v0:v0+3]+np.cross(world@d.qvel[v0+3:v0+6],arm)-[d.qvel[model.jnt_dofadr[drive]],0,0]
            N=max(0.,4500*depth-5*np.dot(vel,normal));tangent=R[:,0]-normal*np.dot(R[:,0],normal);tangent/=max(.01,np.linalg.norm(tangent))
            F=np.clip(45*(-speed-np.dot(vel,tangent)),-min(.78*N,18.),min(.78*N,18.)) if enabled else 0.
            force=normal*N+tangent*F;wr[:3]+=force;wr[3:]+=np.cross(arm,force);maxF=max(maxF,abs(F))
            if abs(F)>1 and takeover is None:takeover=t
        d.xfrc_applied[bi]=wr
        inj=set()
        for ct in d.contact:
            for gg in (ct.geom1,ct.geom2):
                if model.geom_bodyid[gg]==bi:continue
                nm=model.geom(gg).name
                if nm:
                    label='lip' if nm.startswith('wear') else 'backer' if nm.startswith('back') else 'center belt' if nm.startswith('pad') else 'nose roller' if nm.startswith('nose') else nm
                    touched.setdefault(label,round(t,4));inj.add(label)
        if 'floor' in inj:floorLast=t
        if 'lip' in inj:maxLiftAtLip=max(maxLiftAtLip,q[2]-.0254)
        peakRise=max(peakRise,q[2]-.0254)
        mj.mj_step2(model,d)''')
replace("round(h,6)])","round(h,6),*['+'.join(sorted(inj))],round(d.qpos[model.jnt_qposadr[jid('flap')]],5)])")
replace('return dict(n=n,dt=dt,enabled=',"return dict(lip=LIP,lipFriction=LIP_MU,approachM=stroke,pickAngle=-angle,firstContactS=touched,maxRiseMm=1000*peakRise,liftDuringLipMm=1000*maxLiftAtLip,sideBeltTakeoverS=takeover,lastCarpetContactS=floorLast,maxPoweredSideForceN=maxF,n=n,dt=dt,enabled=")
# Keep the original 0.25 m/s approach. Only its finite distance is a trial input.
beater=False;press=False;shoe=0;phase=0;topGeom=(.624,.075,.025)
angle=-g['pickAngle'];X,_,Z=np.array(g['pickPivot'])/1000;C=m.cos(m.radians(angle));S=-m.sin(m.radians(angle))
LIP=bool(int(sys.argv[1]));enabled=bool(int(sys.argv[2]));n=int(sys.argv[3]);stroke=float(sys.argv[4]);LIP_MU=.15
src=src.replace('round(2.0/dt)','round(2.5/dt)');from gap import mat,quat,clearance
checks=[]
src=src.replace('pos=d.qpos[qi:qi+3].copy()', '''pos=d.qpos[qi:qi+3].copy(); fm=mat(-angle,X*1000,0,Z*1000)@mat(0,457.5,0,84)@mat(m.degrees(d.qpos[model.jnt_qposadr[jid('flap')]])); qp=quat(d.qpos[qi+3:qi+7],(pos-[advance,0,0])*1000);checks.append(clearance(qp,fm))''')
exec(src);r=run(n,.024/n,enabled,False,float(sys.argv[5]) if len(sys.argv)>5 else 0)
fn='trial-'+str(int(LIP))+'-'+str(int(enabled))+'-'+str(n)+'-'+str(stroke)+'-'+str(r['offset'])+'.json'
r['minimumSolidClearanceMm']=np.min(checks,axis=0).tolist();r['checkedPoses']=len(checks)
(p/fn).write_text(json.dumps(r,separators=(',',':')))
print({k:v for k,v in r.items() if k!='trajectory'})


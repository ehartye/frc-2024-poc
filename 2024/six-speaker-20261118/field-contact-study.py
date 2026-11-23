"""Private field contact trial using registered STEP triangulations and attained NOTE release."""
import argparse
import hashlib
import json
import sys
from pathlib import Path

import igl
import mujoco
import numpy as np

from wing_speaker_passage import rotation, MAJOR, TUBE

def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()

def run(root, output, dt, count, belt, horizon, stiffness, damping):
    folder=root/'field-source/blue-speaker'
    manifest=json.loads((folder/'panels.json').read_bytes())
    mesh=folder/'named-panels.npz'
    assert digest(mesh)==manifest['meshSha256']
    arrays=np.load(mesh)
    parts=manifest['parts']
    bounds=np.array([p['blueFieldBoundsM'] for p in parts])
    trees={}
    def project(index, points):
        key=parts[index]['key']
        if key not in trees:
            vs=np.ascontiguousarray(arrays[key+'_v'],dtype=float)
            fs=np.ascontiguousarray(arrays[key+'_t'],dtype=np.int64)
            tree=igl.AABB()
            tree.init(vs,fs)
            trees[key]=(tree,vs,fs)
        tree,vs,fs=trees[key]
        ds,_,nearest=tree.squared_distance(vs,fs,points)
        return np.sqrt(ds),nearest

    original=root/'result-refined'
    receipt=json.loads((original/'result.json').read_bytes())
    assert receipt['outcome']=='UPWARD_FREE_RELEASE' and digest(original/'trace.jsonl')==receipt['traceSha256']
    with (original/'trace.jsonl').open('rb') as stream:
        for line in stream: pass
    row=json.loads(line)
    assert row['beltNormalN']==row['wheelNormalN']==0 and abs(row['time']-receipt['end'])<1e-8
    basis=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
    mass=.235
    ix=mass*(MAJOR**2/2+5*TUBE**2/8)
    iz=mass*(MAJOR**2+3*TUBE**2/4)
    model=mujoco.MjModel.from_xml_string(f'<mujoco><option timestep="{dt}" gravity="0 0 -9.80665" integrator="implicitfast"/><worldbody><body><freejoint/><inertial pos="0 0 0" mass="{mass}" diaginertia="{ix} {ix} {iz}"/></body></worldbody></mujoco>')
    data=mujoco.MjData(model)
    data.qpos[:3]=np.array([114*.0254,161.625*.0254,0.])+basis@(np.array(row['q'][-7:-4])-[.658,0,0])
    mujoco.mju_mat2Quat(data.qpos[3:],(basis@rotation(row['q'][-4:])).ravel())
    data.qvel[:3]=basis@row['v'][-6:-3]
    data.qvel[3:]=row['v'][-3:]
    angles=2*np.pi*np.arange(count)/count
    local=MAJOR*np.stack((np.cos(angles),np.sin(angles),np.zeros(count)),axis=1)
    first={}
    worst=0.
    output.mkdir(exist_ok=False)
    with (output/'trace.jsonl').open('x') as log:
        for i in range(round(horizon/dt)+1):
            mujoco.mj_step1(model,data)
            mat=data.xmat[1].reshape(3,3)
            arm=local@mat.T
            pos=data.qpos[:3]
            pts=pos+arm
            low,high=pts.min(axis=0)-TUBE,pts.max(axis=0)+TUBE
            nearby=np.flatnonzero(((high>=bounds[:,0])&(low<=bounds[:,1])).all(axis=1))
            omega=mat@data.qvel[3:]
            wrench=np.zeros(6)
            touch={}
            for index in nearby:
                # Individual node AABB cull is conservative for a sphere of radius TUBE.
                active=np.flatnonzero(((pts+TUBE>=bounds[index,0])&(pts-TUBE<=bounds[index,1])).all(axis=1))
                if not len(active): continue
                dist,near=project(index,pts[active])
                hit=dist<TUBE
                if not np.any(hit): continue
                idx=active[hit]
                depth=TUBE-dist[hit]
                normal=(pts[idx]-near[hit])/dist[hit,None]
                point_arm=arm[idx]-normal*(TUBE-.5*depth[:,None])
                rel=data.qvel[:3]+np.cross(omega,point_arm)
                key=parts[index]['key']
                if key in ('p49','p50'):
                    # Counter top and return flights have opposite transport velocities.
                    # Rotation near belt ends is around the field y axis.
                    rel-=belt*np.cross(normal,[0.,1.,0.])
                vn=(rel*normal).sum(axis=1)
                fn=np.maximum(0.,stiffness*depth-damping*vn)/count
                tangent=rel-vn[:,None]*normal
                norm=np.linalg.norm(tangent,axis=1)
                forces=fn[:,None]*(normal-.45*tangent/np.maximum(.05,norm[:,None]))
                wrench[:3]+=forces.sum(axis=0)
                wrench[3:]+=np.cross(point_arm,forces).sum(axis=0)
                peak=float(max(depth))
                worst=max(worst,peak)
                if peak>=TUBE*.85: raise ValueError(('centerline may cross shell', key, peak))
                if fn.sum()>0:
                    touch[key]=float(fn.sum())
                    first.setdefault(key,round(i*dt,7))
            if i%max(1,round(.001/dt))==0:
                log.write(json.dumps(dict(t=data.time,q=data.qpos.tolist(),v=data.qvel.tolist(),force=touch))+'\n')
            data.xfrc_applied[1]=wrench
            if i==round(horizon/dt) or pos[2]<.2:break
            mujoco.mj_step2(model,data)
    out=dict(firstPositiveContactS=first,maximumIndentM=worst,endS=data.time,positionM=pos.tolist(),
             velocity=data.qvel.tolist(),dt=dt,azimuths=count,beltSpeed= belt,stiffness=stiffness,damping=damping,
             releaseTimeS=row['time'],sourceTrace=receipt['traceSha256'],mesh=manifest['meshSha256'])
    (output/'summary.json').write_text(json.dumps(out,indent=2))
    print(json.dumps(out),flush=True)

p=argparse.ArgumentParser()
p.add_argument('root',type=Path)
p.add_argument('output',type=Path)
p.add_argument('--dt',type=float,default=.0002)
p.add_argument('--count',type=int,default=96)
p.add_argument('--belt',type=float,default=0.)
p.add_argument('--horizon',type=float,default=2.)
p.add_argument('--stiffness',type=float,default=1.2e6)
p.add_argument('--damping',type=float,default=450.)
run(**vars(p.parse_args()))

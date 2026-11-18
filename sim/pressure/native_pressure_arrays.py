"""Vectorized evaluation of the declared finite-area law on bound native belts.

This is an additive numerical implementation. The scalar producer is retained
for force, potential, domain and trajectory comparison. No grid reduction.
"""
import math
import numpy as np
import finite_area as a
import pressure_tiled_geometry as tiles

def prepare(config,g,*,note_center,note_basis=a.I,cradle_origin=(0.,0.,0.),cradle_basis=a.I,resolution=(1024,1024),k_area=1e7):
    center=np.array(a.vec(note_center));origin=np.array(a.vec(cradle_origin));nb=np.array(a.basis(note_basis));cb=np.array(a.basis(cradle_basis));major=config['note']['majorRadius']/1000;tube=config['note']['tubeRadius']/1000
    if k_area<=0:raise ValueError('positive pressure density')
    next(a.material_points(major,tube,*resolution))
    if max(np.max(np.abs(center)),np.max(np.abs(origin)))>1e6:raise a.DomainError('ARRAY_GEOMETRY_COORDINATE_DOMAIN')
    relative=cb.T@(center-origin);transform=cb.T@nb;extent=np.array([major*math.hypot(row[0],row[1])+tube*math.hypot(*row) for row in transform]);pad=max(1e-12,64*np.finfo(float).eps*(1+max(np.max(np.abs(center)),np.max(np.abs(origin)),max(extent))))
    boxes=[([g['rearX']/1000-g['outerRadius']/1000,(side*g['axisYAbs']-g['outerRadius'])/1000,-g['halfHeight']/1000],[(g['frontX']+g['outerRadius'])/1000,(side*g['axisYAbs']+g['outerRadius'])/1000,g['halfHeight']/1000]) for side in [-1,1]]
    if all(np.any(relative+extent+pad<lo) or np.any(relative-extent-pad>hi) for lo,hi in boxes):grid=np.empty((0,4))
    else:grid=tiles.candidate_grid(major,tube,resolution,transform.tolist(),relative.tolist(),boxes,pad)
    lever=grid[:,:3]@nb.T;world=lever+center;arm=world-origin;local=arm@cb;p=local*1000;indices=[];normals=[];tangents=[];forces=[];areas=[];depths=[];motors=[];features=[]
    for side in [-1,1]:
        dx=p[:,0]-np.clip(p[:,0],g['rearX'],g['frontX']);dy=p[:,1]-side*g['axisYAbs'];rad=np.hypot(dx,dy);radial=rad-g['outerRadius'];inner=g['innerRadius']-rad;height=np.abs(p[:,2])-g['halfHeight'];planar=np.maximum(radial,inner);sdf=np.hypot(np.maximum(planar,0),np.maximum(height,0))+np.minimum(np.maximum(planar,height),0);idx=np.flatnonzero(sdf<0)
        if len(idx)==0:continue
        if np.any(rad[idx]==0):raise a.DomainError('UNSUPPORTED_CAPSULE_AXIS')
        if np.any((inner[idx]>=radial[idx])|(side*dy[idx]>=0)):raise a.DomainError('UNSUPPORTED_DEEP_OR_CAVITY_CONTACT')
        if np.any(np.abs(radial[idx]-height[idx])<1e-9):raise a.DomainError('UNSUPPORTED_SDF_GRADIENT_TIE')
        normal=np.column_stack([dx[idx]/rad[idx],dy[idx]/rad[idx],np.zeros(len(idx))]);cap=radial[idx]<=height[idx];normal[cap]=np.column_stack([np.zeros(sum(cap)),np.zeros(sum(cap)),np.sign(p[idx[cap],2])]);tangent=np.column_stack([side*dy[idx]/rad[idx],-side*dx[idx]/rad[idx],np.zeros(len(idx))]);normal=normal@cb.T;tangent=tangent@cb.T
        depth=-sdf[idx]/1000
        if np.any(depth>g['treadThicknessMm']/1000+1e-12):raise a.DomainError('UNSUPPORTED_SUBSTRATE_CONTACT')
        indices.extend(idx.tolist());normals.append(normal);tangents.append(tangent);forces.append(k_area*depth*grid[idx,3]);areas.append(grid[idx,3]);depths.append(depth);motors.extend([0 if side==-1 else 1]*len(idx));features.extend(['finite-height-cap' if cap[i] else 'outer-straight' if g['rearX']<=p[j,0]<=g['frontX'] else 'outer-wrap' for i,j in enumerate(idx)])
    count=len(indices);idx=np.array(indices,dtype=int);motor=np.array(motors,dtype=int);normal=np.concatenate(normals) if count else np.empty((0,3));tangent=np.concatenate(tangents) if count else np.empty((0,3));force=np.concatenate(forces) if count else np.empty(0);area=np.concatenate(areas) if count else np.empty(0);depth=np.concatenate(depths) if count else np.empty(0);side_rows=[]
    for i in range(2):
        mask=motor==i
        if any(mask):side_rows.append(dict(motor=i,normalMagnitudeN=float(np.sum(force[mask])),activeSamples=int(sum(mask)),activeAreaM2=float(np.sum(area[mask]))))
    return dict(contacts=count,normalWorld=normal,tangentWorld=tangent,normalN=force,noteLeverWorldM=lever[idx],cradleLeverWorldM=arm[idx],motor=motor,potentialJ=float(np.sum(.5*k_area*depth*depth*area)),features={name:features.count(name) for name in set(features)},sideContacts=side_rows,maximumPenetrationM=float(max(depth,default=0)),resolution=list(resolution))

def rates(p,*,note_velocity,note_omega,cradle_velocity,cradle_omega,rotor_omega,pitch_radius_m,mu,slip_scale):
    nv=np.array(a.vec(note_velocity));nw=np.array(a.vec(note_omega));cv=np.array(a.vec(cradle_velocity));cw=np.array(a.vec(cradle_omega));rotor=np.array(rotor_omega);radius=float(pitch_radius_m)
    if not np.isfinite(rotor).all() or rotor.shape!=(2,) or radius<=0 or mu<0 or slip_scale<=0:raise ValueError('array contact rate domain')
    n=p['normalWorld'];t=p['tangentWorld'];base=cv+np.cross(cw,p['cradleLeverWorldM']);body=nv+np.cross(nw,p['noteLeverWorldM']);belt=base+t*(radius*rotor[p['motor']])[:,None];diff=belt-body;slip=diff-n*np.sum(diff*n,axis=1)[:,None];speed=np.linalg.norm(slip,axis=1);factor=np.divide(mu*p['normalN']*np.tanh(speed/slip_scale),speed,out=np.zeros_like(speed),where=speed!=0);ff=slip*factor[:,None];normal=n*p['normalN'][:,None];f=normal+ff;force=np.sum(f,axis=0);moment=np.sum(np.cross(p['noteLeverWorldM'],f),axis=0);reaction=-force;reactionmoment=np.sum(np.cross(p['cradleLeverWorldM'],-f),axis=0);axis_force=np.sum(ff*t,axis=1);side=[float(np.sum(axis_force[p['motor']==i])) for i in range(2)];torque=-radius*np.array(side);heat=float(np.sum(ff*slip));normalwork=float(np.sum(normal*(body-base)));bodypower=float(force@nv+moment@nw);basepower=float(reaction@cv+reactionmoment@cw);rotorpower=float(torque@rotor)
    return dict(noteForceWorldN=force.tolist(),noteMomentWorldNm=moment.tolist(),cradleReactionWorldN=reaction.tolist(),cradleMomentWorldNm=reactionmoment.tolist(),rotorContactTorqueNm=torque.tolist(),sideBeltAxisTractionN=side,slipHeatW=heat,workResidualW=bodypower+basepower+rotorpower-normalwork+heat)

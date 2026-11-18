"""Finite grip and floor wrenches for a free toroidal NOTE in three dimensions."""
import numpy as np

from native_pressure_lift import LiftDomainError


def belt_wrenches(p, *, note_v, note_w, cradle_v, cradle_w, belt_w, cradle_basis,
                  forward, shear_m, motor_radius, mu, slip_scale, half_stiffness=1250.):
    """Four axial shear regions, split by side and fore/aft from the NOTE center.

    Two symmetric side regions in parallel recover the braked experiment's
    2500 N/m fore/aft stiffness. All vectors and moments are in world axes.
    """
    normal, tangent = p["normalWorld"], p["tangentWorld"]
    load, sides = p["normalN"], p["motor"]
    arm, lever = p["cradleLeverWorldM"], p["noteLeverWorldM"]
    axis = cradle_basis[:,2]
    base = cradle_v + np.cross(cradle_w,arm)
    body = note_v + np.cross(note_w,lever)
    slip = base - body + tangent*(motor_radius*belt_w[sides])[:,None]
    slip -= normal*np.sum(normal*slip,axis=1)[:,None]
    speed = np.linalg.norm(slip,axis=1)
    direction = np.divide(slip,speed[:,None],out=np.zeros_like(slip),where=speed[:,None]>0)
    elastic = np.zeros_like(slip)
    rate = np.zeros(4)
    wall = np.abs(normal@axis)<1e-8
    region_id = 2*sides+(lever@forward>=0).astype(int)
    for i in range(4):
        mask = wall & (region_id==i)
        weight = load[mask]
        force = half_stiffness*shear_m[i]
        if abs(force)>mu*weight.sum()+1e-12:
            raise LiftDomainError("AXIAL_TREAD_YIELD")
        if weight.size:
            weight = weight/weight.sum()
            rate[i] = weight@(slip[mask]@axis)
            elastic[mask] = (weight*force)[:,None]*axis
    projection = np.sum(elastic*direction,axis=1)
    cap = mu*load
    budget = np.sqrt(np.maximum(0.,cap**2-np.sum(elastic**2,axis=1)+projection**2))-projection
    kinetic = direction*(budget*np.tanh(speed/slip_scale))[:,None]
    friction = elastic+kinetic
    if np.any(np.linalg.norm(friction,axis=1)>cap+1e-12):
        raise LiftDomainError("FRICTION_CONE_VIOLATION")
    contact = normal*load[:,None]+friction
    force = contact.sum(axis=0)
    note_moment = np.cross(lever,contact).sum(axis=0)
    cradle_moment = np.cross(arm,-contact).sum(axis=0)
    rotor = -motor_radius*np.bincount(sides,weights=np.sum(friction*tangent,axis=1),minlength=2)
    heat = np.sum(kinetic*slip)
    power = (force@(note_v-cradle_v)+note_moment@note_w+cradle_moment@cradle_w+rotor@belt_w)
    normal_power = np.sum(normal*load[:,None]*(body-base))
    residual = power-normal_power+heat+half_stiffness*shear_m@rate
    if abs(residual)>1e-8 or heat< -1e-12:
        raise LiftDomainError("THREE_DIMENSIONAL_CONTACT_WORK")
    return dict(force=force,noteMoment=note_moment,cradleMoment=cradle_moment,rotor=rotor,
                shearRate=rate,normal=np.bincount(sides,weights=load,minlength=2),
                heat=heat,potential=p["potentialJ"]+.5*half_stiffness*(shear_m@shear_m),
                powerResidual=residual)


def floor_wrench(center, rotation, velocity, omega, ring, tube, contract, friction, slip_scale):
    lever=ring@rotation.T
    lever[:,2]-=tube
    depth=np.maximum(0.,-center[2]-lever[:,2])
    v=velocity+np.cross(omega,lever)
    k=contract["floorStiffnessNpm"]/len(ring)
    b=contract["floorDampingNspm"]/len(ring)
    compression=np.where(depth>0,np.maximum(0.,-v[:,2]),0.)
    normal=k*depth+b*compression
    speed=np.linalg.norm(v[:,:2],axis=1)
    factor=np.divide(-friction*normal*np.tanh(speed/slip_scale),speed,
                     out=np.zeros_like(speed),where=speed>0)
    forces=np.column_stack((v[:,:2]*factor[:,None],normal))
    return (forces.sum(axis=0),np.cross(lever,forces).sum(axis=0),
            .5*k*(depth@depth),b*(compression@compression)-np.sum(forces[:,:2]*v[:,:2]))


"""XZ dynamics and finite-effort feedback for the native floor-to-handoff route."""
import math

import numpy as np

from axial_shear_lift import ShearLift
from native_pressure_lift import LiftDomainError
from native_pressure_arrays import prepare
from patch_registration import rotation


class RouteLift(ShearLift):
    def geometry(self, s, cache):
        d, n = s["chassis"], s["note"]
        if max(abs(d[k]) for k in ("heading", "vy", "yaw_rate")) > 1e-8 or max(
                abs(n["yaw"]), abs(n["v"][1]), abs(n["spin"])) > 1e-8:
            raise LiftDomainError("NONPLANAR_ROUTE_STATE")
        z, angle = s["zPivot"], s["theta"]
        x = n["p"][0] - d["x"]
        origin_x = self.cad["stageCenterX"] / 1000
        dx, dz = x-origin_x, s["noteZ"]-z
        u = math.cos(angle)*dx + math.sin(angle)*dz
        h = -math.sin(angle)*dx + math.cos(angle)*dz
        if not (-.02 < u < .025 and abs(h) < .015 and abs(s["notePitch"]-angle) < .1):
            raise LiftDomainError("LEFT_REGISTERED_GRIP_CORRIDOR")
        if not (self.z0-.002 < z < self.cad["handoffCenterZ"]/1000+.015
                and -.02 < angle < math.radians(self.cad["pitchDeg"])+.035):
            raise LiftDomainError("OUTSIDE_ROUTE_WORKSPACE")
        key = (d["x"], d["y"], *n["p"], z, angle, s["noteZ"], s["notePitch"])
        if key not in cache:
            cache[key] = prepare(self.cad, self.binding["effectiveGeometryMm"],
                                 note_center=[*n["p"], s["noteZ"]], note_basis=rotation(n["yaw"], s["notePitch"]),
                                 cradle_origin=[d["x"]+origin_x,d["y"],z], cradle_basis=rotation(d["heading"],angle),
                                 resolution=tuple(self.c["resolution"]), k_area=1e7)
        return cache[key]


def ramp(t, duration):
    """Quintic position fraction and its first two derivatives."""
    u = min(1., max(0., t / duration))
    return (u**3*(10+u*(-15+6*u)), 30*u*u*(1-u)**2/duration,
            60*u*(1-u)*(1-2*u)/duration**2)


def target(model, tick):
    t = tick / 100
    initial = model.z0
    lift = .0022
    pitch = math.radians(model.cad["pitchDeg"])
    radius = model.cad["note"]["majorRadius"] / 1000
    if tick < 80:
        a,b,c = ramp(t, .5)
        return (initial+lift*a, lift*b, lift*c), (0.,0.,0.)
    if tick < 300:
        a,b,c = (pitch*value for value in ramp(t-.8, 2.2))
        z = initial+lift+radius*math.sin(a)
        return (z, radius*math.cos(a)*b, radius*(math.cos(a)*c-math.sin(a)*b*b)), (a,b,c)
    start = initial+lift+radius*math.sin(pitch)
    travel = model.cad["handoffCenterZ"]/1000-start
    a,b,c = ramp(t-3., 2.)
    return (start+travel*a, travel*b, travel*c), (pitch,0.,0.)


def controls(m, tick, bus):
    s = m.s
    d, n = s["chassis"], s["note"]
    angle = s["theta"]
    cz, sz = math.cos(angle), math.sin(angle)
    axis = np.array([cz,0.,sz])
    zref, aref = target(m,tick)
    q = [d["x"],s["zPivot"],angle]
    vel = [d["vx"],s["vPivot"],s["thetaRate"]]
    terms = m.act.terms(q,vel)
    acc = [0., zref[2]+225*(zref[0]-q[1])+30*(zref[1]-vel[1]),
           aref[2]+225*(aref[0]-q[2])+30*(aref[1]-vel[2])]
    if 80 <= tick < 300:
        # Follow the lowest point of the finite ring while passing the bumper.
        # The NOTE can lag the cradle under load, especially at pitch startup.
        radius = m.cad["note"]["majorRadius"]/1000
        pitch = s["notePitch"]
        sign = -1 if pitch < 0 else 1
        gap = s["noteZ"] - m.z0 - radius*abs(math.sin(pitch))
        edge_speed = s["noteVz"] - radius*sign*math.cos(pitch)*s["notePitchRate"]
        acc[1] = zref[2] + 225*(.0018-gap) - 30*edge_speed
    contact = m.contact(s,{})
    reaction = contact["q"]["cradleReactionWorldN"]
    moment = contact["q"]["cradleMomentWorldNm"]
    effort = np.array(terms["massMatrix"]) @ acc - terms["biasForce"] - np.array([reaction[0],reaction[2],-moment[1]])
    actuator = []
    for i, gain in enumerate(terms["transmissionGains"]):
        current = effort[i+1]/(gain*m.ac["motorK"])
        volts = terms["rotorOmegaRadps"][i]*m.ac["motorK"] + current*m.ac["resistanceOhm"]
        actuator.append(float(np.clip(volts/bus,-.95,.95)))
    dx = n["p"][0]-d["x"]-m.cad["stageCenterX"]/1000
    dz = s["noteZ"]-s["zPivot"]
    relative = cz*dx+sz*dz
    relative_v = cz*(n["v"][0]-d["vx"])+sz*(s["noteVz"]-s["vPivot"])+s["thetaRate"]*(-sz*dx+cz*dz)
    initial_offset = m.initial["note"]["p"][0]-m.initial["chassis"]["x"]-m.cad["stageCenterX"]/1000
    indexed, index_rate, index_acc = ramp(tick/100, .55)
    desired_offset = initial_offset*(1-indexed)
    desired_speed = -initial_offset*index_rate
    force = m.nc["pieceMassKg"] * (9.80665*sz + zref[2]*sz - initial_offset*index_acc +
                150*(desired_offset-relative)+24*(desired_speed-relative_v))
    force -= float(contact["floorForce"] @ axis)
    traction = float(m.nc["beltMu"]*sum(contact["pressure"]["normalN"]))
    if traction < 1e-8:
        raise LiftDomainError("LOST_BELT_CONTACT")
    slip = m.nc["slipRegularizationMps"]*math.atanh(float(np.clip(force/traction,-.94,.94)))
    desired_omega = -(relative_v+slip)/m.nc["radiusM"]
    pinch = []
    for omega in n["w"]:
        torque = m.nc["rotorInertiaKgM2"]*150*(desired_omega-omega) - m.nc["radiusM"]*force/2
        current = m.nc["kvRadpsPerV"]*torque
        volts = omega/m.nc["kvRadpsPerV"]+current*m.nc["motorResistanceOhm"]
        if current*volts < 0:
            volts = -math.copysign(.001,omega)
        pinch.append(float(np.clip(volts/bus,-.025,.025)))
    return dict(drive=[0.,0.],driveModes=["brake","brake"],pinch=pinch,
                actuator=actuator,actuatorModes=["drive","drive"],
                requestedPivotZM=zref[0],requestedPitchRad=aref[0])

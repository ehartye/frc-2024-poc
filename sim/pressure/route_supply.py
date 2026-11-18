"""Midpoint battery port for the route's braked chassis and four powered motors."""
import copy
import math
import numpy as np
from native_pressure_lift import LiftDomainError
from shared_supply import solve_supply
from signed_pressure_acquisition import motor_electrical


def midpoint_port(model, dt, cmd, contact, cache):
    if cmd["driveModes"] != ["brake","brake"] or cmd["actuatorModes"] != ["drive","drive"]:
        raise ValueError("Route port requires braked chassis and powered mechanism")
    s = model.s
    d, n = s["chassis"], s["note"]
    v = [d["vx"], s["vPivot"], s["thetaRate"]]
    terms = model.act.terms([d["x"],s["zPivot"],s["theta"]],v)
    gains = np.asarray(terms["transmissionGains"])
    ac, nc = model.ac, model.nc
    matrix = np.array(terms["massMatrix"])
    matrix[0,0] += model.drive.c["massKg"]
    motor_columns = np.zeros((3,2))
    motor_columns[1:, :] = np.diag(gains*ac["motorK"])
    # Reaction, tire load and bias are bus-independent while the drive brakes.
    ref = model.rates(s,12.,cmd,cache,contact)
    gain_matrix = np.linalg.solve(matrix,motor_columns)
    ref_i = np.array([m["currentA"] for m in ref["electrical"]["actuator"]["motors"]])
    fixed = (np.array(v) + .5*dt*(ref["acc"]-gain_matrix@ref_i))[1:]*gains
    slope = (.5*dt*gain_matrix[1:,:]*gains[:,None]).tolist()
    start_emf = np.asarray(terms["rotorOmegaRadps"])*ac["motorK"]
    inertia = nc["rotorInertiaKgM2"]
    k = ac["motorK"]
    resistance = ac["resistanceOhm"]
    limit = ac["currentLimitA"]
    d0,d1 = cmd["actuator"]
    e0,e1 = start_emf.tolist()
    f0,f1 = fixed.tolist()
    a,b = slope[0];c,d1s = slope[1]
    def amps(voltage, emf):
        return max(-limit,min(limit,(voltage-emf)/resistance))
    def port(bus):
        i0,i1 = amps(d0*bus,e0),amps(d1*bus,e1)
        v0,v1 = k*(f0+a*i0+b*i1),k*(f1+c*i0+d1s*i1)
        im0,im1 = amps(d0*bus,v0),amps(d1*bus,v1)
        power=(v0+im0*resistance)*im0+(v1+im1*resistance)*im1
        for duty,w,torque in zip(cmd["pinch"],n["w"],contact["q"]["rotorContactTorqueNm"]):
            first = motor_electrical(duty,bus,w,nc)
            mid = w + .5*dt*(first["torqueNm"]+torque)/inertia
            power += motor_electrical(duty,bus,mid,nc)["terminalPowerW"]
        return {"busPowerW":float(power)}
    return port


def step_route(model, dt, cmd):
    """The reference explicit midpoint step with a cached electrical port.

    The grid, bracketing, stability checks and root selection stay in solve_supply.
    The full midpoint equations independently check the selected port before commit.
    """
    if dt not in (.0005,.00025,.000125):
        raise ValueError("unsupported route timestep")
    s=model.s
    cache={}
    start_contact=model.contact(s,cache)
    port=midpoint_port(model,dt,cmd,start_contact,cache)
    supply=solve_supply({"native-pressure-lift":port},12.,.02)
    first=model.rates(s,supply["busV"],cmd,cache,start_contact)
    mid=model.shifted(s,first,dt/2)
    second=model.rates(mid,supply["busV"],cmd,cache)
    difference=supply["busPowerW"]-second["powerW"]
    if not math.isfinite(difference) or abs(difference)>1e-8:
        raise LiftDomainError("MIDPOINT_PORT_MISMATCH")
    if first["infeasible"] or second["infeasible"]:
        raise LiftDomainError("ACTUATOR_VOLTAGE_DOMAIN")
    end=model.shifted(s,second,dt)
    for key,vel in (("zPivot","vPivot"),("theta","thetaRate"),("noteZ","noteVz"),("notePitch","notePitchRate")):
        end[key]=s[key]+dt*mid[vel]
    end["chassis"]["x"]=s["chassis"]["x"]+dt*mid["chassis"]["vx"]
    end["chassis"]["wheel_angle"]=[x+dt*w for x,w in zip(s["chassis"]["wheel_angle"],mid["chassis"]["wheel"])]
    end["note"]["p"][0]=s["note"]["p"][0]+dt*mid["note"]["v"][0]
    final=model.contact(end,cache)
    rates=dict(second["energyRates"],sourceWork=supply["sourcePowerW"],sourceHeat=supply["sourceHeatW"])
    ledger={key:amount+dt*rates[key] for key,amount in model.ledger.items()}
    mechanical=model.mechanical(end,final)
    residual=model.initial["initialEnergyJ"]+ledger["sourceWork"]-sum(
        v for key,v in ledger.items() if key!="sourceWork")-mechanical
    end["ledger"]=ledger
    model.ledger,model.s=ledger,end
    return dict(timeS=end["timeS"],state=copy.deepcopy(end),busV=supply["busV"],
                sourcePowerResidualW=supply["powerResidualW"],portMidpointPowerDifferenceW=difference,
                pressureWorkResidualW=final["q"]["workResidualW"],noteContactForceN=final["q"]["noteForceWorldN"],
                floorForceN=final["floorForce"].tolist(),normalN=[c["normalMagnitudeN"] for c in final["pressure"]["sideContacts"]],
                contactCount=final["pressure"]["contacts"],maximumPenetrationM=final["pressure"]["maximumPenetrationM"],
                energyJ=dict(ledger,mechanical=mechanical,pressure=final["pressure"]["potentialJ"],floor=final["floorEnergy"],residual=residual),
                acceptance=False)

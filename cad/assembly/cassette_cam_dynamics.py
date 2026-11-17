"""Isolated fixed-cradle two-coordinate hypothesis, not a calibrated servo model."""
from dataclasses import dataclass, fields
import math
from powered_cam_contact import evaluate, scalar
from cassette_spring_law import spring_state

class DomainStop(ValueError):
    """Unresolved event; input immutable state is still the last accepted state."""

@dataclass(frozen=True)
class Parameters:
    cassette_mass_kg: float
    output_inertia_kg_m2: float
    contact_stiffness_n_m: float
    contact_damping_ns_m: float
    torque_limit_nm: float
    def validate(self):
        for f in fields(self):
            v=scalar(f.name,getattr(self,f.name))
            if v<0 or (f.name!='contact_damping_ns_m' and v==0):
                raise ValueError('invalid positive hypothesis '+f.name)

@dataclass(frozen=True)
class State:
    u: float
    udot: float
    theta: float
    thetadot: float
    time: float=0.0
    actuator_work: float=0.0
    load_work: float=0.0
    contact_heat: float=0.0


def ports(state, parameters, requested_torque_nm, note_generalized_force_n=0.0):
    if not isinstance(state,State) or not isinstance(parameters,Parameters):raise ValueError('state/parameters type')
    parameters.validate()
    for f in fields(state):scalar(f.name,getattr(state,f.name))
    if state.time<0 or state.contact_heat<0:raise ValueError('invalid time/heat')
    if not 0<=state.u<=.0015 or not 0<=state.theta<=math.pi:
        raise DomainStop('studied geometry domain crossed; impact or extension unresolved')
    if (state.u==0 and state.udot<0) or (state.u==.0015 and state.udot>0):
        raise DomainStop('outward velocity at hard stop; impulse law unresolved')
    torque=scalar('requested torque',requested_torque_nm)
    load=scalar('NOTE generalized load',note_generalized_force_n)
    torque=max(-parameters.torque_limit_nm,min(parameters.torque_limit_nm,torque))
    contact=evaluate(state.u,state.udot,state.theta,state.thetadot,
        stiffness_n_m=parameters.contact_stiffness_n_m,damping_n_s_m=parameters.contact_damping_ns_m)
    spring=spring_state(state.u)
    free_force=spring['pairInwardForceN']+contact['cassette_generalized_force_n']+load
    # Only static complementarity. Moving impacts are deliberately not projected.
    stop=0.0
    if state.udot==0:
        if state.u==0:stop=max(0.0,-free_force)
        elif state.u==.0015:stop=min(0.0,-free_force)
    force=free_force+stop
    total_torque=torque+contact['cam_generalized_torque_nm']
    try:
        kinetic=.5*parameters.cassette_mass_kg*state.udot**2+.5*parameters.output_inertia_kg_m2*state.thetadot**2
        energy=kinetic+spring['storedEnergyJ']+contact['potential_j']
        acceleration=force/parameters.cassette_mass_kg
        angular_acceleration=total_torque/parameters.output_inertia_kg_m2
        actuator_power=torque*state.thetadot;load_power=load*state.udot
        energy_rate=force*state.udot+total_torque*state.thetadot-spring['pairInwardForceN']*state.udot+contact['potential_rate_w']
        residual=energy_rate+contact['dissipation_w']-actuator_power-load_power
    except OverflowError as ex:raise ValueError('derived overflow') from ex
    numbers=[force,stop,total_torque,energy,acceleration,angular_acceleration,actuator_power,load_power,residual]
    if not all(math.isfinite(v) for v in numbers):raise ValueError('nonfinite dynamics')
    return dict(acceleration=acceleration,angular_acceleration=angular_acceleration,
        energy=energy,kinetic_energy=kinetic,spring_energy=spring['storedEnergyJ'],cam_energy=contact['potential_j'],
        applied_torque=torque,stop_force=stop,stop_power=stop*state.udot,
        actuator_power=actuator_power,load_power=load_power,heat_rate=contact['dissipation_w'],
        instantaneous_power_residual=residual,electrical_current_a=None,positive_latch=False)


def step(state,parameters,dt,requested_torque_nm,note_generalized_force_n=0.0):
    """RK4 with constant external inputs over this step. No state mutation on failure.
    RK4 is not energy exact; callers must retain residual and step convergence.
    Every stage and endpoint validates geometry. No hidden clipping/reflection.
    """
    dt=scalar('dt',dt)
    if dt<=0:raise ValueError('positive dt required')
    def rates(s):
        r=ports(s,parameters,requested_torque_nm,note_generalized_force_n)
        return (s.udot,r['acceleration'],s.thetadot,r['angular_acceleration'],1.0,r['actuator_power'],r['load_power'],r['heat_rate'])
    def shifted(s,rate,h):
        return State(*(getattr(s,f.name)+h*v for f,v in zip(fields(s),rate)))
    a=rates(state);b=rates(shifted(state,a,dt/2));c=rates(shifted(state,b,dt/2));d=rates(shifted(state,c,dt))
    result=shifted(state,tuple((v+2*w+2*x+y)/6 for v,w,x,y in zip(a,b,c,d)),dt)
    ports(result,parameters,requested_torque_nm,note_generalized_force_n)
    if result.time<=state.time:raise ValueError('time increment not representable')
    return result


def equilibrium(theta,parameters):
    """Nominal spring/contact equilibrium, no NOTE, interior cassette travel."""
    parameters.validate();theta=scalar('theta',theta)
    if not 0<=theta<=math.pi:raise ValueError('angle domain')
    k=parameters.contact_stiffness_n_m;a=.00075*(1-math.cos(theta))
    u=(14.6234+k*a)/(5780+k)
    state=State(u,0.0,theta,0.0)
    r=evaluate(u,0,theta,0,stiffness_n_m=k,damping_n_s_m=parameters.contact_damping_ns_m)
    torque=-r['cam_generalized_torque_nm']
    if abs(torque)>parameters.torque_limit_nm:raise ValueError('equilibrium exceeds declared torque bound')
    ports(state,parameters,torque)
    return state,torque

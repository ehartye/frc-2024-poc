"""Two hypothetical transverse cassette mechanisms, not an adopted robot model."""
from dataclasses import dataclass, fields, replace
import math
import cassette_coupling as one

@dataclass(frozen=True)
class Parameters:
    positive: one.Parameters=one.Parameters()
    negative: one.Parameters=one.Parameters()
    def validate(self):
        self.positive.validate();self.negative.validate()
        for key in ['note_mass','note_x','grid_phase_rad','resolution']:
            if getattr(self.positive,key)!=getattr(self.negative,key):raise ValueError('one shared NOTE/grid required')

@dataclass(frozen=True)
class State:
    y: float
    ydot: float
    up: float
    udp: float
    tp: float
    tdp: float
    un: float
    udn: float
    tn: float
    tdn: float
    time: float=0.
    actuator_work: float=0.
    cam_heat: float=0.

def ports(s,p,config,torques=(0.,0.)):
    if not isinstance(s,State) or not isinstance(p,Parameters):raise ValueError('typed state and parameters')
    p.validate()
    for f in fields(s):one.cam.scalar(f.name,getattr(s,f.name))
    if len(torques)!=2:raise ValueError('two mechanical torque inputs')
    results=[]
    for sign,par,u,ud,theta,td,torque in [(1,p.positive,s.up,s.udp,s.tp,s.tdp,torques[0]),(-1,p.negative,s.un,s.udn,s.tn,s.tdn,torques[1])]:
        # Reflection changes material-grid phase and polar y velocity; u is inward on either side.
        local=one.State(sign*s.y,sign*s.ydot,u,ud,theta,td,s.time,s.actuator_work,s.cam_heat)
        result=one.ports(local,replace(par,grid_phase_rad=sign*par.grid_phase_rad),config,torque)
        results.append(result)
    a,b=results;mass=p.positive.note_mass
    note_kinetic=.5*mass*s.ydot*s.ydot
    fy=a['noteForceY']-b['noteForceY']
    energy=a['energyJ']+b['energyJ']-note_kinetic
    # Moments are axial vectors: reflection in y maps (Mx,My,Mz) to (-Mx,My,-Mz).
    cm=[a['noteConstraintMoment'][i]+([-1,1,-1][i])*b['noteConstraintMoment'][i] for i in range(3)]
    cf=[a['noteConstraintForce'][i]+([1,-1,1][i])*b['noteConstraintForce'][i] for i in range(3)]
    out=dict(sides=results,noteForceY=fy,accelerations=[fy/mass,a['accelerations'][1],a['accelerations'][2],b['accelerations'][1],b['accelerations'][2]],energyJ=energy,kineticJ=a['kineticJ']+b['kineticJ']-note_kinetic,pressureJ=a['pressureJ']+b['pressureJ'],springJ=a['springJ']+b['springJ'],camJ=a['camJ']+b['camJ'],actuatorPowerW=a['actuatorPowerW']+b['actuatorPowerW'],heatW=a['heatW']+b['heatW'],powerResidualW=a['powerResidualW']+b['powerResidualW'],noteConstraintForce=cf,noteConstraintMoment=cm,constraintPowerW=0.,electricalCurrentA=None,acceptance=False)
    def finite(v):
        if isinstance(v,dict):
            for x in v.values():finite(x)
        elif isinstance(v,list):
            for x in v:finite(x)
        elif isinstance(v,float) and not math.isfinite(v):raise ValueError('nonfinite derived two-side result')
    finite(out);return out

def step(s,p,config,dt,torques=(0.,0.)):
    dt=one.cam.scalar('dt',dt)
    if dt<=0:raise ValueError('positive dt')
    def rates(q):
        r=ports(q,p,config,torques);ay,ap,atp,an,atn=r['accelerations']
        return (q.ydot,ay,q.udp,ap,q.tdp,atp,q.udn,an,q.tdn,atn,1.,r['actuatorPowerW'],r['heatW'])
    def shift(q,k,h):return State(*(getattr(q,f.name)+h*v for f,v in zip(fields(q),k)))
    a=rates(s);b=rates(shift(s,a,dt/2));c=rates(shift(s,b,dt/2));d=rates(shift(s,c,dt))
    out=shift(s,tuple((x+2*y+2*z+w)/6 for x,y,z,w in zip(a,b,c,d)),dt)
    ports(out,p,config,torques)
    if out.time<=s.time:raise ValueError('unrepresentable time step')
    return out

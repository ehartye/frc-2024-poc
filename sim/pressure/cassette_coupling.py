"""Isolated transverse NOTE/cassette/cam mechanics; not robot acquisition."""
from dataclasses import dataclass, fields
import pathlib, sys, math, json
R=pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0,str(R/'cad/assembly'))
import powered_cam_contact as cam
from cassette_spring_law import spring_state
import fast_geometry as geometry
import finite_transport as transport
import numpy as np

@dataclass(frozen=True)
class Parameters:
    note_mass: float=.234925
    cassette_mass: float=1.5
    cam_inertia: float=.0001
    pressure_density: float=1e7
    cam_stiffness: float=1e6
    cam_damping: float=10.
    torque_limit: float=.03
    spring_free_length: float=.00953
    resolution: int=1024
    note_x: float=.010567440020061214
    grid_phase_rad: float=0.
    def validate(self):
        for f in fields(self):
            if f.name=='resolution':
                if type(self.resolution) is not int or not 8<=self.resolution<=2048:raise ValueError('grid domain')
            else:cam.scalar(f.name,getattr(self,f.name))
        if min(self.note_mass,self.cassette_mass,self.cam_inertia,self.pressure_density,self.cam_stiffness,self.torque_limit)<=0 or self.cam_damping<0:raise ValueError('positive hypotheses required')
        if abs(self.note_x)>.5 or abs(self.grid_phase_rad)>math.pi:raise ValueError('bounded fixture pose/phase')
        spring_state(0,free_length_m=self.spring_free_length)

@dataclass(frozen=True)
class State:
    y: float
    ydot: float
    u: float
    udot: float
    theta: float
    thetadot: float
    time: float=0.
    actuator_work: float=0.
    cam_heat: float=0.

class DomainStop(ValueError):pass

def positive_side(config,s,p):
    """Same scalar cells, but only the actually present positive-side cassette.
    Fixed axes and bounded y/u allow a padded AABB filter; exact retained points
    are recomputed with the original scalar helpers in material-grid order.
    """
    a=transport.a;g=dict(a.belt.geometry(config));g['axisYAbs']-=s.u*1000
    cs=math.cos(p.grid_phase_rad);sn=math.sin(p.grid_phase_rad)
    nb=[[cs,-sn,0.],[sn,cs,0.],[0.,0.,1.]];center=[p.note_x,s.y,0.]
    grid=geometry.material_grid(config['note']['majorRadius']/1000,config['note']['tubeRadius']/1000,p.resolution,p.resolution)
    world=grid[:,:3]@np.array(nb).T+np.array(center)
    lo=np.array([g['rearX']-g['outerRadius'],g['axisYAbs']-g['outerRadius'],-g['halfHeight']])/1000
    hi=np.array([g['frontX']+g['outerRadius'],g['axisYAbs']+g['outerRadius'],g['halfHeight']])/1000
    pad=max(1e-12,64*np.finfo(float).eps*(1+max(map(abs,center))))
    if not np.isfinite(world).all():raise ValueError('nonfinite transformed point')
    mask=np.all((world>=lo-pad)&(world<=hi+pad),axis=1)
    contacts=[];potential=0.;features={}
    for index in np.flatnonzero(mask):
        lever=a.mv(nb,grid[index,:3].tolist());point=a.add(center,lever);area=float(grid[index,3])
        q=transport.gradient(point,1,g);depth=q['penetrationM']
        if depth==0:continue
        wall=a.belt.query(a.scale(point,1000),1,g)['outerSidewall']
        if wall is None or not wall['noteFacingHalf']:raise a.DomainError('UNSUPPORTED_BELT_TANGENT')
        normal=q['normal'];tangent=wall['acquisitionUnitTangent']
        if abs(a.dot(normal,tangent))>1e-10:raise a.DomainError('NORMAL_TANGENT_NOT_ORTHOGONAL')
        fn=p.pressure_density*depth*area;energy=.5*p.pressure_density*depth*depth*area
        if not math.isfinite(fn) or not math.isfinite(energy):raise ValueError('nonfinite pressure')
        potential+=energy;features[q['feature']]=features.get(q['feature'],0)+1
        contacts.append(dict(side=1,motor=1,pointWorldM=point,noteLeverWorldM=lever,cradleLeverWorldM=point,normalWorld=normal,acquisitionTangentWorld=tangent,normalN=fn,areaM2=area,penetrationM=depth,feature=q['feature']))
    return dict(contacts=contacts,potentialJ=potential,features=features,cassetteDirectionWorld=[0.,-1.,0.],acceptance=False)

def ports(s,p,config,requested_torque=0.):
    if not isinstance(s,State) or not isinstance(p,Parameters):raise ValueError('typed state/parameters required')
    p.validate()
    for f in fields(s):cam.scalar(f.name,getattr(s,f.name))
    if s.time<0 or s.cam_heat<0:raise ValueError('negative time/heat')
    if abs(s.y)>.005 or not 0<=s.u<=.0015 or not 0<=s.theta<=math.pi:raise DomainStop('bounded transverse geometry')
    if (s.u==0 and s.udot<0) or (s.u==.0015 and s.udot>0):raise DomainStop('moving stop impact unresolved')
    requested=cam.scalar('mechanical torque',requested_torque)
    torque=max(-p.torque_limit,min(p.torque_limit,requested))
    prepared=positive_side(config,s,p)
    contacts=prepared['contacts'];potential=prepared['potentialJ']
    r=transport.rates(prepared,note_velocity=[0.,s.ydot,0.],u_velocity=s.udot,rotor_omega=[0.,0.],pitch_radius_m=.012,mu=0.)
    spring=spring_state(s.u,free_length_m=p.spring_free_length)
    c=cam.evaluate(s.u,s.udot,s.theta,s.thetadot,stiffness_n_m=p.cam_stiffness,damping_n_s_m=p.cam_damping)
    fy=r['noteForceWorldN'][1];qu=r['cassetteGeneralizedForceN']
    free=spring['pairInwardForceN']+c['cassette_generalized_force_n']+qu
    stop=0.
    if s.udot==0:
        if s.u==0:stop=max(0.,-free)
        elif s.u==.0015:stop=min(0.,-free)
    qt=torque+c['cam_generalized_torque_nm']
    try:kinetic=.5*p.note_mass*s.ydot**2+.5*p.cassette_mass*s.udot**2+.5*p.cam_inertia*s.thetadot**2
    except OverflowError as ex:raise ValueError('derived kinetic overflow') from ex
    energy=kinetic+potential+spring['storedEnergyJ']+c['potential_j']
    power=fy*s.ydot+(free+stop)*s.udot+qt*s.thetadot+r['normalPotentialRateW']-spring['pairInwardForceN']*s.udot+c['potential_rate_w']+c['dissipation_w']-torque*s.thetadot
    result=dict(accelerations=[fy/p.note_mass,(free+stop)/p.cassette_mass,qt/p.cam_inertia],energyJ=energy,kineticJ=kinetic,pressureJ=potential,springJ=spring['storedEnergyJ'],camJ=c['potential_j'],actuatorPowerW=torque*s.thetadot,heatW=c['dissipation_w'],powerResidualW=power,noteForceY=fy,cassetteContactForce=qu,springForce=spring['pairInwardForceN'],stopForce=stop,appliedTorque=torque,activeCells=len(contacts),activeArea=sum(q['areaM2'] for q in contacts),noteConstraintForce=[-r['noteForceWorldN'][0],0.,-r['noteForceWorldN'][2]],noteConstraintMoment=[-x for x in r['noteMomentWorldNm']],constraintPowerW=0.,contactWorkResidualW=r['workResidualW'],electricalCurrentA=None,acceptance=False)
    def check(v):
        if isinstance(v,dict):
            for x in v.values():check(x)
        elif isinstance(v,list):
            for x in v:check(x)
        elif isinstance(v,float) and not math.isfinite(v):raise ValueError('nonfinite derived result')
    check(result);return result

def step(s,p,config,dt,requested_torque=0.):
    dt=cam.scalar('dt',dt)
    if dt<=0:raise ValueError('positive dt')
    def rates(q):
        r=ports(q,p,config,requested_torque);ay,au,at=r['accelerations']
        return (q.ydot,ay,q.udot,au,q.thetadot,at,1.,r['actuatorPowerW'],r['heatW'])
    def shift(q,k,h):return State(*(getattr(q,f.name)+h*v for f,v in zip(fields(q),k)))
    a=rates(s);b=rates(shift(s,a,dt/2));c=rates(shift(s,b,dt/2));d=rates(shift(s,c,dt))
    out=shift(s,tuple((x+2*y+2*z+w)/6 for x,y,z,w in zip(a,b,c,d)),dt)
    ports(out,p,config,requested_torque)
    if out.time<=s.time:raise ValueError('unrepresentable time step')
    return out

def config():return json.loads((R/'cad/assembly/intake_components.json').read_text())

"""Actual acquisition wrapper at fixed u=0; dynamic release deliberately unavailable."""
import copy, math
from dataclasses import dataclass
import cassette_coupling as component
from pressure_acquisition import rotate
import finite_transport

@dataclass(frozen=True)
class LockedSide:
    theta: float = .2
    spring_free_length: float = .00953
    cam_stiffness: float = 1e6
    cam_damping: float = 10.
    def stored(self):
        for name in self.__dataclass_fields__: component.cam.scalar(name,getattr(self,name))
        spring=component.spring_state(0.,free_length_m=self.spring_free_length)
        cam=component.cam.evaluate(0.,0.,self.theta,0.,stiffness_n_m=self.cam_stiffness,damping_n_s_m=self.cam_damping)
        return spring,cam

class FixedCassettes:
    def __init__(self,plant,positive=LockedSide(),negative=LockedSide()):
        # This is a fresh experiment, not an adapter that overwrites a moving checkpoint.
        if plant.t!=0 or any(plant.energy.values()) or plant.initial_energy!=0:
            raise ValueError('fresh zero-reference acquisition required')
        if plant.drive.kinetic()!=0 or plant.note_kinetic(plant.note)!=0:
            raise ValueError('fresh rest required')
        prepared=plant.prepared(plant.drive.state,plant.note,{})
        if prepared['contacts'] or prepared['potentialJ']!=0:raise ValueError('fresh separated NOTE required')
        if not isinstance(positive,LockedSide) or not isinstance(negative,LockedSide):raise ValueError('typed side hypotheses')
        self.sides=(positive,negative)
        states=[s.stored() for s in self.sides]
        self.initial_stored=sum(s['storedEnergyJ']+c['potential_j'] for s,c in states)
        if not math.isfinite(self.initial_stored):raise ValueError('initial energy overflow')
        self.plant=copy.deepcopy(plant)
    def constraint_ports(self,drive,note):
        p=self.plant;prepared=p.prepared(drive,note,{})
        stage=p.cad['stageCenterX']/1000;offset=rotate([stage,0.],drive['heading'])
        cv=[drive['vx']-drive['yaw_rate']*offset[1],drive['vy']+drive['yaw_rate']*offset[0],0.]
        out=[]
        for sign,hypothesis in zip((1,-1),self.sides):
            subset=dict(prepared,contacts=[cell for cell in prepared['contacts'] if cell['side']==sign])
            rate=finite_transport.rates(subset,note_velocity=[*note['v'],0.],note_omega=[0.,0.,note['spin']],cradle_velocity=cv,cradle_omega=[0.,0.,drive['yaw_rate']],rotor_omega=note['w'],pitch_radius_m=p.c['radiusM'],mu=p.c['beltMu'],slip_scale=p.c['slipRegularizationMps'])
            direction=rotate([0.,-sign],drive['heading'])
            force=sum(a*b for a,b in zip(rate['cradleReactionWorldN'][:2],direction))
            spring,cam=hypothesis.stored()
            out.append(dict(side=sign,u=0.,udot=0.,theta=hypothesis.theta,thetadot=0.,pressureGeneralizedForceN=force,lockedUReactionN=-(force+spring['pairInwardForceN']+cam['cassette_generalized_force_n']),lockedThetaReactionNm=-cam['cam_generalized_torque_nm'],relativeConstraintWorkW=0.,componentPotentialJ=spring['storedEnergyJ']+cam['potential_j'],transport=rate))
        return out
    def step(self,*args,**kwargs):
        # Stage everything on a copy so unsupported post-step diagnostics cannot commit state.
        candidate=copy.deepcopy(self)
        row=candidate.plant.step(*args,**kwargs)
        sides=candidate.constraint_ports(candidate.plant.drive.state,candidate.plant.note)
        energy=row['energyJ'];total_initial=energy['initial']+self.initial_stored
        total_final=energy['kinetic']+energy['spring']+self.initial_stored
        if not all(math.isfinite(v) for v in (total_initial,total_final)):raise ValueError('combined energy overflow')
        result=dict(acquisition=row,sides=sides,energy=dict(initial=total_initial,currentStoredAndKinetic=total_final,constantComponentPotentialJ=self.initial_stored,residual=energy['residual']),dynamicReleaseAvailable=False,acceptance=False)
        self.plant=candidate.plant
        return result
    def release(self,*args,**kwargs):
        raise ValueError('dynamic release unavailable: moving mass partition and reciprocal inertial coupling required; state unchanged')

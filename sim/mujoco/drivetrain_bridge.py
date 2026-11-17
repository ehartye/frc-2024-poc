"""Torque/force-driven MuJoCo planar drivetrain bridge; NOT native tire contact.
Five DOFs: chassis world X/Y/yaw, two abstract axle angles. MuJoCo integrates
state; reviewed source six-station slip forces and four-CIM electrical law
supply generalized forces. Axles are separate inertial coordinates because
roll/pitch reactions are constrained out of the source planar reduction.
"""
import copy,hashlib,importlib.util,json,math,pathlib
import mujoco
import numpy as np


def load_source(root):
    root=pathlib.Path(root)
    config_path=root/'sim/drivetrain/config.json';model_path=root/'sim/drivetrain/plant.py'
    spec=importlib.util.spec_from_file_location('reviewed_source_drive',model_path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    config=json.loads(config_path.read_text());drive=module.DrivePlant(config)
    pins={str(p.relative_to(root)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in [config_path,model_path]}
    return drive,pins


def xml(config,axle_inertia,dt):
    sites=''.join(f'<site name="contact_{side}_{i}" pos="{x} {y} 0" size=".01"/>' for side,rows in enumerate(config['contactStationsM']) for i,(x,y) in enumerate(rows))
    rotor=''.join(f'<body name="abstract_axle_{i}" pos="0 0 {-1-i}"><joint name="axle_{i}" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="{axle_inertia} {axle_inertia} {axle_inertia}"/></body>' for i in range(2))
    return f'''<mujoco model="source-planar torque bridge"><compiler angle="radian"/><option timestep="{dt}" gravity="0 0 0" integrator="Euler"/><worldbody><body name="chassis"><joint name="x" type="slide" axis="1 0 0"/><joint name="y" type="slide" axis="0 1 0"/><joint name="yaw" type="hinge" axis="0 0 1"/><inertial pos="0 0 0" mass="{config['massKg']}" diaginertia="{config['yawInertiaKgM2']} {config['yawInertiaKgM2']} {config['yawInertiaKgM2']}"/>{sites}</body>{rotor}</worldbody></mujoco>'''


class Bridge:
    def __init__(self,source_root,dt=.00025):
        if type(dt) not in (float,int) or not math.isfinite(dt) or not 0<dt<=.001:raise ValueError('dt must be finite and within (0,1ms]')
        self.source,self.source_pins=load_source(source_root);self.c=self.source.c
        self.model=mujoco.MjModel.from_xml_string(xml(self.c,self.source.axle_inertia,dt));self.data=mujoco.MjData(self.model)
        self.energy={k:0. for k in self.source.energy};self.initial_kinetic=0.
        mujoco.mj_forward(self.model,self.data)
    def state(self):
        q=self.data.qpos;v=self.data.qvel
        return dict(t=float(self.data.time),x=float(q[0]),y=float(q[1]),heading=float(q[2]),vx=float(v[0]),vy=float(v[1]),yaw_rate=float(v[2]),wheel=v[3:5].tolist(),wheel_angle=q[3:5].tolist())
    def query(self,demands,modes=('drive','drive'),bus_v=12.,normal_loads=None,external_force_world=(0.,0.),external_yaw_torque=0.):
        normal_loads=[self.c['massKg']*9.80665/2]*2 if normal_loads is None else normal_loads
        self.source._validate_step(self.model.opt.timestep,normal_loads,external_force_world,external_yaw_torque)
        r=self.source._rates(self.state(),bus_v,demands,modes,normal_loads,external_force_world,external_yaw_torque)
        if r['electrical']['infeasibleCurrentLimitVoltage']:raise ValueError('unsupported controller terminal voltage')
        force=[r['dvx']*self.c['massKg'],r['dvy']*self.c['massKg'],r['dyaw']*self.c['yawInertiaKgM2'],*[v*self.source.axle_inertia for v in r['dw']]]
        if not all(math.isfinite(v) for v in force):raise ValueError('nonfinite generalized force')
        return r,np.array(force)
    def step(self,demands,modes=('drive','drive'),bus_v=12.,normal_loads=None,external_force_world=(0.,0.),external_yaw_torque=0.):
        # Validate pure source queries before any MuJoCo integration or ledger change.
        rates,force=self.query(demands,modes,bus_v,normal_loads,external_force_world,external_yaw_torque)
        self.data.qfrc_applied[:]=force
        mujoco.mj_step(self.model,self.data)
        for key,power in rates['energyRates'].items():self.energy[key]+=self.model.opt.timestep*power
        state=self.state();kinetic=self.source.kinetic(state)
        residual=self.initial_kinetic+self.energy['terminalWork']+self.energy['externalWork']-sum(v for k,v in self.energy.items() if k not in ('terminalWork','externalWork'))-kinetic
        return dict(state=state,electrical=rates['electrical'],contactModel='reviewed six-station regularized slip, not MuJoCo ground contacts',sourceContactSamples=rates['contacts'],energyJ=dict(self.energy,kinetic=kinetic,residual=residual),acceptance=False)

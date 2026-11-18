"""Isolated uncalibrated finite-area gravity-release experiment, not a robot demo.

MuJoCo owns the free NOTE's six DOFs. The existing pressure law supplies only
forces and moments; the fixed belt fixture supplies reaction, never a pose reset.
"""
import argparse, hashlib, json, math, pathlib, sys, time
import mujoco
import numpy as np

def run(args):
    root=args.project.resolve()
    sys.path[:0]=[str(root/'sim/pressure'),str(root/'cad/assembly'),str(root/'cad')]
    import fast_geometry, finite_area, finite_transport
    config=json.loads((root/'cad/assembly/intake_components.json').read_text())
    model=json.loads((root/'sim/acquisition/free-planar-config.json').read_text())['model']
    original=finite_area.belt.geometry
    def revised(c):
        g=original(c);g['axisYAbs']+=1.;g['outerRadius']+=args.tread_mm;return g
    finite_area.belt.geometry=revised
    mass=model['pieceMassKg'];major=config['note']['majorRadius']/1000;tube=config['note']['tubeRadius']/1000
    ix=mass*(major**2/2+5*tube**2/8);iz=mass*(major**2+3*tube**2/4)
    assert abs(iz-model['yawInertiaKgM2'])<1e-12
    xml=f'<mujoco><option timestep="{args.dt}" gravity="0 0 -9.80665" integrator="Euler"/><worldbody><body name="note" pos=".185 0 .3"><freejoint/><inertial pos="0 0 0" mass="{mass}" diaginertia="{ix} {ix} {iz}"/></body></worldbody></mujoco>'
    m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    assert m.nv==6 and m.neq==0 and m.nu==0 and m.ngeom==0
    assert not args.output.exists();args.output.mkdir(parents=True)
    (args.output/'model.xml').write_text(xml+'\n')
    vel=np.zeros(6);start=time.perf_counter();rows=[];heat=0.;max_residual=0.;initial_energy=None;status='HORIZON';error=None
    count=round(args.duration/args.dt)
    with (args.output/'trace.jsonl').open('x') as trace:
        for step in range(count+1):
            mujoco.mj_forward(m,d)
            mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_BODY,1,vel,0)
            try:
                if args.no_contact:
                    q={'potentialJ':0.,'contacts':[]}
                    r={'noteForceWorldN':[0.,0.,0.],'noteMomentWorldNm':[0.,0.,0.],'slipHeatW':0.,'workResidualW':0.}
                else:
                    q=fast_geometry.prepare(config,note_center=d.xpos[1].tolist(),note_basis=d.xmat[1].reshape(3,3).tolist(),cradle_origin=[.178,0.,.3],resolution=(args.resolution,args.resolution),k_area=1e7)
                    r=finite_transport.rates(q,note_velocity=vel[3:].tolist(),note_omega=vel[:3].tolist(),pitch_radius_m=model['radiusM'],mu=args.mu,slip_scale=model['slipRegularizationMps'])
            except finite_area.DomainError as e:
                status='DOMAIN_STOP';error=str(e);break
            rotational=.5*float(np.dot(np.array([ix,ix,iz])*d.qvel[3:],d.qvel[3:]))
            kinetic=.5*mass*float(np.dot(vel[3:],vel[3:]))+rotational
            energy=kinetic+mass*9.80665*(float(d.xpos[1,2])-.3)+q['potentialJ']
            if initial_energy is None:initial_energy=energy
            max_residual=max(max_residual,abs(r['workResidualW']))
            row={'step':step,'timeS':float(d.time),'qpos':d.qpos.tolist(),'qvel':d.qvel.tolist(),'forceN':r['noteForceWorldN'],'momentNm':r['noteMomentWorldNm'],'contactCount':len(q['contacts']),'normalN':sum(c['normalN'] for c in q['contacts']),'potentialJ':q['potentialJ'],'kineticJ':kinetic,'cumulativeSlipHeatJ':heat,'energyBalanceErrorJ':energy-initial_energy+heat,'workResidualW':r['workResidualW']}
            trace.write(json.dumps(row)+'\n');rows.append(row)
            if step%100==0:trace.flush();print(json.dumps({'step':step,'z':float(d.xpos[1,2]),'elapsedS':time.perf_counter()-start}),flush=True)
            if float(d.xpos[1,2])<.25:status='DROPPED_50MM';break
            if step==count:break
            d.qfrc_applied[:]=0.
            mujoco.mj_applyFT(m,d,np.array(r['noteForceWorldN']),np.array(r['noteMomentWorldNm']),d.xpos[1],1,d.qfrc_applied)
            heat+=r['slipHeatW']*args.dt
            mujoco.mj_step(m,d)
            assert np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all()
    files=['sim/pressure/fast_geometry.py','sim/pressure/finite_area.py','sim/pressure/finite_transport.py','cad/assembly/belt_contact_geometry.py','cad/assembly/intake_components.json','sim/acquisition/free-planar-config.json']
    pins={f:hashlib.sha256((root/f).read_bytes()).hexdigest() for f in files}
    report={'schema':'fixed-fixture-gravity-release/1','complete':True,'status':status,'error':error,'parameters':{'treadThicknessMm':args.tread_mm,'shaftOutboardMm':1.,'dtS':args.dt,'resolution':args.resolution,'mu':args.mu,'kAreaNpm3':1e7,'noContactControl':args.no_contact},'runtime':{'mujoco':mujoco.__version__,'numpy':np.__version__},'inputs':pins,'scriptSha256':hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest(),'traceSha256':hashlib.sha256((args.output/'trace.jsonl').read_bytes()).hexdigest(),'steps':len(rows),'final':rows[-1] if rows else None,'lastState':{'timeS':float(d.time),'qpos':d.qpos.tolist(),'qvel':d.qvel.tolist()},'maxAbsWorkResidualW':max_residual,'maxAbsEnergyBalanceErrorJ':max(abs(r['energyBalanceErrorJ']) for r in rows),'seconds':time.perf_counter()-start,'acceptance':False,'limits':['Isolated stationary fixture, initially staged NOTE; no intake, cradle motion, drivetrain, launcher or scoring','Only MuJoCo advances the free six-DOF NOTE; finite-area pressure supplies force and torque, no attachment or pose reset','Uncalibrated pressure, friction, solid-torus inertia and tread compliance; fixed cradle/locked belt is an external support boundary','Regularized Coulomb law has no static sticking state and may creep; this is not verified hardware retention','No rigid hard-part or floor contact in this isolated fixture; stop on unsupported contact domain or 50 mm drop']}
    (args.output/'receipt.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--project',type=pathlib.Path,required=True);p.add_argument('--output',type=pathlib.Path,required=True);p.add_argument('--tread-mm',type=float,required=True);p.add_argument('--resolution',type=int,default=512);p.add_argument('--dt',type=float,default=.001);p.add_argument('--duration',type=float,default=1.);p.add_argument('--mu',type=float,default=.6);p.add_argument('--no-contact',action='store_true');args=p.parse_args()
    assert args.tread_mm in (1.,1.5) and args.dt>0 and args.duration>0 and 0<=args.mu<=1
    run(args)

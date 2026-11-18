"""Built-in MuJoCo contact comparison for a native-backed stationary pinch fixture.

Capsules approximate the NOTE torus with a quantified centerline sagitta. Two
boxes represent only the native inner straight belt runs. No robot/lift/launcher.
"""
import argparse,hashlib,json,math,pathlib,time
import mujoco
import numpy as np

def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def make_model(config,parameters,segments,dt,mu,stiffness,noslip,gravity=True,collision=True):
    R=config['note']['majorRadius']/1000;r=config['note']['tubeRadius']/1000;m=.235
    ix=m*(R*R/2+5*r*r/8);iz=m*(R*R+3*r*r/4)
    p=config['pinch'];separation=(p['beltTeethHypothesis']-p['teeth'])*p['pitch']/2/1000
    cx=(config['stageCenterX']+p['beltCenterX'])/1000
    face=parameters['nominalContactFaceAbsYmm']/1000
    width=(p['beltRadialThicknessHypothesis']+parameters['outerTreadThicknessMm'])/1000
    geoms=[]
    for side in (-1,1):
        geoms.append(f'<geom name="belt{side}" type="box" pos="{cx} {side*(face+width/2)} .3" size="{separation/2} {width/2} {p["beltWidth"]/2000}" contype="2" conaffinity="1" priority="1" condim="3" friction="{mu} 0 0" solref="{-stiffness} {-2*math.sqrt(stiffness)}" solimp=".9 .9 .001 .5 2"/>')
    rings=[]
    for i in range(segments):
        a=2*math.pi*i/segments;b=2*math.pi*(i+1)/segments
        rings.append(f'<geom name="note{i}" type="capsule" fromto="{R*math.cos(a)} {R*math.sin(a)} 0 {R*math.cos(b)} {R*math.sin(b)} 0" size="{r}" contype="{1 if collision else 0}" conaffinity="{2 if collision else 0}" density="0"/>')
    xml=f'<mujoco><option timestep="{dt}" gravity="0 0 {-9.80665 if gravity else 0}" cone="elliptic" solver="Newton" iterations="100" tolerance="1e-12" noslip_iterations="{noslip}" noslip_tolerance="1e-12"/><worldbody>{"".join(geoms)}<body name="note" pos=".185 0 .3"><freejoint/><inertial pos="0 0 0" mass="{m}" diaginertia="{ix} {ix} {iz}"/>{"".join(rings)}</body></worldbody></mujoco>'
    model=mujoco.MjModel.from_xml_string(xml);data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    assert model.nv==6 and model.neq==0 and model.nu==0
    return model,data,xml

def contact_result(model,data):
    force=np.zeros(3);torque=np.zeros(3);normals=[0.,0.];max_excess=0.;raw=np.zeros(6)
    for i in range(data.ncon):
        c=data.contact[i];g1,g2=map(int,c.geom);assert (g1<2) != (g2<2)
        belt=min(g1,g2);sign=1 if model.geom_bodyid[g2]==1 else -1
        mujoco.mj_contactForce(model,data,i,raw)
        f=sign*c.frame.reshape(3,3).T@raw[:3]
        force+=f;torque+=np.cross(c.pos-data.xpos[1],f)
        normals[belt]+=raw[0]
        max_excess=max(max_excess,math.hypot(raw[1],raw[2])-c.friction[0]*raw[0])
    assert np.max(np.abs(force-data.qfrc_constraint[:3]))<1e-8
    return {'normalN':normals,'forceWorldN':force.tolist(),'momentWorldNm':torque.tolist(),'contacts':int(data.ncon),'maxFrictionConeExcessN':max_excess}

def run(args):
    root=args.project.resolve();receipt_path=root/'sim/integrated/evidence/track-tread-candidate-20261118.json'
    source=json.loads(receipt_path.read_text());assert digest(args.native_archive)==source['native']['finalArchiveSha256']
    config_path=root/'cad/assembly/intake_components.json';config=json.loads(config_path.read_text())
    assert digest(config_path)==source['staticCapacity']['onePointFiveMm']['inputs']['cad/assembly/intake_components.json']
    parameters=source['native']['parameters'];assert parameters['returnRunRelief'] and parameters['outerTreadThicknessMm']==1.5
    target=next(r['normalN'][0] for r in source['staticCapacity']['onePointFiveMm']['rows'] if r['resolution']==2048 and r['noteXmm']==185)
    # Match only the initial normal force with gravity/friction disabled. Never
    # tune against the hold/drop outcome. This is a hypothesis fit, not calibration.
    trial,td,_=make_model(config,parameters,args.segments,args.dt,0,1000,0,gravity=False)
    initial=contact_result(trial,td);scale=sum(initial['normalN'])/2
    assert scale>0
    stiffness=1000*target/scale
    check,cd,_=make_model(config,parameters,args.segments,args.dt,0,stiffness,0,gravity=False)
    matched=contact_result(check,cd);assert max(abs(n-target) for n in matched['normalN'])<target*.001
    model,data,xml=make_model(config,parameters,args.segments,args.dt,args.mu,stiffness,args.noslip,collision=not args.no_contact)
    assert not args.output.exists();args.output.mkdir(parents=True)
    (args.output/'model.xml').write_text(xml+'\n')
    data.qvel[2]=args.initial_vz
    start=time.perf_counter();rows=[];status='HORIZON';max_cone=0.;max_xy=0.;max_rotation=0.
    with (args.output/'trace.jsonl').open('x') as f:
        for step in range(round(args.duration/args.dt)+1):
            mujoco.mj_forward(model,data);c=contact_result(model,data)
            max_cone=max(max_cone,c['maxFrictionConeExcessN'])
            xy=math.hypot(float(data.qpos[0])-.185,float(data.qpos[1]));rotation=2*math.acos(min(1,abs(float(data.qpos[3]))));max_xy=max(max_xy,xy);max_rotation=max(max_rotation,rotation)
            row={'step':step,'timeS':float(data.time),'qpos':data.qpos.tolist(),'qvel':data.qvel.tolist(),**c}
            if step%10==0 or step==round(args.duration/args.dt):f.write(json.dumps(row)+'\n')
            rows.append(row)
            if float(data.qpos[2])<.25:status='DROPPED_50MM';f.write(json.dumps(row)+'\n');break
            if xy>.001 or rotation>.01:status='OUTSIDE_STRAIGHT_FIXTURE_DOMAIN';f.write(json.dumps(row)+'\n');break
            if step==round(args.duration/args.dt):break
            mujoco.mj_step(model,data)
            assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
    warning_counts=[int(w.number) for w in data.warning];assert not any(warning_counts),warning_counts
    result={'schema':'native-static-contact-comparison/1','complete':True,'status':status,'nativeArchiveSha256':digest(args.native_archive),'sourceReceiptSha256':digest(receipt_path),'sourceConfigSha256':digest(config_path),'scriptSha256':digest(pathlib.Path(__file__)),'mujocoVersion':mujoco.__version__,'parameters':{'segments':args.segments,'dtS':args.dt,'mu':args.mu,'noslipIterations':args.noslip,'initialVzMps':args.initial_vz,'noContactControl':args.no_contact,'fittedStiffness':stiffness,'impedance':.9,'initialNormalForceTargetPerSideN':target,'matchedGravityFreeNormalN':matched['normalN'],'torusApproximationMaxSagittaMm':config['note']['majorRadius']*(1-math.cos(math.pi/args.segments))},'steps':len(rows),'final':rows[-1],'maxAbsVerticalDisplacementM':max(abs(x['qpos'][2]-.3) for x in rows),'maxXYDisplacementM':max_xy,'maxRotationRad':max_rotation,'maxFrictionConeExcessN':max_cone,'warningCounts':warning_counts,'seconds':time.perf_counter()-start,'traceSha256':digest(args.output/'trace.jsonl'),'acceptance':False,'limits':['Built-in MuJoCo contact formulation comparison, not the original finite-area law or hardware calibration','Normal stiffness fits the prior uncalibrated initial force hypothesis with gravity/friction off; no retention-outcome tuning','Only two native straight inner belt faces; NOTE torus is a finite capsule-chain approximation with stated sagitta','Stationary fixture with no floor, initially staged NOTE; no acquisition, drive, lift, launcher or motor simulation','Free six-DOF NOTE, no equality constraint, actuator, attachment or state overwrite after initialization','Stop outside 1 mm XY/0.01 rad straight-fixture domain; no claim for curved wrap or other native hard parts']}
    (args.output/'receipt.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--project',type=pathlib.Path,required=True);p.add_argument('--native-archive',type=pathlib.Path,required=True);p.add_argument('--output',type=pathlib.Path,required=True);p.add_argument('--segments',type=int,default=128);p.add_argument('--dt',type=float,default=.0002);p.add_argument('--mu',type=float,default=.6);p.add_argument('--noslip',type=int,default=20);p.add_argument('--initial-vz',type=float,default=0.);p.add_argument('--duration',type=float,default=1.);p.add_argument('--no-contact',action='store_true');a=p.parse_args();assert a.segments>=32 and a.segments%4==0 and a.dt>0 and a.duration>0 and 0<=a.mu<=1 and a.noslip>=0;run(a)

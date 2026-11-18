"""Single-solve launcher/drivetrain experiment from an explicitly staged NOTE.

No intake-to-launcher transition, full native hard-part clearance or scoring claim.
"""
import argparse,copy,hashlib,json,math,pathlib,sys,time,xml.etree.ElementTree as ET
import mujoco
import numpy as np

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def model_xml(drive,binding,config,args,stiffness,mu,gravity=True,damping=None):
    tree=ET.fromstring(drive.xml);world=tree.find('worldbody');chassis=world.find("body[@name='chassis']")
    option=tree.find('option');option.set('cone','elliptic');option.set('noslip_iterations',str(args.noslip_iterations));option.set('noslip_tolerance','1e-12')
    if not gravity:option.set('gravity','0 0 0')
    for geom in tree.findall('.//geom'):geom.set('contype','1');geom.set('conaffinity','1')
    if args.floating:
        option.set('gravity','0 0 0');world.find('geom').set('contype','0');world.find('geom').set('conaffinity','0')
    mass=.2;spin=config['rotatingInertiaEachKgM2'];positions=[];yaw_moments=[]
    for i,wheel in enumerate(binding['wheels']):
        center=wheel['centerM'];positions.append(center);width=2*wheel['halfWidthM'];trans=spin/2+mass*width*width/12;yaw_moments.append(trans)
        body=ET.SubElement(chassis,'body',name=f'launcher_{i}',pos=' '.join(map(str,center)))
        ET.SubElement(body,'joint',name=f'launcher_{i}',type='hinge',axis=f'0 {-wheel["side"]} 0')
        ET.SubElement(body,'inertial',pos='0 0 0',mass=str(mass),diaginertia=f'{trans} {spin} {trans}')
        ET.SubElement(body,'geom',name=f'tread_{i}',type='cylinder',size=f'{wheel["radiusM"]} {wheel["halfWidthM"]}',quat='.7071067811865476 .7071067811865476 0 0',density='0',contype='2',conaffinity='4',priority='1',condim='3',friction=f'{mu} 0 0',solref=f'{-stiffness} {-(2*math.sqrt(stiffness) if damping is None else damping)}',solimp='.9 .9 .001 .5 2')
    inertia=chassis.find('inertial');old_mass=float(inertia.get('mass'));old_pos=np.fromstring(inertia.get('pos'),sep=' ');old_i=np.fromstring(inertia.get('diaginertia'),sep=' ')
    new_mass=old_mass-4*mass;new_pos=(old_mass*old_pos-mass*np.sum(positions,axis=0))/new_mass
    yaw=old_i[2]+old_mass*sum(old_pos[:2]**2)-new_mass*sum(new_pos[:2]**2)-sum(iz+mass*sum(np.array(pos[:2])**2) for iz,pos in zip(yaw_moments,positions))
    inertia.set('mass',str(new_mass));inertia.set('pos',' '.join(map(str,new_pos)));inertia.set('diaginertia',f'{old_i[0]} {old_i[1]} {yaw}')
    a=binding['pitchRad'];direction=np.array([math.cos(a),0,math.sin(a)]);position=np.array(binding['stationM'])+args.start*direction+np.array([0,0,.001])
    body=ET.SubElement(world,'body',name='note',pos=' '.join(map(str,position)),quat=f'{math.cos(a/2)} 0 {-math.sin(a/2)} 0')
    ET.SubElement(body,'freejoint',name='note_free')
    m=config['pieceMassKg'];R=binding['noteMajorRadiusM'];r=binding['noteTubeRadiusM'];ix=m*(R*R/2+5*r*r/8);iz=m*(R*R+3*r*r/4)
    ET.SubElement(body,'inertial',pos='0 0 0',mass=str(m),diaginertia=f'{ix} {ix} {iz}')
    if args.surface=='sdf':
        extension=ET.SubElement(tree,'extension');plugin=ET.SubElement(extension,'plugin',plugin='mujoco.sdf.torus');instance=ET.SubElement(plugin,'instance',name='note_torus')
        ET.SubElement(instance,'config',key='radius1',value=str(R));ET.SubElement(instance,'config',key='radius2',value=str(r))
        asset=tree.find('asset')
        if asset is None:asset=ET.SubElement(tree,'asset')
        mesh=ET.SubElement(asset,'mesh',name='note_torus');ET.SubElement(mesh,'plugin',instance='note_torus')
        geom=ET.SubElement(body,'geom',name='note_sdf',type='sdf',mesh='note_torus',density='0',contype='0' if args.no_contact else '4',conaffinity='0' if args.no_contact else '2');ET.SubElement(geom,'plugin',instance='note_torus')
        option.set('sdf_initpoints',str(args.sdf_points));option.set('sdf_iterations',str(args.sdf_iterations))
    for i in range(args.segments if args.surface=='capsules' else 0):
        aa=2*math.pi*i/args.segments+math.radians(args.ring_phase_deg);bb=2*math.pi*(i+1)/args.segments+math.radians(args.ring_phase_deg)
        ET.SubElement(body,'geom',name=f'note_{i}',type='capsule',fromto=f'{R*math.cos(aa)} {R*math.sin(aa)} 0 {R*math.cos(bb)} {R*math.sin(bb)} 0',size=str(r),density='0',contype='0' if args.no_contact else '4',conaffinity='0' if args.no_contact else '2')
    return ET.tostring(tree,encoding='unicode')

def observe(model,data,binding):
    nb=model.body('note').id;cb=model.body('chassis').id;rotation=data.xmat[cb].reshape(3,3);a=binding['pitchRad']
    direction=rotation@np.array([math.cos(a),0,math.sin(a)]);station=data.xpos[cb]+rotation@np.array(binding['stationM'])
    normal=rotation@np.array([-math.sin(a),0,math.cos(a)]);delta=data.xpos[nb]-station
    force=np.zeros(3);generalized=np.zeros(model.nv);normals=[0.]*4;count=0;cone=0.;deepest=0.;edge_contacts=0;minimum_radial=float('inf')
    for i in range(data.ncon):
        contact=data.contact[i];g1,g2=map(int,contact.geom);b1,b2=map(int,model.geom_bodyid[[g1,g2]])
        if nb not in (b1,b2):continue
        other=b1 if b2==nb else b2;name=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,other);assert name.startswith('launcher_')
        wi=int(name.rsplit('_',1)[1]);raw=np.zeros(6);mujoco.mj_contactForce(model,data,i,raw)
        f=(1 if b2==nb else -1)*contact.frame.reshape(3,3).T@raw[:3];force+=f
        mujoco.mj_applyFT(model,data,f,np.zeros(3),contact.pos,nb,generalized);normals[wi]+=raw[0];count+=1
        cone=max(cone,math.hypot(raw[1],raw[2])-contact.friction[0]*raw[0]);deepest=max(deepest,-float(contact.dist))
        axis=data.xmat[cb].reshape(3,3)[:,1]
        radial=contact.pos-data.xpos[other];radial-=axis*np.dot(axis,radial);minimum_radial=min(minimum_radial,float(np.linalg.norm(radial)))
        if abs(np.dot(contact.frame.reshape(3,3)[0],axis))>.1:edge_contacts+=1
    adr=int(model.joint('note_free').dofadr[0]);error=float(np.max(np.abs(generalized[adr:adr+6]-data.qfrc_constraint[adr:adr+6])));assert error<1e-8
    velocity=data.qvel[adr:adr+3];com=np.sum(model.body_mass[:,None]*data.xipos,axis=0)/sum(model.body_mass)
    mass_matrix=np.zeros((model.nv,model.nv));mujoco.mj_fullM(model,data,mass_matrix)
    note_velocity=data.qvel[adr:adr+6];kinetic=.5*float(note_velocity@mass_matrix[adr:adr+6,adr:adr+6]@note_velocity);potential=-model.body_mass[nb]*float(model.opt.gravity@data.xpos[nb])
    return {'notePositionM':data.xpos[nb].tolist(),'noteVelocityMps':velocity.tolist(),'noteAngularVelocityRadps':data.qvel[adr+3:adr+6].tolist(),'advanceM':float(delta@direction),'normalOffsetM':float(delta@normal),'lateralOffsetM':float(delta@rotation[:,1]),'noteAxisWorld':data.xmat[nb].reshape(3,3)[:,2].tolist(),'chassisPositionM':data.xpos[cb].tolist(),'chassisYawRad':float(data.qpos[3]),'normalForcesN':normals,'noteForceWorldN':force.tolist(),'contacts':count,'axialEdgeContactCount':edge_contacts,'minimumTreadContactRadiusM':None if not count else minimum_radial,'maximumPenetrationM':deepest,'coneExcessN':cone,'forceProjectionErrorN':error,'systemComM':com.tolist(),'noteMechanicalEnergyJ':kinetic+potential,'noteContactPowerW':float(generalized[adr:adr+6]@note_velocity)}

def motor_query(config,omega,duty,voltage):
    k=1/config['backEmfRadPerSV'];res=config['windingResistanceOhm'];limit=config['phaseCurrentLimitA']
    # Positive drive or coast only: this does not invent regenerative absorption.
    amps=[max(0.,min(limit,(duty*voltage-k*w)/res)) if duty else 0. for w in omega]
    rows=[]
    for w,i in zip(omega,amps):
        terminal=k*w+i*res;torque=k*i;power=terminal*i;heat=i*i*res
        assert abs(power-heat-torque*w)<1e-8
        if i:assert terminal<=voltage+1e-8
        rows.append({'omegaRadps':w,'currentA':i,'terminalV':terminal,'torqueNm':torque,'terminalPowerW':power,'heatW':heat,'shaftPowerW':torque*w})
    return {'busPowerW':sum(x['terminalPowerW'] for x in rows),'motors':rows}

def run(args):
    root=args.project.resolve();sys.path[:0]=[str(root/'sim/mujoco'),str(root/'sim/integrated')];from native_drive import NativeDrive
    from shared_supply import solve_supply
    binding=json.loads(args.binding.read_text(encoding='utf-8'));assert sha(args.native_archive)==binding['nativeArchiveSha256']
    for name,digest in binding['inputs'].items():assert sha(root/name)==digest,name
    config=json.loads((root/'sim/component-launcher/six-drive-config.json').read_text(encoding='utf-8'))
    assert config['radiusM']==binding['wheels'][0]['radiusM']
    drive=NativeDrive(root,dt=args.dt)
    baseline_com=np.sum(drive.m.body_mass[:,None]*drive.d.xipos,axis=0)/45.
    trial=mujoco.MjModel.from_xml_string(model_xml(drive,binding,config,args,1000,0,False));td=mujoco.MjData(trial);mujoco.mj_forward(trial,td)
    trial_normals=observe(trial,td,binding)['normalForcesN'];assert min(trial_normals)>0 or args.no_contact
    # Fit a collision-enabled initial pose even for the no-contact negative control.
    if args.no_contact:
        args.no_contact=False
        trial=mujoco.MjModel.from_xml_string(model_xml(drive,binding,config,args,1000,0,False));td=mujoco.MjData(trial);mujoco.mj_forward(trial,td);trial_normals=observe(trial,td,binding)['normalForcesN']
        args.no_contact=True
    stiffness=1000*config['normalEachN']/(sum(trial_normals)/4)
    def normal_probe(k,value,segments,phase,surface=None):
        probe_args=copy.copy(args);probe_args.no_contact=False;probe_args.floating=False;probe_args.segments=segments;probe_args.ring_phase_deg=phase
        if surface is not None:probe_args.surface=surface
        probe=mujoco.MjModel.from_xml_string(model_xml(drive,binding,config,probe_args,k,0,False,damping=value))
        pd=mujoco.MjData(probe);adr=int(probe.joint('note_free').dofadr[0]);normal=np.array([-math.sin(binding['pitchRad']),0,math.cos(binding['pitchRad'])]);forces=[]
        mujoco.mj_forward(probe,pd);initial_forces=observe(probe,pd,binding)['normalForcesN']
        for sign in (-1,1):
            pd.qvel[:]=0.;pd.qvel[adr:adr+3]=sign*.001*normal;mujoco.mj_forward(probe,pd);forces.append(observe(probe,pd,binding)['normalForcesN'])
        response=sum(w['side']*(b-a)/.002 for w,a,b in zip(binding['wheels'],forces[0],forces[1]))/4
        return sum(initial_forces)/4,response
    reference_force,_=normal_probe(1000.,2*math.sqrt(1000.),128,0.,surface='capsules')
    reference_stiffness=1000.*config['normalEachN']/reference_force
    _,damping_target=normal_probe(reference_stiffness,2*math.sqrt(reference_stiffness),128,0.,surface='capsules')
    def damping_response(value):return normal_probe(stiffness,value,args.segments,args.ring_phase_deg)[1]
    measured=damping_response(100.);assert measured>0
    damping=100.*damping_target/measured
    for damping_iterations in range(6):
        matched_damping=damping_response(damping)
        if abs(matched_damping-damping_target)<1e-6:break
        damping*=damping_target/matched_damping
    assert abs(matched_damping-damping_target)<1e-6,(measured,damping,matched_damping,damping_target)
    xml=model_xml(drive,binding,config,args,stiffness,args.mu,not args.floating,damping=damping)
    drive.m=mujoco.MjModel.from_xml_string(xml);drive.d=mujoco.MjData(drive.m);m,d=drive.m,drive.d
    indices=[int(m.joint(f'launcher_{i}').dofadr[0]) for i in range(4)]
    for index in indices:d.qvel[index]=args.initial_rpm*2*math.pi/60
    mujoco.mj_forward(m,d);note_id=m.body('note').id
    assert m.nv==20 and m.neq==4 and m.nu==0 and abs(sum(m.body_mass)-45.235)<1e-10
    robot_com=sum(m.body_mass[i]*d.xipos[i] for i in range(1,m.nbody) if i!=note_id)/45.;assert np.max(np.abs(robot_com-baseline_com))<1e-12
    pins=dict(binding['inputs'],**drive.pins)
    for path in ['sim/mujoco/native_drive.py','sim/mujoco/drivetrain_bridge.py','sim/integrated/shared_supply.py']:pins[path]=sha(root/path)
    assert not args.output.exists();args.output.mkdir(parents=True)
    (args.output/'model.xml').write_text(xml+'\n',encoding='utf-8',newline='\n')
    initial=observe(m,d,binding);start=time.perf_counter();max_force=max_current=max_cone=max_projection=max_penetration=max_note_energy_error=0.;edge_count=0;last_contact=None;status='HORIZON';min_bus=args.source_voltage;max_bus_current=0.
    energy={'launcherTerminalJ':0.,'launcherHeatJ':0.,'launcherShaftJ':0.,'driveAxleJ':0.,'sourceJ':0.,'sourceHeatJ':0.,'busJ':0.,'noteContactWorkJ':0.};initial_rotor=.5*config['rotatingInertiaEachKgM2']*sum(d.qvel[i]**2 for i in indices)
    with (args.output/'trace.jsonl').open('x',encoding='utf-8',newline='\n') as stream:
        for step in range(round(args.duration/args.dt)+1):
            demand=(args.drive_duty,args.drive_duty);modes=('drive','drive') if args.drive_duty else ('coast','coast')
            rotor_omega=[float(d.qvel[i]) for i in indices]
            supply=solve_supply({'drive':lambda voltage:drive.motor_query(demand,modes,voltage)[0],'launcher':lambda voltage:motor_query(config,rotor_omega,args.duty,voltage)},args.source_voltage,args.source_resistance)
            de,torques,domega=drive.motor_query(demand,modes,supply['busV'])
            electrical=supply['ports']['launcher'];min_bus=min(min_bus,supply['busV']);max_bus_current=max(max_bus_current,supply['busCurrentA'])
            d.qfrc_applied[:]=0.
            for side in range(2):d.qfrc_applied[drive.v[f'wheel_{side}_1']]=torques[side]
            for i,row in zip(indices,electrical['motors']):d.qfrc_applied[i]=row['torqueNm']
            mujoco.mj_forward(m,d);observation=observe(m,d,binding)
            if observation['contacts']:last_contact={'step':step,'timeS':float(d.time),**observation}
            max_force=max(max_force,float(np.linalg.norm(observation['noteForceWorldN'])));max_current=max(max_current,*(x['currentA'] for x in electrical['motors']));max_cone=max(max_cone,observation['coneExcessN']);max_projection=max(max_projection,observation['forceProjectionErrorN']);max_penetration=max(max_penetration,observation['maximumPenetrationM']);edge_count+=observation['axialEdgeContactCount']
            note_error=observation['noteMechanicalEnergyJ']-initial['noteMechanicalEnergyJ']-energy['noteContactWorkJ'];max_note_energy_error=max(max_note_energy_error,abs(note_error))
            row={'step':step,'timeS':float(d.time),**observation,'noteWorkResidualJ':note_error,'launcherMotors':electrical['motors'],'driveDemand':demand,'driveBusPowerW':de['busPowerW'],'busV':supply['busV'],'busCurrentA':supply['busCurrentA'],'supplyPowerResidualW':supply['powerResidualW'],'qpos':d.qpos.tolist(),'qvel':d.qvel.tolist()}
            if abs(observation['lateralOffsetM'])>.02:status='LATERAL_DOMAIN_EXIT'
            elif observation['advanceM']<args.start-.08:status='REVERSED_80MM'
            elif observation['advanceM']>.3 and observation['contacts']==0:status='PASSED_LOCAL_LAUNCHER_DOMAIN'
            elif observation['minimumTreadContactRadiusM'] is not None and observation['minimumTreadContactRadiusM']<.04572:status='UNMODELED_TREAD_CAVITY_CONTACT'
            if step%5==0 or status!='HORIZON' or step==round(args.duration/args.dt):stream.write(json.dumps(row,allow_nan=False)+'\n')
            if status!='HORIZON' or step==round(args.duration/args.dt):break
            for key,field in [('launcherTerminalJ','terminalPowerW'),('launcherHeatJ','heatW'),('launcherShaftJ','shaftPowerW')]:energy[key]+=args.dt*sum(x[field] for x in electrical['motors'])
            energy['driveAxleJ']+=args.dt*sum(a*b for a,b in zip(torques,domega))
            energy['sourceJ']+=args.dt*supply['sourcePowerW'];energy['sourceHeatJ']+=args.dt*supply['sourceHeatW'];energy['busJ']+=args.dt*supply['busPowerW'];energy['noteContactWorkJ']+=args.dt*observation['noteContactPowerW']
            mujoco.mj_step(m,d);assert np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all()
    warnings=[int(w.number) for w in d.warning];assert not any(warnings)
    result={'schema':'native-launcher-contact-experiment/1','complete':True,'acceptance':False,'status':status,'nativeArchiveSha256':binding['nativeArchiveSha256'],'bindingSha256':sha(args.binding),'inputs':pins,'scriptSha256':sha(pathlib.Path(__file__)),'mujocoVersion':mujoco.__version__,'parameters':{'segments':args.segments,'ringPhaseDeg':args.ring_phase_deg,'dtS':args.dt,'mu':args.mu,'duty':args.duty,'driveDuty':args.drive_duty,'initialRpm':args.initial_rpm,'initialAdvanceM':args.start,'noContact':args.no_contact,'floating':args.floating,'fittedStiffness':stiffness,'targetInitialNormalPerWheelN':config['normalEachN'],'rotatingInertiaEachKgM2':config['rotatingInertiaEachKgM2'],'bodyMassEachKgHypothesis':.2,'openCircuitV':args.source_voltage,'sourceResistanceOhm':args.source_resistance,'currentLimitA':config['phaseCurrentLimitA']},'initial':initial,'final':row,'lastContact':last_contact,'initialRotorKineticJ':initial_rotor,'energyLedgerJ':energy,'maxNoteWorkResidualJ':max_note_energy_error,'minimumBusV':min_bus,'maximumBusCurrentA':max_bus_current,'maxNoteForceN':max_force,'maxPhaseCurrentA':max_current,'maxConeExcessN':max_cone,'maxForceProjectionErrorN':max_projection,'maxPenetrationM':max_penetration,'axialEdgeContactSamples':edge_count,'warningCounts':warnings,'traceSha256':sha(args.output/'trace.jsonl'),'modelSha256':sha(args.output/'model.xml'),'wallSeconds':time.perf_counter()-start,'limits':['Staged NOTE inside launcher; no acquisition, lift, transfer continuity or prescribed feed velocity','Cylinder tread exteriors omit annular cavities and all other native hard parts; no flange/structure clearance acceptance','Stiffness/normal target/friction/mass/inertia/motor/controller behavior are hypotheses, not measurements','Shared quasistatic resistive supply is an existing hypothesis; no battery transients, faults, thermal or controller validation','Chassis roll/pitch constrained; rotor torque reactions do not establish tipping or dynamic load-transfer behavior','No aerodynamic/field/target scoring model; launcher-only ejection and free flight are not a complete robot cycle']}
    result['parameters'].update(surface=args.surface,sdfPoints=args.sdf_points,sdfIterations=args.sdf_iterations,noslipIterations=args.noslip_iterations,fittedDamping=damping,targetInitialNormalDampingPerWheelNspm=damping_target,matchedInitialNormalDampingPerWheelNspm=matched_damping,referenceStiffness=reference_stiffness,dampingBasis='Preserve the original 128-segment zero-phase initial normal response at +/- 1 mm/s with gravity/friction disabled; no ejection outcome fit and no hardware calibration')
    (args.output/'receipt.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('inputs','initial','final','lastContact','limits')}))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--project',type=pathlib.Path,required=True);p.add_argument('--binding',type=pathlib.Path,required=True);p.add_argument('--native-archive',type=pathlib.Path,required=True);p.add_argument('--output',type=pathlib.Path,required=True);p.add_argument('--dt',type=float,default=.0001);p.add_argument('--segments',type=int,default=128);p.add_argument('--surface',choices=['capsules','sdf'],default='capsules');p.add_argument('--sdf-points',type=int,default=40);p.add_argument('--sdf-iterations',type=int,default=20);p.add_argument('--noslip-iterations',type=int,choices=[0,20],default=20);p.add_argument('--ring-phase-deg',type=float,default=0.);p.add_argument('--mu',type=float,default=.8);p.add_argument('--duty',type=float,default=.8);p.add_argument('--drive-duty',type=float,default=0.);p.add_argument('--initial-rpm',type=float,default=0.);p.add_argument('--start',type=float,default=-.03);p.add_argument('--duration',type=float,default=.4);p.add_argument('--source-voltage',type=float,default=12.);p.add_argument('--source-resistance',type=float,default=.02);p.add_argument('--no-contact',action='store_true');p.add_argument('--floating',action='store_true');a=p.parse_args();assert 0<a.dt<=.00025 and 0<a.duration<=2 and a.segments>=32 and a.segments%4==0 and 0<=a.ring_phase_deg<360 and 0<=a.mu<=1 and 0<=a.duty<=1 and 0<=a.drive_duty<=.3 and 0<=a.initial_rpm<=4000 and -.04<=a.start<=.04 and 6<=a.source_voltage<=13 and 0<=a.source_resistance<=.03;run(a)

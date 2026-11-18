"""Prescribed-state constitutive probes, never a simulated robot trajectory."""
import argparse,hashlib,json,math,pathlib,sys
import mujoco
import numpy as np

sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def probe(packet,project):
    sys.path.insert(0,str(project/'sim/mujoco'))
    from native_launcher_contact import observe
    binding_path=project/'sim/mujoco/evidence/native-launcher-binding-20261118.json'
    binding=json.loads(binding_path.read_text(encoding='utf-8'))
    cases={**{f'capsule-{n}':f'launcher-matched/grid-{n}' for n in [128,256,512,1024]},**{f'sdf-{n}':f'launcher-sdf-matched/points{n}' for n in [40,80,160]}}
    angle=binding['pitchRad'];direction=np.array([math.cos(angle),0,math.sin(angle)]);normal=np.array([-math.sin(angle),0,math.cos(angle)])
    rows=[];inputs={};gains={}
    for label,relative in cases.items():
        path=packet/relative;receipt=json.loads((path/'receipt.json').read_text(encoding='utf-8'))
        assert receipt['modelSha256']==sha(path/'model.xml') and receipt['bindingSha256']==sha(binding_path)
        model=mujoco.MjModel.from_xml_path(str(path/'model.xml'));data=mujoco.MjData(model)
        model.opt.gravity[:]=0.;model.opt.noslip_iterations=0
        nj=model.joint('note_free');q=int(nj.qposadr[0]);v=int(nj.dofadr[0]);nb=model.body('note').id
        rotor=[int(model.joint(f'launcher_{i}').dofadr[0]) for i in range(4)]
        initial=data.qpos.copy();inputs[label]={'receiptSha256':sha(path/'receipt.json'),'modelSha256':receipt['modelSha256'],'parameters':receipt['parameters']}
        for advance in [-.03,0.,.03,.06,.09]:
            for offset in [-.0005,0.,.0005]:
                for note_speed,rim_speed,mu in [(0.,0.,0.),(0.,0.,.8),(0.,3.,.8),(3.,3.,.8),(3.,0.,.8)]:
                    data.qpos[:]=initial;data.qpos[q:q+3]=np.array(binding['stationM'])+advance*direction+offset*normal+np.array([0,0,.001])
                    data.qvel[:]=0.;data.qvel[v:v+3]=note_speed*direction
                    data.qfrc_applied[:]=0.
                    for index in rotor:data.qvel[index]=rim_speed/binding['wheels'][0]['radiusM']
                    for i in range(4):model.geom(f'tread_{i}').friction[0]=mu
                    mujoco.mj_forward(model,data);ob=observe(model,data,binding)
                    contact_rows=[];contact_power=0.;normal_power=0.
                    for ci in range(data.ncon):
                        c=data.contact[ci];g1,g2=map(int,c.geom);b1,b2=map(int,model.geom_bodyid[[g1,g2]])
                        if nb not in (b1,b2):continue
                        force=np.zeros(6);mujoco.mj_contactForce(model,data,ci,force)
                        # Contact frame normal points geom1 -> geom2. Applied force
                        # on geom2 is +frame.T @ force, with opposite geom1 reaction.
                        wrench=c.frame.reshape(3,3).T@force[:3]
                        both=np.zeros(model.nv)
                        mujoco.mj_applyFT(model,data,wrench,np.zeros(3),c.pos,b2,both)
                        mujoco.mj_applyFT(model,data,-wrench,np.zeros(3),c.pos,b1,both)
                        cp=float(both@data.qvel);contact_power+=cp
                        normal_only=np.zeros(model.nv);nf=c.frame.reshape(3,3)[0]*force[0]
                        mujoco.mj_applyFT(model,data,nf,np.zeros(3),c.pos,b2,normal_only);mujoco.mj_applyFT(model,data,-nf,np.zeros(3),c.pos,b1,normal_only)
                        normal_power+=float(normal_only@data.qvel)
                        contact_rows.append({'wheel':mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,b1 if b2==nb else b2),'positionM':c.pos.tolist(),'normal':c.frame[:3].tolist(),'penetrationM':-float(c.dist),'forceContactN':force[:3].tolist()})
                    rows.append({'case':label,'advanceM':advance,'normalOffsetM':offset,'noteSpeedMps':note_speed,'rimSpeedMps':rim_speed,'mu':mu,'normalForcesN':ob['normalForcesN'],'noteForceWorldN':ob['noteForceWorldN'],'longitudinalForceN':float(np.array(ob['noteForceWorldN'])@direction),'contactCount':len(contact_rows),'contactPowerBothBodiesW':contact_power,'normalPowerBothBodiesW':normal_power,'frictionPowerBothBodiesW':contact_power-normal_power,'rotorReactionNm':[float(data.qfrc_constraint[i]) for i in rotor],'contacts':contact_rows})
        print(label,'done',flush=True)
    return {'schema':'launcher-prescribed-contact-response/1','acceptance':False,'scriptSha256':sha(pathlib.Path(__file__)),'launcherRunnerSha256':sha(project/'sim/mujoco/native_launcher_contact.py'),'bindingSha256':sha(binding_path),'mujocoVersion':mujoco.__version__,'inputs':inputs,'rows':rows,'limits':['Prescribed states only: these forces are not an integrated acquisition or shooting trajectory','Same saved model source and initial calibration, with gravity and optional NoSlip disabled','No hardware calibration; contact normal fit is only local to the original initial pose']}

if __name__=='__main__':
    p=argparse.ArgumentParser()
    for name in ['packet','project','output']:p.add_argument('--'+name,type=pathlib.Path,required=True)
    a=p.parse_args();assert not a.output.exists();r=probe(a.packet,a.project);a.output.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps({'rows':len(r['rows']),'outputSha256':sha(a.output)}))

"""Reject a discontinuous splice of separately staged experiments.

This reads saved models/states. It does not synthesize the missing lift/transfer.
"""
import argparse,hashlib,json,math,pathlib
import mujoco,numpy as np
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def state(folder,last):
    receipt=json.loads((folder/'receipt.json').read_text(encoding='utf-8'))
    assert receipt['complete'] and sha(folder/'model.xml')==receipt['modelSha256'] and sha(folder/'trace.jsonl')==receipt['traceSha256']
    row=receipt['final'] if last else json.loads((folder/'trace.jsonl').read_text().splitlines()[0])
    model=mujoco.MjModel.from_xml_path(str(folder/'model.xml'));data=mujoco.MjData(model);data.qpos[:]=row['qpos'];data.qvel[:]=row['qvel'];mujoco.mj_forward(model,data)
    note=model.body('note').id;chassis=model.body('chassis').id;rotation=data.xmat[chassis].reshape(3,3)
    relative=rotation.T@(data.xpos[note]-data.xpos[chassis]);axis=rotation.T@data.xmat[note].reshape(3,3)[:,2]
    jacp=np.zeros((3,model.nv));jacr=np.zeros_like(jacp);mujoco.mj_jacBody(model,data,jacp,jacr,chassis);cv=jacp@data.qvel;cw=jacr@data.qvel
    mujoco.mj_jacBody(model,data,jacp,jacr,note);nv=jacp@data.qvel
    relv=rotation.T@(nv-cv-np.cross(cw,data.xpos[note]-data.xpos[chassis]))
    return {'receiptSha256':sha(folder/'receipt.json'),'nativeArchiveSha256':receipt['nativeArchiveSha256'],'modelSha256':receipt['modelSha256'],'traceSha256':receipt['traceSha256'],'timeS':row['timeS'],'relativeNoteM':relative.tolist(),'relativeNoteVelocityMps':relv.tolist(),'relativeNoteAxis':axis.tolist(),'nv':model.nv,'noteMassKg':float(model.body_mass[note]),'joints':[mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_JOINT,i) for i in range(model.njnt)]}

def compare(a,b):
    dp=np.array(b['relativeNoteM'])-a['relativeNoteM'];dv=np.array(b['relativeNoteVelocityMps'])-a['relativeNoteVelocityMps'];angle=math.acos(float(np.clip(np.dot(a['relativeNoteAxis'],b['relativeNoteAxis']),-1,1)))
    return {'positionJumpM':float(np.linalg.norm(dp)),'positionDifferenceM':dp.tolist(),'relativeVelocityJumpMps':float(np.linalg.norm(dv)),'noteAxisJumpRad':angle,'sameModel':a['modelSha256']==b['modelSha256'],'sameNativeArchive':a['nativeArchiveSha256']==b['nativeArchiveSha256'],'sameJointSet':a['joints']==b['joints'],'gravityPotentialDifferenceJ':float(b['noteMassKg']*9.80665*dp[2]),'directSplicePermitted':bool(np.linalg.norm(dp)<1e-6 and np.linalg.norm(dv)<1e-6 and angle<1e-6 and a['modelSha256']==b['modelSha256'] and a['joints']==b['joints'])}

if __name__=='__main__':
    p=argparse.ArgumentParser()
    for name in ['retention','launcher','output']:p.add_argument('--'+name,type=pathlib.Path,required=True)
    a=p.parse_args();assert not a.output.exists();previous=state(a.retention,True);following=state(a.launcher,False);control=compare(previous,previous);assert control['directSplicePermitted'];result=compare(previous,following)
    assert not result['directSplicePermitted']
    receipt={'schema':'native-cycle-handoff-boundary/1','acceptance':False,'scriptSha256':sha(pathlib.Path(__file__)),'retention':previous,'launcher':following,'identityControl':control,'comparison':result,'missing':['One continuous physical state including intake, lift/pitch and launcher joints/motor states','Attained floor acquisition/index and subsequent transport, rather than a staged carry-height fixture','Native-backed articulated clearance and calibrated contact over the missing lift/transfer'],'limits':['A state-composition rejection, not a failed physical transfer experiment','Chassis-frame comparison allows a rigid world-frame change; it does not teleport the NOTE or supply the missing potential energy','Potential difference covers NOTE gravity only, not carriage mass, losses or necessary actuator power']}
    a.output.write_text(json.dumps(receipt,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps(result))

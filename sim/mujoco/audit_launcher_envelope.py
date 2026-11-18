"""Finite saved-trajectory witnesses against an all-phase bare-wheel envelope.

Overlap with a containing cylinder is UNKNOWN, never proof of material contact.
"""
import argparse,hashlib,json,math,pathlib,xml.etree.ElementTree as ET
import mujoco
import numpy as np

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def audit(run,binding_path,envelope_path,project):
    receipt=json.loads((run/'receipt.json').read_text(encoding='utf-8'))
    binding=json.loads(binding_path.read_text(encoding='utf-8'));envelope=json.loads(envelope_path.read_text(encoding='utf-8'))
    assert receipt['bindingSha256']==sha(binding_path) and receipt['traceSha256']==sha(run/'trace.jsonl') and receipt['modelSha256']==sha(run/'model.xml')
    for rel,digest in binding['inputs'].items():assert sha(project/rel)==digest
    tx=json.loads((project/'cad/components/a2-transmission.json').read_text(encoding='utf-8'))['serviceInterfaces']['tread']
    source=json.loads((project/'cad/components/a2-wheel-cartridge.json').read_text(encoding='utf-8'))['sources']['wheel']
    assert source['sha256']==envelope['sourceSha256']
    axial=envelope['axialBoundsMm'];radius=envelope['radiusUpperMm']/1000;half=(axial[1]-axial[0])/2000
    offset=(-tx['assemblyTreadCenterOffsetMm']-15.875+(axial[0]+axial[1])/2)/1000
    # Direct signed-distance negative/positive controls independent of saved state.
    cx=f'<mujoco><worldbody><geom name="wheel" type="cylinder" size="{radius} {half}" quat=".7071067811865476 .7071067811865476 0 0"/><geom name="inside" type="sphere" size=".0254"/><geom name="outside" type="sphere" size=".0254" pos=".2 0 0"/></worldbody></mujoco>'
    cm=mujoco.MjModel.from_xml_string(cx);cd=mujoco.MjData(cm);mujoco.mj_forward(cm,cd)
    controls={'overlapM':mujoco.mj_geomDistance(cm,cd,0,1,1.,None),'separatedM':mujoco.mj_geomDistance(cm,cd,0,2,1.,None)}
    assert controls['overlapM']<-.01 and controls['separatedM']>.1
    tree=ET.fromstring((run/'model.xml').read_text(encoding='utf-8'))
    for geom in tree.findall('.//geom'):geom.set('contype','0');geom.set('conaffinity','0')
    for i in range(4):
        body=tree.find(f"worldbody/body[@name='chassis']/body[@name='launcher_{i}']")
        ET.SubElement(body,'geom',name=f'bare_envelope_{i}',type='cylinder',size=f'{radius} {half}',pos=f'0 {offset} 0',quat='.7071067811865476 .7071067811865476 0 0',density='0',contype='0',conaffinity='0')
    sagitta=binding['noteMajorRadiusM']*(1-math.cos(math.pi/receipt['parameters']['segments']))
    for geom in tree.findall("worldbody/body[@name='note']/geom"):geom.set('size',str(binding['noteTubeRadiusM']+sagitta+1e-7))
    model=mujoco.MjModel.from_xml_string(ET.tostring(tree,encoding='unicode'));data=mujoco.MjData(model)
    note_ids=np.array([model.geom(f'note_{i}').id for i in range(receipt['parameters']['segments'])]);bare_ids=[model.geom(f'bare_envelope_{i}').id for i in range(4)]
    rows=[];queries=0
    for line in (run/'trace.jsonl').read_text(encoding='utf-8').splitlines():
        state=json.loads(line);data.qpos[:]=state['qpos'];data.qvel[:]=0.;mujoco.mj_forward(model,data);gaps=[]
        for gid in bare_ids:
            clip=.005
            distances=np.linalg.norm(data.geom_xpos[note_ids]-data.geom_xpos[gid],axis=1)
            possible=note_ids[distances<=model.geom_rbound[note_ids]+model.geom_rbound[gid]+clip]
            values=[float(mujoco.mj_geomDistance(model,data,gid,int(nid),clip,None)) for nid in possible];queries+=len(values)
            gap=min(values,default=clip);assert math.isfinite(gap);gaps.append(gap)
        rows.append({'step':state['step'],'timeS':state['timeS'],'minimumGapByWheelM':gaps,'classified':'SEPARATED_CONTAINING_ENVELOPE' if min(gaps)>1e-7 else 'UNKNOWN_ENVELOPE_CONTACT'})
    unresolved=[r for r in rows if r['classified']=='UNKNOWN_ENVELOPE_CONTACT']
    return {'schema':'native-launcher-envelope-witnesses/1','complete':True,'acceptance':False,'bindingSha256':sha(binding_path),'nativeEnvelopeSha256':sha(envelope_path),'sourceReceiptSha256':sha(run/'receipt.json'),'sourceTraceSha256':sha(run/'trace.jsonl'),'scriptSha256':sha(pathlib.Path(__file__)),'mujocoVersion':mujoco.__version__,'controls':controls,'torusCenterlineSagittaM':sagitta,'extraMarginM':1e-7,'queryCount':queries,'witnessCount':len(rows),'unresolvedWitnessCount':len(unresolved),'minimumGapM':min(min(r['minimumGapByWheelM']) for r in rows),'rows':rows,'limits':['Finite saved states only, not continuous clearance between samples','Containing cylinder is all-phase but any overlap remains unknown; no favorable source clocking substituted','Capsules inflated by centerline sagitta plus margin contain the nominal rigid torus, not a measured deformed NOTE','Only four bare-wheel source envelopes; full robot hard parts, fasteners, installed registration and physical compliance remain unqualified']}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=pathlib.Path,required=True);p.add_argument('--binding',type=pathlib.Path,required=True);p.add_argument('--envelope',type=pathlib.Path,required=True);p.add_argument('--project',type=pathlib.Path,required=True);p.add_argument('--output',type=pathlib.Path,required=True);a=p.parse_args();assert not a.output.exists();r=audit(a.run,a.binding,a.envelope,a.project);a.output.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps({k:v for k,v in r.items() if k not in ('rows','limits')}))

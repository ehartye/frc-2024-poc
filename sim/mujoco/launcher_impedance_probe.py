"""Check a derived quadrature weighting with exact duplicated-contact controls.

Inference from MuJoCo's documented R=(1-d)/d*Ahat and direct k=K/d,b=B/d
for constant impedance. Duplicating each constraint q times requires R'=qR
and unchanged reference acceleration; therefore d'=d/(d+q*(1-d)),
K'=K*d'/d and B'=B*d'/d. This is a solver-law hypothesis, not calibration.
"""
import argparse,copy,hashlib,json,math,pathlib,sys,xml.etree.ElementTree as ET
import mujoco,numpy as np
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def adjusted_xml(xml,scale,duplicate=1):
    root=ET.fromstring(xml);option=root.find('option');option.set('noslip_iterations','0');option.set('gravity','0 0 0')
    body=root.find("worldbody/body[@name='note']")
    for geom in list(body.findall('geom')):
        for i in range(1,duplicate):
            clone=copy.deepcopy(geom);clone.set('name',geom.get('name')+f'_duplicate_{i}');body.append(clone)
    for geom in root.findall("worldbody/body[@name='chassis']/body/geom"):
        if not geom.get('name','').startswith('tread_'):continue
        solimp=list(map(float,geom.get('solimp').split()));assert solimp[0]==solimp[1]
        d=solimp[0];new_d=d/(d+scale*(1-d));solimp[0]=solimp[1]=new_d
        geom.set('solimp',' '.join(map(str,solimp)));geom.set('solref',' '.join(str(float(v)*new_d/d) for v in geom.get('solref').split()))
    return ET.tostring(root,encoding='unicode')

def main(a):
    sys.path.insert(0,str(a.project/'sim/mujoco'));from native_launcher_contact import observe
    bp=a.project/'sim/mujoco/evidence/native-launcher-binding-20261118.json';binding=json.loads(bp.read_text());angle=binding['pitchRad'];direction=np.array([math.cos(angle),0,math.sin(angle)]);normal=np.array([-math.sin(angle),0,math.cos(angle)])
    base=a.packet/'launcher-matched/grid-128';receipt=json.loads((base/'receipt.json').read_text());assert sha(base/'model.xml')==receipt['modelSha256'];xml=(base/'model.xml').read_text();rows=[]
    for duplication in [1,2,4,8]:
        for weighted in ([True] if duplication==1 else [False,True]):
            model=mujoco.MjModel.from_xml_string(adjusted_xml(xml,duplication if weighted else 1,duplication));data=mujoco.MjData(model);initial=data.qpos.copy();q=int(model.joint('note_free').qposadr[0]);v=int(model.joint('note_free').dofadr[0]);rotors=[int(model.joint(f'launcher_{i}').dofadr[0]) for i in range(4)]
            for advance in [-.03,.03]:
                for offset in [0.,.0005]:
                    for speed,rim,mu in [(0.,0.,0.),(0.,3.,0.),(0.,3.,.8),(3.,3.,.8),(3.,0.,.8)]:
                        data.qpos[:]=initial;data.qpos[q:q+3]=np.array(binding['stationM'])+advance*direction+offset*normal+[0,0,.001];data.qvel[:]=0;data.qvel[v:v+3]=speed*direction
                        for i in rotors:data.qvel[i]=rim/.04826
                        for i in range(4):model.geom(f'tread_{i}').friction[0]=mu
                        mujoco.mj_forward(model,data);ob=observe(model,data,binding)
                        rows.append({'duplicates':duplication,'weighted':weighted,'advanceM':advance,'normalOffsetM':offset,'noteSpeedMps':speed,'rimSpeedMps':rim,'mu':mu,'normalForcesN':ob['normalForcesN'],'noteForceWorldN':ob['noteForceWorldN'],'qacc':data.qacc.tolist(),'qfrcConstraint':data.qfrc_constraint.tolist(),'contacts':ob['contacts']})
            print(duplication,weighted,'done',flush=True)
    reference=[r for r in rows if r['duplicates']==1];comparisons=[]
    keys=['advanceM','normalOffsetM','noteSpeedMps','rimSpeedMps','mu']
    for n in [2,4,8]:
        for weighted in [False,True]:
            values=[r for r in rows if r['duplicates']==n and r['weighted']==weighted];errors={k:0. for k in ['normalForcesN','noteForceWorldN','qacc','qfrcConstraint']}
            for row,ref in zip(values,reference):
                assert all(row[k]==ref[k] for k in keys)
                for k in errors:errors[k]=max(errors[k],max(abs(x-y) for x,y in zip(row[k],ref[k])))
            comparisons.append({'duplicates':n,'weighted':weighted,'maximumAbsoluteDifferences':errors})
    out={'schema':'launcher-duplicate-contact-invariance/1','acceptance':False,'scriptSha256':sha(pathlib.Path(__file__)),'sourceModelSha256':receipt['modelSha256'],'runnerSha256':sha(a.project/'sim/mujoco/native_launcher_contact.py'),'bindingSha256':sha(bp),'mujocoVersion':mujoco.__version__,'comparisons':comparisons,'rows':rows,'limits':['Prescribed-state solver test; no dynamics or hardware acceptance','Exact duplicate-invariance is a necessary control, not spatial quadrature convergence or constitutive validation']}
    a.output.write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps(comparisons))

if __name__=='__main__':
    p=argparse.ArgumentParser()
    for name in ['project','packet','output']:p.add_argument('--'+name,type=pathlib.Path,required=True)
    a=p.parse_args();assert not a.output.exists();main(a)

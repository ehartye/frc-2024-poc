"""Predeclared pose/velocity comparisons for uniform-ring constraint quadrature."""
import argparse,hashlib,json,math,pathlib,sys,types
import mujoco,numpy as np
import native_launcher_quadrature as runner
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main(a):
    sys.path.insert(0,str(a.project/'sim/mujoco'));from native_drive import NativeDrive
    contract=json.loads(a.contract.read_text());bp=a.project/'sim/mujoco/evidence/native-launcher-binding-20261118.json';binding=json.loads(bp.read_text());config=json.loads((a.project/'sim/component-launcher/six-drive-config.json').read_text());drive=NativeDrive(a.project,dt=.0001)
    angle=binding['pitchRad'];direction=np.array([math.cos(angle),0,math.sin(angle)]);normal=np.array([-math.sin(angle),0,math.cos(angle)])
    args=types.SimpleNamespace(segments=128,noslip_iterations=0,floating=False,start=-.03,surface='capsules',no_contact=False,ring_phase_deg=0.)
    m=mujoco.MjModel.from_xml_string(runner.model_xml(drive,binding,config,args,1000.,0.,False,damping=0.));d=mujoco.MjData(m);mujoco.mj_forward(m,d);kref=1000.*25/(sum(runner.observe(m,d,binding)['normalForcesN'])/4);bref=2*math.sqrt(kref)
    rows=[]
    for n,phase in [(n,0.) for n in [4096,8192,16384]]+[(8192,180/8192),(16384,180/16384)]:
        args.segments=n;args.ring_phase_deg=phase;impedance=.9/(.9+(n/128)*.1)
        model=mujoco.MjModel.from_xml_string(runner.model_xml(drive,binding,config,args,kref*impedance/.9,.8,False,damping=bref*impedance/.9));data=mujoco.MjData(model);initial=data.qpos.copy();q=int(model.joint('note_free').qposadr[0]);v=int(model.joint('note_free').dofadr[0]);rotors=[int(model.joint(f'launcher_{i}').dofadr[0]) for i in range(4)]
        for advance in contract['staticFineComparison']['posesAdvanceM']:
            for offset in contract['staticFineComparison']['normalOffsetsM']:
                for speed,rim,mu in [(0.,0.,0.),(0.,0.,.8),(0.,3.,.8),(3.,3.,.8),(3.,0.,.8)]:
                    data.qpos[:]=initial;data.qpos[q:q+3]=np.array(binding['stationM'])+advance*direction+offset*normal+[0,0,.001];data.qvel[:]=0;data.qvel[v:v+3]=speed*direction
                    for i in rotors:data.qvel[i]=rim/.04826
                    for i in range(4):model.geom(f'tread_{i}').friction[0]=mu
                    mujoco.mj_forward(model,data);ob=runner.observe(model,data,binding)
                    rows.append({'segments':n,'phaseDeg':phase,'advanceM':advance,'normalOffsetM':offset,'noteSpeedMps':speed,'rimSpeedMps':rim,'mu':mu,'normalForcesN':ob['normalForcesN'],'noteForceWorldN':ob['noteForceWorldN'],'rotorReactionNm':[float(data.qfrc_constraint[i]) for i in rotors],'contacts':ob['contacts'],'maxPenetrationM':ob['maximumPenetrationM']})
        print(n,phase,'done',flush=True)
    keys=['advanceM','normalOffsetM','noteSpeedMps','rimSpeedMps','mu'];comparisons=[]
    for first,second in [((4096,0.),(8192,0.)),((8192,0.),(16384,0.)),((8192,0.),(8192,180/8192)),((16384,0.),(16384,180/16384))]:
        aa=[r for r in rows if (r['segments'],r['phaseDeg'])==first];bb=[r for r in rows if (r['segments'],r['phaseDeg'])==second];errors={key:0. for key in ['normalForcesN','noteForceWorldN','rotorReactionNm']};worst={}
        for x,y in zip(aa,bb):
            assert all(x[k]==y[k] for k in keys)
            for field in errors:
                floor=.01 if field=='rotorReactionNm' else .1
                for i,(u,v) in enumerate(zip(x[field],y[field])):
                    error=abs(u-v)/max(floor,.01*max(abs(u),abs(v)))
                    if error>errors[field]:errors[field]=error;worst[field]={**{k:x[k] for k in keys},'component':i,'firstValue':u,'secondValue':v}
        comparisons.append({'first':first,'second':second,'maxToleranceRatio':errors,'passed':max(errors.values())<=1,'worst':worst})
    result={'schema':'launcher-quadrature-response/1','acceptance':False,'contractSha256':sha(a.contract),'scriptSha256':sha(pathlib.Path(__file__)),'runnerSha256':sha(pathlib.Path(runner.__file__)),'bindingSha256':sha(bp),'mujocoVersion':mujoco.__version__,'referenceK':kref,'referenceB':bref,'comparisons':comparisons,'rows':rows}
    a.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps(comparisons))

if __name__=='__main__':
    p=argparse.ArgumentParser()
    for name in ['project','contract','output']:p.add_argument('--'+name,type=pathlib.Path,required=True)
    a=p.parse_args();assert not a.output.exists();main(a)

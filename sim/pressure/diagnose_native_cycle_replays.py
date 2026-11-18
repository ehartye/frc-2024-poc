import argparse,copy,hashlib,json,pathlib,sys
import numpy as np
p=argparse.ArgumentParser()
for n in ['project','source','binding','contract','output']:p.add_argument('--'+n,type=pathlib.Path,required=True)
a=p.parse_args();assert not a.output.exists();sys.path[:0]=[str(a.project/f'sim/{x}') for x in ['pressure','integrated','drivetrain','actuation','acquisition']];sys.path.insert(0,str(pathlib.Path(__file__).parent))
from native_static_cycle import NativeStaticCycle
load=lambda p:json.loads(p.read_bytes());r=load(a.source);c=load(a.contract);m=NativeStaticCycle(load(a.project/'sim/drivetrain/config.json'),load(a.project/'sim/acquisition/free-planar-config.json')['model'],load(a.project/'cad/assembly/intake_components.json'),load(a.binding),load(a.project/'sim/actuation/config.json'),c);rows=r['trace'];selected={0,len(rows)-1,next(i for i,row in enumerate(rows) if row['contactCount'])}|{j['step'] for j in r['joins']};cases=[]
def differences(x,y,path=''):
 if isinstance(x,dict):return sum((differences(x[k],y[k],path+'/'+k) for k in x),[])
 if isinstance(x,list):return sum((differences(q,s,path+'/'+str(i)) for i,(q,s) in enumerate(zip(x,y))),[])
 return [(path,float(abs(x-y)))] if isinstance(x,(int,float,np.number)) and isinstance(y,(int,float,np.number)) else []
for i in sorted(selected):
 m.s=copy.deepcopy(r['initial'] if i==0 else rows[i-1]['state']);m.ledger=copy.deepcopy(m.s['ledger']);mass=m.mass(m.s);command=r['events'][i//round(.01/r['stepS'])]['command'];again=m.step(r['stepS'],command);expected=rows[i];dv=m.velocity(again['state'])-m.velocity(expected['state']);metric=float(.5*dv@mass@dv);bound=2*(max(0.,again['friction']['gapJ'])+max(0.,expected['friction']['gapJ']));changes=sorted(differences(again['state'],expected['state']),key=lambda x:x[1],reverse=True);energy=max(v for _,v in differences(again['energyJ'],expected['energyJ']));cases.append(dict(step=i+1,largestStateDifferences=changes[:6],maximumEnergyDifferenceJ=energy,originalStateBoundPassed=changes[0][1]<c['coldReplayBounds']['maximumVelocityDifference'],originalEnergyBoundPassed=energy<c['coldReplayBounds']['maximumEnergyDifferenceJ'],massMetricVelocityDifferenceJ=metric,twiceSumSolverGapsJ=bound,massMetricBoundPassed=metric<=bound+1e-18,coldIterations=again['friction']['iterations'],warmIterations=expected['friction']['iterations']))
out=dict(schema='native-cycle-cold-replay-diagnosis/1',producerSha256=hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest(),sourceSha256=hashlib.sha256(a.source.read_bytes()).hexdigest(),cases=cases,limits=['Original componentwise and energy tolerances are preserved and explicitly classified','Mass metric is diagnostic: two convex QP gap certificates bound velocity separation by twice their sum; no physical model change','This diagnostic does not replace a failed originally declared gate'])
a.output.write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))

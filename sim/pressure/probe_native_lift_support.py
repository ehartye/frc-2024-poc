"""Local pressure refinement and an optimistic friction-cone support bound.

Prescribed states are sampled from an attained trajectory. This is neither a
new trajectory nor a static-friction solution or native clearance check.
"""
import argparse,copy,hashlib,json,pathlib,sys
import numpy as np
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def run(a):
 sys.path[:0]=[str(a.project/f'sim/{x}') for x in ['pressure','integrated','drivetrain','actuation','acquisition']]
 import native_pressure_lift as lift
 load=lambda p:json.loads(p.read_bytes())
 source=load(a.source);r=load(a.trajectory);c=load(a.contract);binding=load(a.binding)
 m=lift.NativePressureLift(source['final'],load(a.project/'sim/drivetrain/config.json'),load(a.project/'sim/acquisition/free-planar-config.json')['model'],load(a.project/'cad/assembly/intake_components.json'),binding,load(a.project/'sim/actuation/config.json'),c)
 rows=r['trace'];peak=max(rows,key=lambda x:x['state']['noteZ']);air=[x for x in rows if x['floorForceN'][2]==0 and x['state']['noteZ']>.025401];ret=next(x for x in rows if x['timeS']>peak['timeS'] and x['floorForceN'][2]>0)
 picks=[('entry',r['initial']),('first-airborne',air[0]['state']),('peak-rise',peak['state']),('before-floor-return',rows[rows.index(ret)-1]['state']),('floor-return',ret['state']),('end',r['final'])];samples=[]
 for label,s in picks:
  levels=[]
  for n in [512,1024,2048]:
   m.c['resolution']=[n,n];con=m.contact(s,{});p=con['pressure'];nz=p['normalWorld'][:,2];normal_z=float(np.sum(p['normalN']*nz));bound=normal_z+float(np.sum(m.nc['beltMu']*p['normalN']*np.sqrt(np.maximum(0,1-nz*nz))))
   levels.append(dict(resolution=[n,n],normalN=float(sum(p['normalN'])),normalVerticalN=normal_z,actualContactVerticalN=con['q']['noteForceWorldN'][2],optimisticContactVerticalUpperBoundN=bound,weightN=m.nc['pieceMassKg']*9.80665,upperBoundBelowWeight=bound<m.nc['pieceMassKg']*9.80665,contactForceN=con['q']['noteForceWorldN'],potentialJ=p['potentialJ'],features=p['features'],workResidualW=con['q']['workResidualW']))
  f,g=levels[-2:];delta=max(abs(x-y) for x,y in zip(f['contactForceN'],g['contactForceN']));samples.append(dict(label=label,elapsedS=s['timeS']-r['initial']['timeS'],noteRiseM=s['noteZ']-m.z0,pivotRiseM=s['zPivot']-m.z0,levels=levels,fineMaximumForceDifferenceN=delta,finePotentialDifferenceJ=abs(f['potentialJ']-g['potentialJ'])))
 floor=[];m.c['resolution']=[1024,1024]
 for label,s in picks:
  levels=[]
  for n in [16,32,64]:
   m.c['floorPoints']=n;con=m.contact(s,{})
   levels.append(dict(points=n,forceN=con['floorForce'].tolist(),potentialJ=con['floorEnergy'],normalHeatW=con['floorHeat'],frictionHeatW=con['floorSlip']))
  floor.append(dict(label=label,levels=levels))
 return dict(schema='native-lift-local-support/1',inputs={label:sha(p) for label,p in [('producer',pathlib.Path(__file__)),('model',pathlib.Path(lift.__file__)),('sourceReceipt',a.source),('trajectoryReceipt',a.trajectory),('contract',a.contract),('binding',a.binding)]},samples=samples,floorSamples=floor,acceptance=False,limits=['Prescribed-state local refinement only; no full spatial trajectory convergence','Optimistic upper bound allows arbitrary tangential friction within each Coulomb cone independently; attainable force may be smaller','An upper bound below weight excludes static support only at that fixed model geometry; it is not a hardware or design-wide impossibility proof','Regularized friction has no static sticking state; material, floor, mass and installed transmission remain uncalibrated','No Fusion access or lifted native whole-assembly clearance'])
if __name__=='__main__':
 p=argparse.ArgumentParser()
 for n in ['project','source','trajectory','binding','contract','output']:p.add_argument('--'+n,type=pathlib.Path,required=True)
 a=p.parse_args();assert not a.output.exists();r=run(a);a.output.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps({'output':str(a.output),'samples':[dict(label=x['label'],forceDifferenceN=x['fineMaximumForceDifferenceN'],supportBoundN=x['levels'][-1]['optimisticContactVerticalUpperBoundN'],weightN=x['levels'][-1]['weightN']) for x in r['samples']]}))

"""Bind the actual indexed endpoint to finite belt witnesses; no attachment."""
import pathlib,json,hashlib,sys,math,argparse
R=pathlib.Path(__file__).resolve().parents[2];sys.path[:0]=[str(R/'cad/assembly'),str(R/'sim/integrated')]
import belt_contact_geometry as belt
from patch_registration import evaluate,rotation,matvec
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def run(output):
 if output.exists():raise ValueError('fresh output required')
 source=R/'sim/integrated/evidence/fast-regulated-index.json';r=json.loads(source.read_text());files=[source,pathlib.Path(__file__),R/'sim/integrated/patch_registration.py']
 pins=dict(r['inputs'],**{p.relative_to(R).as_posix():sha(p) for p in files})
 if any(sha(R/p)!=h for p,h in pins.items()):raise ValueError('stale source')
 c=json.loads((R/'cad/assembly/intake_components.json').read_text());g=belt.geometry(c);f=r['final'];d=f['chassis'];n=f['note'];cb=rotation(d['heading'],0.);nb=rotation(n['yaw'],0.)
 stage=matvec(cb,[c['stageCenterX']/1000,0.,c['floorCenterZ']/1000]);origin=[d['x']+stage[0],d['y']+stage[1],stage[2]];center=[*n['p'],c['floorCenterZ']/1000];offset=f['relativeRobotXY'][0]-c['stageCenterX']/1000;points=[]
 for motor,side in enumerate([-1,1]):
  for dx in [-.012,-.006,0.,.006,.012]:
   for z in [-.003,0.,.003]:
    p=[offset+dx,side*c['pinch']['innerFaceYAbs']/1000,z];q=belt.query([v*1000 for v in p],side,g);wall=q['outerSidewall']
    if abs(q['signedDistanceToMaterialMm'])>1e-8 or wall['feature']!='straight' or not wall['withinAxialHeight'] or not wall['noteFacingHalf']:raise ValueError('outside finite authored belt face')
    points.append(dict(motor=motor,cradlePointM=p,normalCradle=[0.,-side,0.]))
 ev=evaluate(points,origin,cb,center,nb,c['note']['majorRadius']/1000,c['note']['tubeRadius']/1000)
 if not ev['allNominalWitnessesInside']:raise ValueError('initial footprint unsupported')
 result=dict(schema='frc-indexed-patch-registration/1',inputs=pins,timeS=f['timeS'],offsetM=offset,preservedState=dict(chassis=d,note=n),cradleOriginWorldM=origin,cradleBasis=cb,noteCenterWorldM=center,noteBasis=nb,cradlePoints=points,evaluation=ev,acceptance=False,limits=['Finite source-sized face witnesses, not measured pressure or holding. One total normal budget per side, not per point.','Initial Z velocity and pitch are zero because preceding model explicitly constrains the NOTE flat on the floor; XY/yaw/rotor state remains unchanged.','Runtime must reevaluate every witness under actual transforms; any lost witness is unknown in this fixed-footprint model, not an ideal attachment.','No shear history is initialized. Memoryless pressure/friction distribution and finite actuator response remain hypotheses.','Other robot/guide contact, bearing retention and whole-body motion remain separate gates.'])
 if any(sha(R/p)!=h for p,h in pins.items()):raise ValueError('changed inputs')
 output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');return result
if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('--output',type=pathlib.Path,required=True);x=a.parse_args();print(run(x.output)['offsetM'])

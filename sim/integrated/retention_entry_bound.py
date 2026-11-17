"""Necessary flat-entry side-friction bound, not a lift/retention simulation."""
import math,json,pathlib,hashlib,sys,argparse
R=pathlib.Path(__file__).resolve().parents[2]
sys.path[:0]=[str(R/'sim/integrated'),str(R/'sim/acquisition')]
from coupled_acquisition import acquisition_rates
import free_geometry,obstacle_gate
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def static_bound(normals,mu,mass,gravity=9.80665):
 if len(normals)!=2 or any(type(x) not in (int,float) or not math.isfinite(x) or x<0 for x in [*normals,mu]) or type(gravity) not in (int,float) or not math.isfinite(gravity) or gravity<=0 or type(mass) not in (int,float) or not math.isfinite(mass) or mass<=0:raise ValueError('finite two-side force domain')
 capacity=mu*sum(normals);weight=mass*gravity
 return dict(maxVerticalFrictionN=capacity,weightN=weight,marginN=capacity-weight,necessaryStaticConditionMet=capacity>=weight,retentionAccepted=False)

def run(source,output):
 if output.exists():raise ValueError('fresh output required')
 r=json.loads(source.read_text());pins=dict(r['inputs']);pins[source.relative_to(R).as_posix()]=sha(source)
 for name in ['sim/integrated/retention_entry_bound.py']:pins[name]=sha(R/name)
 if any(sha(R/p)!=h for p,h in pins.items()):raise ValueError('stale input')
 f=r['final'];c=json.loads((R/'sim/acquisition/free-planar-config.json').read_text())['model'];cad=json.loads((R/'cad/assembly/intake_components.json').read_text());obs=json.loads((R/'sim/acquisition/free-obstacles.json').read_text())
 geometry=free_geometry.make_geometry(cad,obstacle_gate.make_gate(obs,cad['note']['majorRadius'],cad['note']['tubeRadius'],cad['floorCenterZ']))
 # Recompute at actual END state; trace contact rows themselves are midpoint records.
 rates=acquisition_rates(f['note'],f['chassis'],c,geometry,f['busV'],f['controller']['pinchDemands'])
 if len(rates['contacts'])!=2 or set(x['motor'] for x in rates['contacts'])!={0,1}:raise ValueError('two-sided end contact required')
 normal=[x['normalN'] for x in rates['contacts']];delta=[x['penetrationM'] for x in rates['contacts']]
 result=dict(schema='frc-retention-entry-necessary-bound/1',inputs=pins,timeS=f['timeS'],preservedState=dict(chassis=f['chassis'],note=f['note']),endContact=rates['contacts'],bound=static_bound(normal,c['beltMu'],c['pieceMassKg']),stiffnessScenarios=[dict(stiffnessNpm=k,**static_bound([k*d for d in delta],c['beltMu'],c['pieceMassKg'])) for k in [1000.,2000.,4000.,8000.]],acceptance=False,limits=['Necessary flat-entry static force bound only; all available friction is optimistically assigned vertically, ignoring longitudinal force and torque requirements.','No pitch/lift advancement or ideal attachment. Actual world velocity/spin and rotor state remain nonzero.','Stiffness and friction are hypothetical; changing a numerical stiffness is not measured preload, a CAD design correction or proven retention.','Sufficient force sum does not establish distributed torque feasibility, pressure persistence, motor holding, actuator motion or structural acceptance.'])
 if any(sha(R/p)!=h for p,h in pins.items()):raise ValueError('changed input')
 output.parent.mkdir(parents=True,exist_ok=True);tmp=output.with_suffix('.tmp');tmp.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');tmp.replace(output);return result
if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('--source',type=pathlib.Path,required=True);a.add_argument('--output',type=pathlib.Path,required=True);x=a.parse_args();print(run(x.source.resolve(),x.output)['bound'])

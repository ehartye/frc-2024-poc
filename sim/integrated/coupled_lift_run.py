"""Fresh10ms checkpoint continuation; failures are retained, not tuned away."""
import pathlib,sys,json,hashlib,copy,math
R=pathlib.Path(__file__).resolve().parents[2];sys.path.insert(0,str(R/'sim/integrated'))
from coupled_lift_phase import CoupledLift
import finite_retention_guide as guide
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
FILES=['sim/integrated/continuation_state.py','sim/integrated/coupled_lift_phase.py','sim/integrated/coupled_lift_run.py','sim/integrated/patch_registration.py','sim/integrated/guide_contact.py','cad/assembly/finite_retention_guide.py','sim/actuation/mechanics.py','sim/holding/finite_motor_patch.py','sim/drivetrain/plant.py','sim/integrated/shared_supply.py','sim/actuation/config.json','sim/holding/config.json']
def setup():
 sources=['sim/integrated/evidence/fast-regulated-index.json','sim/integrated/evidence/indexed-patch-registration.json','sim/actuation/evidence.json','sim/holding/evidence.json']
 docs=[];pins={}
 for name in sources:
  d=json.loads((R/name).read_text());docs.append(d)
  for f,h in d['inputs'].items():
   if sha(R/f)!=h:raise ValueError('stale receipt '+f)
   if f in pins and pins[f]!=h:raise ValueError('conflicting receipt lineage')
   pins[f]=h
  pins[name]=sha(R/name)
 source,patch,_,_=docs;gc=guide.load_contract(R);pins.update(gc['sourceInputs']);pins['cad/assembly/evidence/rounded-retention-guide.json']=gc['receiptSha256'];pins.update({f:sha(R/f) for f in FILES})
 saved=source['checkpoint'];f=source['final'];a=json.loads((R/'sim/actuation/config.json').read_text());h=json.loads((R/'sim/holding/config.json').read_text());m=saved['noteConfig']['pieceMassKg']
 c=dict(inheritedNormalSpringJ=f['energyJ']['spring'],driveDemands=f['controller']['driveDemands'],pinchDemandsRetention=[-x for x in f['controller']['pinchDemands']],notePitchInertiaKgM2=m*(.1524**2/2+5*.0254**2/8),floorPoints=16,floorStiffnessNpm=20000.,floorDampingNspm=20.,guideStiffnessNpm=20000.,guideDampingNspm=20.,guideMu=.2,nonplanarWrenchLimit=1e-8,nonplanarPowerLimitW=1e-10,nonplanarVelocityLimit=1e-8,planarAngleLimitRad=1e-8)
 return source,patch,a,h,gc,c,pins

def run(output):
 if output.exists():raise ValueError('fresh output required')
 source,patch,a,h,gc,c,pins=setup();cases=[]
 for name,dt in [('gravity-feedforward',.001),('half-step',.0005),('quarter-step',.00025),('actuators-coast',.001)]:
  p=CoupledLift(source['checkpoint'],patch,a,h,gc,c,source['inputs']);cmd=p.commands('actuators-coast' if name=='actuators-coast' else 'gravity-feedforward');initial=copy.deepcopy(p.s);initial_energy=copy.deepcopy(p.ledger);trace=[];status='HORIZON';failure=None
  for _ in range(round(.01/dt)):
   before=copy.deepcopy((p.s,p.ledger))
   try:trace.append(p.step(dt,cmd))
   except ValueError as e:
    status='UNKNOWN_DOMAIN';failure=str(e)
    if before!=(p.s,p.ledger):raise ValueError('failure mutated state')
    break
  expected=source['final']['energyJ']['kinetic']+source['final']['energyJ']['spring']
  if abs(p.initialMechanical-expected)>1e-12:raise ValueError('initial mass/energy discontinuity')
  cases.append(dict(name=name,stepS=dt,commands=cmd,initialState=initial,initialCumulativeEnergyJ=initial_energy,initialMechanicalJ=p.initialMechanical,sourceMechanicalJ=expected,initialEnergyResidualJ=p.sourceInitial+initial_energy['sourceWork']-sum(v for k,v in initial_energy.items() if k!='sourceWork')-p.initialMechanical,initialEnergyDifferenceJ=p.initialMechanical-expected,outcome=status,reason=failure,trace=trace,finalState=p.s,finalLedger=p.ledger,acceptance=False))
  print(name,status,failure,'steps',len(trace),'dz',p.s['zPivot']-.0254,'pitch',p.s['theta'],flush=True)
 if any(sha(R/f)!=h for f,h in pins.items()):raise ValueError('inputs changed')
 r=dict(schema='coupled-lift-short-continuation/1',inputs=pins,config=c,cases=cases,acceptance=False,limits=['Planar symmetric10ms phase;actual tiny Y/yaw states advance ballistically with retained energies, and nonplanar forces/torques are reported and bounded.','38kg base plus7kg mechanism replaces prior45kg total;no added initial kinetic energy.','Initialz/pitch velocities are inherited flat-floor constraints, not a new aligned pose.','Compliant16-point floor with20kN/m and20Ns/m compression-only damping replaces the old flat support constraint explicitly.','Finite side budgets remain1.755N/side;no ideal attachment or increased preload.','Inherited side-normal spring energy remains a constant reservoir only while lateral work is negligible and witnesses supported.','Guide remains unadopted/material UNKNOWN;20degree disagreement and unsupported geometry stop atomically.','No lift mission, clearance/retention/score acceptance or calibrated force/actuator claim.'])
 output.parent.mkdir(parents=True,exist_ok=True);temp=output.with_suffix('.tmp');temp.write_text(json.dumps(r,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');temp.replace(output)
if __name__=='__main__':run(pathlib.Path(sys.argv[1]))

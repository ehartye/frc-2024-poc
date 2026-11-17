"""Source-bound interface checks; no lift trajectory or accepted contact handoff."""
import pathlib,sys,json,hashlib,math
R=pathlib.Path(__file__).resolve().parents[2];sys.path[:0]=[str(R/'sim/holding'),str(R/'sim/integrated'),str(R/'sim/acquisition')]
from finite_motor_patch import FiniteMotorPatch
from coupled_acquisition import acquisition_rates
import free_geometry,obstacle_gate
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
FILES=['sim/holding/finite_motor_patch.py','sim/holding/config.json','sim/holding/run.py']
def run(output):
 if output.exists():raise ValueError('fresh output required')
 source=R/'sim/integrated/evidence/fast-regulated-index.json';r=json.loads(source.read_text());pins=dict(r['inputs']);pins[source.relative_to(R).as_posix()]=sha(source);pins.update({f:sha(R/f) for f in FILES})
 if any(sha(R/p)!=h for p,h in pins.items()):raise ValueError('stale checkpoint inputs')
 f=r['final'];a=json.loads((R/'sim/acquisition/free-planar-config.json').read_text())['model'];cad=json.loads((R/'cad/assembly/intake_components.json').read_text());obs=json.loads((R/'sim/acquisition/free-obstacles.json').read_text());c=json.loads((R/'sim/holding/config.json').read_text())
 geometry=free_geometry.make_geometry(cad,obstacle_gate.make_gate(obs,cad['note']['majorRadius'],cad['note']['tubeRadius'],cad['floorCenterZ']))
 rates=acquisition_rates(f['note'],f['chassis'],a,geometry,f['busV'],f['controller']['pinchDemands']);end=sorted(rates['contacts'],key=lambda x:x['motor'])
 if len(end)!=2 or any(abs(x['normalN']-n)>1e-10 for x,n in zip(end,c['normalSideN'])):raise ValueError('end normal/config mismatch')
 expected={'motorK':1/a['kvRadpsPerV'],'resistanceOhm':a['motorResistanceOhm'],'currentLimitA':a['currentLimitA'],'rotorInertiaKgM2':a['rotorInertiaKgM2'],'beltRadiusM':a['radiusM'],'slipRegularizationMps':a['slipRegularizationMps'],'mu':a['beltMu']}
 if any(c[k]!=v for k,v in expected.items()):raise ValueError('motor/friction source mismatch')
 mapped=dict(timeS=f['timeS'],frameAngleRad=0.,rotorOmegaRadps=[-w for w in f['note']['w']]);p=FiniteMotorPatch(c,mapped)
 before=.5*c['rotorInertiaKgM2']*sum(w*w for w in f['note']['w']);after=p.stored(p.state)
 if before!=after:raise ValueError('rotor mapping energy')
 cases=[]
 for law in ['memoryless-tanh','elastic-plastic']:
  for dt in [.001,.0005,.00025]:
   cfg=dict(c,law=law);initial=dict(timeS=0.,frameAngleRad=0.,rotorOmegaRadps=[0.,0.])
   if law=='elastic-plastic':initial['shearWorldM']=[[0.,0.] for _ in c['patchMotor']]
   plant=FiniteMotorPatch(cfg,initial);peak=0.
   for _ in range(round(.02/dt)):
    row=plant.step(dt,12.,(.02,.02),('drive','drive'),0.,[[.05,.02] for _ in c['patchMotor']],[[0.,0.] for _ in c['patchMotor']],[[0.,0.] for _ in c['patchMotor']],[True]*len(c['patchMotor']));peak=max(peak,abs(row['energyJ']['residual']))
   cases.append(dict(law=law,stepS=dt,initial=initial,final=row,peakResidualJ=peak,synthetic=True))
 report=dict(schema='finite-motor-holding-interface/1',inputs=pins,sourceTimeS=f['timeS'],sourceEndContact=end,archivedMidpointContact=f['contacts'],mappedRotorState=mapped,rotorEnergyBeforeJ=before,rotorEnergyAfterJ=after,staticBound=dict(capacityN=sum(c['normalSideN'])*c['mu'],weightN=a['pieceMassKg']*9.80665,necessaryConditionMet=sum(c['normalSideN'])*c['mu']>=a['pieceMassKg']*9.80665),finitePatchRegistered=False,elasticHandoffAvailable=False,cases=cases,acceptance=False,limits=['Mapped motor state is not an advanced lift simulation or a registered finite patch.','Synthetic interface cases prescribe body/contact velocities and declared initial state; no acquisition history is reset or relabeled.','The normal budget is the actual end-state value, not the old10N hypothesis.','Memoryless mode adds no elastic state; elastic mode requires separately supplied history and accounts for numerical return-map loss.','Two-dimensional friction budget permits creep/slip; actual pitch/lift actuation and contact persistence remain separate gates.'])
 if any(sha(R/p)!=h for p,h in pins.items()):raise ValueError('inputs changed')
 output.parent.mkdir(parents=True,exist_ok=True);tmp=output.with_suffix('.tmp');tmp.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');tmp.replace(output);print(report['staticBound']);print([(x['law'],x['stepS'],x['final']['nextState']['rotorOmegaRadps'],x['peakResidualJ']) for x in cases])
if __name__=='__main__':run(pathlib.Path(sys.argv[1]))

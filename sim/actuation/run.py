"""Source-bound standalone actuator fixtures; base fixed, not achieved lift motion."""
import pathlib,json,hashlib,sys
from mechanics import ActuatorMechanics
R=pathlib.Path(__file__).resolve().parents[2]
FILES=['sim/actuation/mechanics.py','sim/actuation/config.json','sim/actuation/run.py','sim/actuation/reference-pinch-model.json','cad/assembly/actuator-geometry-contract.json']
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def run(output):
 if output.exists():raise ValueError('fresh output required')
 c=json.loads((R/'sim/actuation/config.json').read_text());g=json.loads((R/'cad/assembly/actuator-geometry-contract.json').read_text());pins=dict(g['inputs']);pins.update({f:sha(R/f) for f in FILES})
 if any(sha(R/f)!=v for f,v in pins.items()):raise ValueError('stale source')
 if c['geometryContractSha256']!=pins['cad/assembly/actuator-geometry-contract.json']:raise ValueError('geometry contract')
 source=json.loads((R/c['motorLawProvenance']['snapshot']).read_text())['model']
 if c['motorK']!=1/source['kvRadpsPerV'] or c['resistanceOhm']!=source['motorResistanceOhm'] or c['currentLimitA']!=source['currentLimitA']:raise ValueError('motor law snapshot mismatch')
 cases=[]
 for name in ['powered','coast','brake','loaded','half-step','quarter-step']:
  dt=.00025 if name=='quarter-step' else .0005 if name=='half-step' else .001
  state=dict(timeS=0.,q=[0.,.3,.2],qd=[0.,.05,.2] if name in ['coast','brake'] else [0.,0.,0.]);p=ActuatorMechanics(c,state);peak=0.;trace=[]
  modes=(name,name) if name in ['coast','brake'] else ('drive','drive');external=(3.,-10.,.3) if name=='loaded' else (0.,-1.,.1)
  for i in range(round(.05/dt)):
   row=p.step_fixed(dt,12.,(.04,.01),modes,external);peak=max(peak,abs(row['energyJ']['residual']))
   if i%10==0:trace.append(row)
  cases.append(dict(name=name,stepS=dt,initial=state,externalGeneralizedForce=external,trace=trace,final=row,peakResidualJ=peak))
 if any(sha(R/f)!=v for f,v in pins.items()):raise ValueError('inputs changed')
 result=dict(schema='finite-actuator-interface/1',inputs=pins,config=c,cases=cases,acceptance=False,limits=c['assumptions']+['Fixed-base fixtures explicitly impose zero base velocity; they are not continuation of the indexed acquisition state.','Full baseX/z/theta mass and force terms are exposed for caller coupling; no chassis mass is hidden here.','12V fixture is supplied externally; actual multi-mechanism shared-bus integration remains with the coordinator.'])
 output.parent.mkdir(parents=True,exist_ok=True);tmp=output.with_suffix('.tmp');tmp.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');tmp.replace(output)
 print([(x['name'],x['final']['state'],x['peakResidualJ']) for x in cases])
if __name__=='__main__':run(pathlib.Path(sys.argv[1]))

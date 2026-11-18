"""Pure exact-endpoint pressure reconstruction; no lift/contact calibration."""
import pathlib,json,hashlib
import run_inward_extended as model
R=model.R
FILES=['sim/pressure/inward-extended-100ms-evidence.json','sim/pressure/inward-rearwrap-200ms-evidence.json','sim/pressure/inward-tail-200ms-evidence.json']
def summarize():
 pins={f:model.c.factory.r.sha(R/f) for f in FILES+['sim/pressure/summarize_inward_tail.py']};rows=[]
 for f in FILES:
  r=json.loads((R/f).read_text());pins.update(r['inputs']);p=model.restore_domain(r['endCheckpoint']);before=model.export_domain(p)
  prepared=p.prepared(p.drive.state,p.note,{})
  normal=[sum(c['normalN'] for c in prepared['contacts'] if c['motor']==i) for i in range(2)]
  final=r['trace'][-1]
  rows.append(dict(source=f,timeS=p.t,relativeXM=final['relativeRobotXY'][0],chassisVxMps=p.drive.state['vx'],noteVxMps=p.note['v'][0],endNormalN=normal,endFeatures=prepared['features'],endPotentialJ=prepared['potentialJ'],lastMidpointTimeS=final['contactSampleTimeS'],lastMidpointNormalN=[c['normalMagnitudeN'] for c in final['contacts']],conditionalFrictionUpperBudgetN=p.c['beltMu']*sum(normal),noteWeightN=p.c['pieceMassKg']*9.80665,mu=p.c['beltMu'],phaseResidualJ=final['newPhaseResidualJ'],runtimeSeconds=r['runtimeSeconds']))
  if model.export_domain(p)!=before:raise ValueError('query mutated state')
 if any(model.c.factory.r.sha(R/f)!=h for f,h in pins.items()):raise ValueError('source drift')
 return dict(schema='inward-tail-endpoint-reconstruction/1',inputs=pins,rows=rows,acceptance=False,limits=['Exact accepted-end pressure query, not archived midpoint force treated as end state.','Conditional generous Coulomb budget assumes these flat-floor normals and coefficient; no lift trajectory or calibrated preload.','No stopping/capture/retention claim; no200ms tail half-step yet.','Constant ground constraint, fixed u and material hypotheses remain unchanged.'])
if __name__=='__main__':
 import sys
 pathlib.Path(sys.argv[1]).write_text(json.dumps(summarize(),indent=2)+'\n',newline='\n')

"""Same-endpoint policy refinement and explicit physically identical controls."""
import json,pathlib,sys,copy
import run_early_policy as r
R=r.R
NAMES=['nominal-1ms','nominal-half','stale','no-pinch']
FILES=['sim/pressure/summarize_early_policy.py','tests/test_pressure_early_receipts.py']
def summarize():
 p,pins,_,_=r.load();start=r.c.export(p);reports=[];rows=[]
 for name in NAMES:
  path='sim/pressure/early-policy-evidence/'+name+'.json';e=json.loads((R/path).read_text())
  if e['schema']!='pressure-early-policy/1' or e['inputs']!=pins or e['startCheckpoint']!=start or e['outcome']!='HORIZON':raise ValueError('case lineage/scope mismatch')
  q=r.c.restore(e['endCheckpoint']);last=e['trace'][-1]
  if abs(q.t-.662)>1e-12 or len(e['trace'])!=round(.1/e['stepS']):raise ValueError('different endpoint')
  rates=p.acquisition_rates(q.note,q.drive.state,last['busV'],last['command']['pinchDemands'],{})
  rows.append(dict(name=name,stepS=e['stepS'],timeS=q.t,relativeX=q.note['p'][0]-q.drive.state['x'],noteX=q.note['p'][0],noteVx=q.note['v'][0],chassisX=q.drive.state['x'],chassisVx=q.drive.state['vx'],rotorOmega=q.note['w'],newPhaseResidualJ=e['trace'][-1]['newPhaseResidualJ'],endContacts=rates['contacts'],pressurePotentialJ=rates['springJ'],runtimeSeconds=e['runtimeSeconds']))
  reports.append(e)
 physical=lambda e:[{k:v for k,v in row.items() if k!='command'} for row in e['trace']]
 if not physical(reports[0])==physical(reports[2])==physical(reports[3]):raise ValueError('controls differ physically')
 if r.c.export(p)!=start:raise ValueError('endpoint reconstruction mutated state')
 inventory=dict(pins);inventory.update({f:r.c.factory.r.sha(R/f) for f in FILES+['sim/pressure/early-policy-evidence/'+name+'.json' for name in NAMES]})
 keys=['noteX','noteVx','chassisX','chassisVx','relativeX','pressurePotentialJ','newPhaseResidualJ']
 return dict(schema='pressure-early-policy-comparison/1',inputs=inventory,rows=rows,halfMinusCoarse={k:rows[1][k]-rows[0][k] for k in keys},controlsPhysicalTraceEquality=True,acceptance=False,limits=['Phase-local timestep comparison from identical corrected-source checkpoint, not full-run or spatial convergence.','End mechanical contact rates reconstructed at same time using last recorded bus; no new electrical equilibrium claim.','Fault/command differences do not imply immediate momentum loss; three1ms physical traces are identical.','Still moving, not indexed, captured, retained or lifted.'])
if __name__=='__main__':
 out=pathlib.Path(sys.argv[1])
 if out.exists():raise ValueError('fresh output required')
 result=summarize()
 if any(r.c.factory.r.sha(R/f)!=h for f,h in result['inputs'].items()):raise ValueError('source changed')
 out.parent.mkdir(parents=True,exist_ok=True);tmp=out.with_suffix('.tmp');tmp.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');tmp.replace(out)

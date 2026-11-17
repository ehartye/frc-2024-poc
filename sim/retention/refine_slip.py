"""Fresh finer-step comparison; energy refinement alone is not trajectory convergence."""
import pathlib,json,hashlib,sys
import numpy as np
import finite_slip as model
R=model.R
sha=model.sha

def compare(a,b):
 ta=np.array([r['timeS'] for r in a['trace']]);tb=np.array([r['timeS'] for r in b['trace']]);t=ta[(ta>=tb[0])&(ta<=tb[-1])]
 out={}
 for key in ['notePitchRad','noteOmegaRadps']:
  va=np.interp(t,ta,[r[key] for r in a['trace']]);vb=np.interp(t,tb,[r[key] for r in b['trace']]);out[key+'MaxAbsDelta']=float(np.max(np.abs(va-vb)))
 for key in ['noteCenterXZ','noteVelocityXZ']:
  delta=np.column_stack([np.interp(t,ta,[r[key][i] for r in a['trace']])-np.interp(t,tb,[r[key][i] for r in b['trace']]) for i in range(2)])
  out[key+'MaxNormDelta']=float(np.max(np.linalg.norm(delta,axis=1)))
 return out

def run(output):
 if output.exists():raise ValueError('Fresh output required')
 basepath=R/'sim/retention/finite-slip-evidence.json';base=json.loads(basepath.read_text())
 if any(sha(R/p)!=h for p,h in base['inputs'].items()):raise ValueError('Stale baseline')
 files=[pathlib.Path(__file__),basepath];pins=dict(base['inputs'],**{p.relative_to(R).as_posix():sha(p) for p in files})
 c=json.loads((R/'sim/retention/finite-slip-config.json').read_text());g=json.loads((R/'cad/assembly/evidence/retention-patch.json').read_text())
 geo=next(x for x in g['cases'] if x['state']=='floor' and x['noteLongitudinalOffsetMm']==9.88835038313412)
 ground=json.loads((R/'sim/acquisition/evidence.json').read_text());c['initialWorldVx']=next(x for x in ground['cases'] if x['case']=='braked-hypothesis')['final']['pieceWorldSpeedMps']
 cases=[dict(stepS=.000025,result=next(x for x in base['cases'] if x['scenario']=='quarter-step'))]
 for dt in [.0000125,.00000625]:
  print('Running',dt,flush=True);cfg=dict(c,stepS=dt);result=model.simulate(cfg,geo,'locked-hypothesis');cases.append(dict(stepS=dt,result=result));print(result['outcome'],result['final']['notePitchRad'],flush=True)
 comparisons=[dict(coarseStepS=a['stepS'],fineStepS=b['stepS'],**compare(a['result'],b['result'])) for a,b in zip(cases,cases[1:])]
 if any(sha(R/p)!=h for p,h in pins.items()):raise ValueError('Changed inputs')
 report=dict(schema='frc-retention-refinement/1',inputs=pins,cases=cases,comparisons=comparisons,acceptance=False,limits=['Same uncalibrated fixed-footprint hypothesis; finite trajectory differences are diagnostics, not proof of continuum convergence.','Interpolated saved traces omit between-sample extrema.','No automatic downstream state reset or transfer acceptance.'])
 output.mkdir(parents=True);(output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');return report
if __name__=='__main__':
 import argparse;p=argparse.ArgumentParser();p.add_argument('--output',type=pathlib.Path,required=True);a=p.parse_args();print(run(a.output)['comparisons'])

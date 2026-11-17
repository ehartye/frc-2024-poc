"""Derive compact, input-bound diagnostics from a complete refinement run."""
import json,pathlib,argparse
from refine_slip import compare,sha,R

def summarize(path):
 r=json.loads(path.read_text())
 if any(sha(R/p)!=h for p,h in r['inputs'].items()):raise ValueError('Stale refinement inputs')
 if [x['stepS'] for x in r['cases']]!=[.000025,.0000125,.00000625]:raise ValueError('Expected three refinements')
 expected=[dict(coarseStepS=a['stepS'],fineStepS=b['stepS'],**compare(a['result'],b['result'])) for a,b in zip(r['cases'],r['cases'][1:])]
 if r['comparisons']!=expected:raise ValueError('Comparison mismatch')
 return dict(schema='frc-retention-refinement-summary/1',inputs=dict(r['inputs'],**{'sim/retention/summarize_refinement.py':sha(pathlib.Path(__file__))}),fullReportSha256=sha(path),comparisons=expected,cases=[dict(stepS=x['stepS'],**{k:v for k,v in x['result'].items() if k!='trace'}) for x in r['cases']],acceptance=False,limits=r['limits'])
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('report',type=pathlib.Path);p.add_argument('output',type=pathlib.Path);a=p.parse_args()
 if a.output.exists():raise ValueError('Fresh summary output required')
 a.output.write_text(json.dumps(summarize(a.report),indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n')

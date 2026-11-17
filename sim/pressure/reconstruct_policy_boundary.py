"""Read-only repeat of rejected next step; saves query before atomicity diagnostics."""
import copy,json,pathlib,sys
import run_policy_transition as r
R=r.R
FILES=['sim/pressure/reconstruct_policy_boundary.py','tests/test_pressure_policy_boundary.py']
def reconstruct(path):
 raw=path.read_bytes();report=json.loads(raw);base,pins=r.load()
 if report['schema']!='pressure-policy-transition/1' or report['inputs']!=pins or report['startCheckpoint']!=r.c.export(base):raise ValueError('policy report lineage')
 if report['outcome']!='UNKNOWN_DOMAIN' or not report['events']:raise ValueError('recorded rejection required')
 p=r.c.restore(report['endCheckpoint']);before=r.c.export(p);last=None;original=p.geometry
 def query(q):
  nonlocal last
  result=original(q);last=dict(relativeRobotXY=list(q),status=result['status']);return result
 p.geometry=query;cmd=report['events'][-1]['command']
 try:p.step(report['stepS'],cmd['driveDemands'],cmd['pinchDemands'],[p.drive.c['massKg']*9.80665/2]*2,modes=cmd['modes'])
 except (ValueError,r.DomainExit) as error:reason=str(error);rejected=copy.deepcopy(last)
 else:raise ValueError('recorded rejection did not recur')
 if reason!=report['reason'] or r.c.export(p)!=before:raise ValueError('rejection/atomicity mismatch')
 if raw!=path.read_bytes() or any(r.c.factory.r.sha(R/f)!=h for f,h in pins.items()):raise ValueError('source changed')
 return dict(scenario=report['scenario'],policyReportSha256=r.c.factory.r.sha(path),acceptedTimeS=p.t,reason=reason,rejectedGeometryQuery=rejected,atomicStateUnchanged=True,command=cmd)
def run(output,paths):
 if output.exists():raise ValueError('fresh output')
 _,pins=r.load();pins.update({f:r.c.factory.r.sha(R/f) for f in FILES});rows=[reconstruct(path) for path in paths]
 if any(r.c.factory.r.sha(R/f)!=h for f,h in pins.items()):raise ValueError('source changed')
 result=dict(schema='pressure-policy-boundary-reconstruction/1',inputs=pins,rows=rows,acceptance=False,limits=['Re-executes only rejected next step from exact archived accepted checkpoint.','UNKNOWN is inherited experiment domain, not physical collision or full release.'])
 output.parent.mkdir(parents=True,exist_ok=True);tmp=output.with_suffix('.tmp');tmp.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');tmp.replace(output)
if __name__=='__main__':run(pathlib.Path(sys.argv[1]),[pathlib.Path(p) for p in sys.argv[2:]])

"""Bounded same-law continuation from an exact-source archived pressure endpoint."""
import pathlib,json,sys,copy,time
import checkpoint as c
R=c.R
FILES=['sim/pressure/checkpoint.py','sim/pressure/compare_entry.py','sim/pressure/run_continuation.py','tests/test_pressure_checkpoint.py']
def run(output,duration=.02):
 if output.exists() or duration not in [.02,.05,.1]:raise ValueError('fresh bounded continuation required')
 source=R/'sim/pressure/acquisition-entry-evidence.json';p,ancestry=c.from_pressure_report(source);start=c.export(p);pins=dict(start['identity']['inputs']);pins.update({f:c.factory.r.sha(R/f) for f in FILES});pins['sim/pressure/acquisition-entry-evidence.json']=ancestry
 rows=[];status='HORIZON';reason=None;t0=time.perf_counter()
 for _ in range(round(duration/.001)):
  before=c.export(p)
  try:row=p.step(.001,[.2,.2],[.12,.12],[p.drive.c['massKg']*9.80665/2]*2)
  except ValueError as error:
   if c.export(p)!=before:raise AssertionError('rejection mutated state')
   status='UNKNOWN_DOMAIN';reason=str(error);break
  rows.append(row)
 if any(c.factory.r.sha(R/f)!=h for f,h in pins.items()):raise ValueError('source changed')
 result=dict(schema='pressure-continuation/1',inputs=pins,ancestrySha256=ancestry,startCheckpoint=start,endCheckpoint=c.export(p),stepS=.001,requestedDurationS=duration,trace=rows,outcome=status,reason=reason,runtimeSeconds=time.perf_counter()-t0,acceptance=False,limits=['Exact same-pressure-law state and cumulative ledger; no old checkpoint swap.','Fixed drive.2 and pinch.12 commands; no capture controller or lift.','Contacts are midpoint samples; states are endpoints.','Known geometry domain gates remain; no physical calibration or capture claim.'])
 output.parent.mkdir(parents=True,exist_ok=True);tmp=output.with_suffix('.tmp');tmp.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');tmp.replace(output)
 print(json.dumps(dict(outcome=status,reason=reason,timeS=p.t,note=p.note,chassisX=p.drive.state['x'],energy=c.diagnostics(p),runtime=result['runtimeSeconds'])))
if __name__=='__main__':run(pathlib.Path(sys.argv[1]),float(sys.argv[2]) if len(sys.argv)>2 else .02)

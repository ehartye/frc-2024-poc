"""Keep failed implementation and controller-clock runs explicitly diagnostic."""
import argparse,hashlib,json,pathlib
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
p=argparse.ArgumentParser();p.add_argument('--packet',type=pathlib.Path,required=True);p.add_argument('--output',type=pathlib.Path,required=True);a=p.parse_args();assert not a.output.exists();rows=[]
names=['native-pressure-lift-2ms','native-pressure-lift-2ms-typed']+['native-pressure-lift-100ms-'+x for x in ['half','quarter','eighth']]+['native-pressure-lift-800ms-'+x for x in ['half','quarter']]
for name in names:
 path=a.packet/name/'receipt.json';r=json.loads(path.read_bytes());producer=a.packet/('lift-initial-type-rejection-source' if name=='native-pressure-lift-2ms' else '')
 for f in ['native_pressure_lift.py','run_native_pressure_lift.py']:assert sha(producer/f)==r['inputs'][f]
 rows.append(dict(name=name,inputs=r['inputs'],outcome=r['outcome'],reason=r['reason'],acceptedSteps=len(r['trace']),stepS=r['stepS'],durationS=r['final']['timeS']-r['initial']['timeS'],receiptSha256=sha(path),authority='Implementation rejection only' if name.endswith('-2ms') else 'Original floating-branch controller; superseded by integer-tick three-level runs for temporal comparison',requestedHoldTransitionElapsedS=next((x['timeS']-r['initial']['timeS'] for x in r['events'] if x['command']['requestedPivotVelocityMps']==0 and x['timeS']>r['initial']['timeS']+.4),None)))
out=dict(schema='native-lift-preserved-diagnostics/1',runs=rows,acceptance=False,limits=['Initial NumPy scalar rejection accepted zero steps; casting scalar rate outputs resolved the interface error','Floating comparisons at 0.2s/0.5s could delay a command phase by one 100Hz tick across timesteps; these are not matched-input refinement','Executed sources, failed runs and original contracts remain unchanged'])
a.output.write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps(out,indent=2))

"""One-command A2 mission, actual-exit screen and portable replay. Fresh atomic output only."""
import argparse,datetime,hashlib,importlib.util,json,pathlib,tempfile
ROOT=pathlib.Path(__file__).resolve().parents[1]
def load(name,file):
 s=importlib.util.spec_from_file_location(name,ROOT/'scripts'/file);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def run(out,java,gradle):
 out=pathlib.Path(out).resolve()
 if out.exists():raise ValueError('Fresh output required')
 source_files=['scripts/run-a2.py','scripts/build-a2-replay.py','scripts/a2-replay-template.html','scripts/a2-exit-screen.py','cad/cycle-contract-A2.json']
 before={p:sha(ROOT/p) for p in source_files};out.parent.mkdir(parents=True,exist_ok=True);pending=pathlib.Path(tempfile.mkdtemp(prefix=out.name+'-pending-',dir=out.parent))
 native=load('mission_runner','run-a2-mission.py');receipt=native.run(pending/'native',java,gradle);replay=load('replay_renderer','build-a2-replay.py');targets=load('exit_screen','a2-exit-screen.py');summaries=[]
 for scenario in native.SCENARIOS:
  csv=pending/'native'/(scenario+'.csv');metadata=pending/'native'/(scenario+'.json')
  for path in [csv,metadata]:
   if sha(path)!=receipt['artifactsSha256'][path.name]:raise ValueError('Native artifact changed')
  replay.render(csv,pending/(scenario+'.html'));screen=targets.report(csv);(pending/(scenario+'-targets.json')).write_text(json.dumps(screen,indent=2,allow_nan=False)+'\n',encoding='utf-8');summaries.append(dict(json.loads(metadata.read_text()),targetScreen=screen))
 if before!={p:sha(ROOT/p) for p in source_files}:raise ValueError('Presentation inputs changed during run')
 (pending/'results.json').write_text(json.dumps({'season':2024,'scenarios':summaries,'officialScores':None},indent=2,allow_nan=False)+'\n',encoding='utf-8')
 lines=['# 2024 A2 run','', 'Open NORMAL.html for the recorded cycle; NORMAL_9V.html and the fault replays retain comparisons.','', '| Scenario | State | Releases | Seconds |','|---|---|---:|---:|']
 for s in summaries:lines.append(f"| {s['scenario']} | {s['state']} | {s['releases']} | {s['timeS']:.3f} |")
 lines+=['','Pickup, telescope, retention and transfer are prescribed hypotheses; shooter/guide energy is modeled. No official score or physical acceptance. A roughly 27-second two-shot cycle is not 15-second autonomous.','', 'Native source/runtime binding: native/receipt.json. Presentation binding: presentation-receipt.json.']
 (pending/'SUMMARY.md').write_text('\n'.join(lines)+'\n',encoding='utf-8');files=[p for p in pending.iterdir() if p.is_file()]
 report={'schema':'frc-a2-presentation-receipt/1','nativeReceiptSha256':sha(pending/'native/receipt.json'),'sourceSha256':before,'artifactsSha256':{p.name:sha(p) for p in files}}
 (pending/'presentation-receipt.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');pending.rename(out);return out
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--java-home',required=True);p.add_argument('--gradle-home',required=True);p.add_argument('--output');a=p.parse_args();out=a.output or ROOT/'artifacts'/('a2-'+datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'));print(run(out,a.java_home,a.gradle_home))

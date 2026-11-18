"""Fresh bounded acquisition -> exact registered checkpoint -> fresh lift pilot.
Not an arbitrary-state adapter or complete mission; refuses mismatched registration.
"""
import hashlib,json,pathlib,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parents[2]
INDEX='sim/integrated/evidence/fast-regulated-index.json'
PILOT='sim/integrated/coupled_lift_evidence.json'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def check_index(fresh,registered):
 if fresh.read_bytes()!=registered.read_bytes():raise ValueError('fresh index differs from registered checkpoint; new registration required')
 return json.loads(fresh.read_text())
def check_handoff(index,pilot):
 cp=index['checkpoint']
 if len(pilot['cases'])!=4:raise ValueError('expected four bounded pilot cases')
 for case in pilot['cases']:
  state=case['initialState']
  if state['drive']!=cp['drive'] or state['note']!=cp['note'] or state['timeS']!=cp['timeS'] or state['pinchW']!=[-w for w in cp['note']['w']]:raise ValueError('handoff state mismatch')
  if any(case['initialCumulativeEnergyJ'].get(k)!=v for k,v in cp['energy'].items()):raise ValueError('handoff cumulative ledger mismatch')
  if abs(case['initialEnergyDifferenceJ'])>1e-12:raise ValueError('handoff mechanical energy mismatch')
def source_pins():
 pins={}
 for name in [INDEX,PILOT]:
  for rel,h in json.loads((ROOT/name).read_text())['inputs'].items():
   p=(ROOT/rel).resolve()
   if not p.is_relative_to(ROOT.resolve()) or sha(p)!=h:raise ValueError('invalid/stale source '+rel)
   if rel in pins and pins[rel]!=h:raise ValueError('conflicting lineage')
   pins[rel]=h
  pins[name]=sha(ROOT/name)
 pins[pathlib.Path(__file__).relative_to(ROOT).as_posix()]=sha(pathlib.Path(__file__))
 return pins
def run(out):
 pins=source_pins();out=out.resolve();out.mkdir(parents=True,exist_ok=False)
 fresh=out/'index.json';phase=out/'pilot.json'
 subprocess.run([sys.executable,str(ROOT/'sim/integrated/run_regulated_acquisition.py'),'--index','--output',str(fresh)],cwd=ROOT,check=True)
 index=check_index(fresh,ROOT/INDEX)
 subprocess.run([sys.executable,str(ROOT/'sim/integrated/coupled_lift_run.py'),str(phase)],cwd=ROOT,check=True)
 pilot=json.loads(phase.read_text());check_handoff(index,pilot)
 if source_pins()!=pins:raise ValueError('sources changed during execution')
 report=dict(schema='fresh-current-checkpoint-chain/1',inputs=pins,outputs={p.name:sha(p) for p in [fresh,phase]},freshIndexEqualsRegisteredBytes=True,checkpointTimeS=index['checkpoint']['timeS'],pilotOutcomes=[dict(name=c['name'],outcome=c['outcome'],reason=c['reason'],elapsedS=c['finalState']['timeS']-c['initialState']['timeS']) for c in pilot['cases']],acceptance=False,limits=['Fresh indexing executes first; continuation is allowed only when its entire receipt equals the registered source bytes.','Pilot consumes the identical registered checkpoint; this is not an arbitrary-state registration adapter.','10ms symmetric-XZ phase, finite hypotheses and retained unsupported-domain failures; not achieved lift, mission, score or hardware validation.'])
 tmp=out/'chain.tmp';tmp.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');tmp.replace(out/'chain.json');return report
if __name__=='__main__':run(pathlib.Path(sys.argv[1]))

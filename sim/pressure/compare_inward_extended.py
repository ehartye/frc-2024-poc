"""Same-state timestep comparison of bounded explicit-domain experiment."""
import json,pathlib,hashlib
R=pathlib.Path(__file__).resolve().parents[2]
FILES=['sim/pressure/inward-extended-100ms-evidence.json','sim/pressure/inward-extended-half-evidence.json']
def compare():
 reports=[json.loads((R/f).read_text()) for f in FILES]
 a,b=reports
 if a['inputs']!=b['inputs'] or a['startCheckpoint']!=b['startCheckpoint'] or a['outcome']!='HORIZON' or b['outcome']!='HORIZON':raise ValueError('same-state bounded comparison')
 for r in reports:
  if any(hashlib.sha256((R/f).read_bytes()).hexdigest()!=h for f,h in r['inputs'].items()):raise ValueError('input bytes changed')
  if len(r['events'])!=10 or any(abs(e['timeS']-(.962+.01*i))>1e-12 for i,e in enumerate(r['events'])):raise ValueError('controller phase')
 rows=[]
 for f,r in zip(FILES,reports):
  end=r['trace'][-1]
  rows.append(dict(file=f,sha256=hashlib.sha256((R/f).read_bytes()).hexdigest(),stepS=r['stepS'],acceptedSteps=len(r['trace']),timeS=end['timeS'],relativeX=end['relativeRobotXY'][0],chassisVx=end['chassis']['vx'],noteVx=end['note']['v'][0],potentialJ=end['energyJ']['spring'],phaseResidualJ=end['newPhaseResidualJ'],runtimeSeconds=r['runtimeSeconds']))
 return dict(schema='pressure-inward-extended-comparison/1',inputs={f:hashlib.sha256((R/f).read_bytes()).hexdigest() for f in FILES+['sim/pressure/compare_inward_extended.py']},rows=rows,absoluteEndpointDeltas={k:abs(rows[0][k]-rows[1][k]) for k in ['relativeX','chassisVx','noteVx','potentialJ']},acceptance=False,limits=['Same .960 start; this refines only next100ms, not full previous trajectory.','No spatial convergence, physical calibration, capture, stopping or lift acceptance.','Domain extension retains full finite pressure law and obstacle rejection.'])
if __name__=='__main__':
 import sys
 Path=pathlib.Path;Path(sys.argv[1]).write_text(json.dumps(compare(),indent=2)+'\n',newline='\n')

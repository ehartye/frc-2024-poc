"""Separate drive-only brake projection, not a bound on coupled NOTE motion."""
import copy,json,pathlib
import run_fresh_rest_controller as fresh
R=fresh.R
THRESHOLD_MPS=.005
FILES=['sim/pressure/braking_headroom.py','tests/test_pressure_braking_headroom.py','sim/drivetrain/plant.py','sim/drivetrain/config.json']
def project(config,state,dt=.0005):
 if dt not in [.001,.0005]:raise ValueError('declared step')
 original=copy.deepcopy(state);p=fresh.c.factory.r.drive_source.DrivePlant(config);p.state=copy.deepcopy(state);p.initial_kinetic=p.kinetic();start=copy.deepcopy(p.state);last=None;outcome='HORIZON'
 for i in range(round(3/dt)):
  last=p.step(dt,[0.,0.],12.,[config['massKg']*9.80665/2]*2,modes=['brake','brake'])
  speed=max(abs(p.state['vx']),abs(p.state['vy']),*(abs(w*config['wheelRadiusM']) for w in p.state['wheel']))
  if speed<THRESHOLD_MPS:outcome='LOW_SPEED_OBSERVATION';break
 if state!=original:raise ValueError('source state mutated')
 return dict(outcome=outcome,thresholdMps=THRESHOLD_MPS,stepS=dt,durationS=p.state['t']-start['t'],travelXM=p.state['x']-start['x'],finalState=p.state,energyJ=last['energyJ'],initialKineticJ=p.initial_kinetic,acceptance=False,limits=['Separate counterfactual starts with actual drive state and local energy reference; not injected into acquisition.','No NOTE force/reaction, no controller latency or uncertain material/load allowance; NOT conservative fullrobot stopping bound.','Same declared drivetrain brake/current/tire/drag/rotor equations; no fit to coast endpoint and no controller retuning.','Threshold is a kinematic observation, not stopped/captured/lift-ready acceptance.'])
def compare(candidate_path):
 config=json.loads((R/'sim/drivetrain/config.json').read_text());files=[fresh.BASELINE,str(candidate_path.relative_to(R))];rows=[];pins={f:fresh.c.factory.r.sha(R/f) for f in FILES+files}
 for f in files:
  report=json.loads((R/f).read_text())
  if any(fresh.c.factory.r.sha(R/name)!=h for name,h in report['inputs'].items()):raise ValueError('trajectory source mismatch')
  pins.update(report['inputs']);first=next(row for row in report['trace'] if row['contacts']);r=project(config,first['chassis'])
  rows.append(dict(source=f,firstAcceptedTimeS=first['timeS'],firstMidpointTimeS=first['contactSampleTimeS'],relativeXM=first['relativeRobotXY'][0],chassisVx=first['chassis']['vx'],noteVx=first['note']['v'][0],stationaryNoteHeadroomToGoalM=first['relativeRobotXY'][0]-.185,driveOnlyProjection=r))
 if any(fresh.c.factory.r.sha(R/f)!=h for f,h in pins.items()):raise ValueError('source drift')
 return dict(schema='drive-only-headroom-comparison/1',inputs=pins,rows=rows,acceptance=False)
if __name__=='__main__':
 import sys
 pathlib.Path(sys.argv[2]).write_text(json.dumps(compare(pathlib.Path(sys.argv[1]).resolve()),indent=2)+'\n',newline='\n')

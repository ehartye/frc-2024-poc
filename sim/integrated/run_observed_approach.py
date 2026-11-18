"""Closed-loop virtual observation plus persistent coupled current-CAD plant."""
import pathlib,sys,json,hashlib,argparse,math
R=pathlib.Path(__file__).resolve().parents[2];sys.path[:0]=[str(R/'sim/integrated'),str(R/'sim/acquisition')]
from coupled_acquisition import CoupledAcquisition,rotate,DomainExit
from approach_supervisor import ApproachSupervisor
import free_geometry,obstacle_gate
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def run(output,trigger=.25,scenario='nominal'):
 if output.exists():raise ValueError('fresh output')
 if type(trigger) not in (int,float) or not math.isfinite(trigger) or not .2<=trigger<=.4 or scenario not in ['nominal','stale']:raise ValueError('experiment domain')
 baseline_path=R/'sim/integrated/evidence/acquisition-nominal.json';base=json.loads(baseline_path.read_text());pins=dict(base['inputs'],**{p.relative_to(R).as_posix():sha(p) for p in [baseline_path,pathlib.Path(__file__),R/'sim/integrated/approach_supervisor.py']})
 if any(sha(R/p)!=h for p,h in pins.items()):raise ValueError('stale input')
 drive=json.loads((R/'sim/drivetrain/config.json').read_text());note=json.loads((R/'sim/acquisition/free-planar-config.json').read_text())['model'];cad=json.loads((R/'cad/assembly/intake_components.json').read_text());obstacles=json.loads((R/'sim/acquisition/free-obstacles.json').read_text())
 geometry=free_geometry.make_geometry(cad,obstacle_gate.make_gate(obstacles,cad['note']['majorRadius'],cad['note']['tubeRadius'],cad['floorCenterZ']))
 p=CoupledAcquisition(drive,note,geometry,dict(p=[.658,0.],v=[0.,0.],yaw=0.,spin=0.,w=[0.,0.]));controller=ApproachSupervisor(trigger);loads=[drive['massKg']*9.80665/2]*2;dt=.001;trace=[];events=[];outcome='HORIZON';command=None;row=None;maxres=0.;dwell=0.
 for i in range(1600):
  if i%10==0:
   d=p.drive.state;rel=rotate([p.note['p'][0]-d['x'],p.note['p'][1]-d['y']],-d['heading']);rv=rotate([p.note['v'][0]-d['vx'],p.note['v'][1]-d['vy']],-d['heading'])
   observed=dict(timestamp=min(p.t,.5) if scenario=='stale' else p.t,relativeX=rel[0],relativeVx=rv[0]);old=controller.state;command=controller.update(p.t,observed)
   if controller.state!=old:events.append(dict(timeS=p.t,fromState=old,toState=controller.state,observation=observed))
  try:row=p.step(dt,command['driveDemands'],command['pinchDemands'],loads,modes=command['modes'])
  except (ValueError,DomainExit) as exc:outcome='UNKNOWN_DOMAIN: '+str(exc);break
  row['controller']=dict(command);maxres=max(maxres,abs(row['energyJ']['residual']))
  speed=math.hypot(row['chassis']['vx'],row['chassis']['vy']);relative_speed=math.hypot(row['note']['v'][0]-row['chassis']['vx'],row['note']['v'][1]-row['chassis']['vy'])
  if controller.state=='BRAKE' and .178<=row['relativeRobotXY'][0]<=.22 and speed<.02 and relative_speed<.02:dwell+=dt
  else:dwell=0.
  if i%5==0:trace.append(row)
  if i%200==0:print(i,row['relativeRobotXY'],controller.state,flush=True)
  if dwell>=.1:outcome='GROUND_DWELL_DIAGNOSTIC_ONLY';break
 if row is not None and (not trace or trace[-1]['timeS']!=row['timeS']):trace.append(row)
 if any(sha(R/f)!=h for f,h in pins.items()):raise ValueError('changed source')
 result=dict(schema='frc-observed-approach-brake/1',inputs=pins,triggerX=trigger,scenario=scenario,stepS=dt,events=events,outcome=outcome,trace=trace,final=row,maxAbsEnergyResidualJ=maxres,acceptance=False,limits=['Virtual exact pose observations at100Hz, not actual sensor hardware.','Brake mode is modeled dissipative motor braking, not an instantaneous velocity reset or qualified controller policy.','Ground dwell window is a numerical experiment diagnostic, not capture, retention or FIRST rule.','No lift or launcher connection; all inherited contact, mass, geometry and field limitations remain.'])
 output.parent.mkdir(parents=True,exist_ok=True);temp=output.with_suffix(output.suffix+'.tmp');temp.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');temp.replace(output);print(outcome,maxres,flush=True);return result
if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('--output',type=pathlib.Path,required=True);a.add_argument('--trigger',type=float,default=.25);a.add_argument('--scenario',default='nominal');x=a.parse_args();run(x.output,x.trigger,x.scenario)

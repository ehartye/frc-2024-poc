"""Fresh current-CAD drive-to-loose-NOTE experiment with shared reaction and power."""
import pathlib,sys,json,hashlib,argparse,importlib.util,math
R=pathlib.Path(__file__).resolve().parents[2];sys.path[:0]=[str(R/'sim/integrated'),str(R/'sim/acquisition')]
import coupled_acquisition as kernel
import run_free_planar,free_geometry,obstacle_gate
spec=importlib.util.spec_from_file_location('drive_source_validation',R/'sim/drivetrain/run.py');drive_source=importlib.util.module_from_spec(spec);spec.loader.exec_module(drive_source)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def run(output,dt=.001,horizon=1.,scenario="nominal"):
 if type(dt) not in (int,float) or not math.isfinite(dt) or not 0<dt<=.0025:raise ValueError("finite timestep domain")
 if type(horizon) not in (int,float) or not math.isfinite(horizon) or not 0<horizon<=10:raise ValueError("finite horizon domain")
 if scenario not in ["nominal","command-off","no-pinch"]:raise ValueError("scenario")
 if output.exists():raise ValueError('Fresh output required')
 obstacles,floor,pins=run_free_planar.load(R/'sim/acquisition/free-obstacles.json');drive=json.loads((R/'sim/drivetrain/config.json').read_text());drive_source.validate(drive);note=json.loads((R/'sim/acquisition/free-planar-config.json').read_text())['model'];cad=json.loads((R/'cad/assembly/intake_components.json').read_text())
 paths=[pathlib.Path(__file__),R/'sim/integrated/coupled_acquisition.py',R/'sim/integrated/shared_supply.py']+[R/p for p in drive_source.FILES]
 pins.update({p.relative_to(R).as_posix():sha(p) for p in paths})
 gate=obstacle_gate.make_gate(obstacles,cad['note']['majorRadius'],cad['note']['tubeRadius'],cad['floorCenterZ']);geometry=free_geometry.make_geometry(cad,gate)
 initial=dict(p=[.658,0.],v=[0.,0.],yaw=0.,spin=0.,w=[0.,0.]);plant=kernel.CoupledAcquisition(drive,note,geometry,initial);loads=[drive['massKg']*9.80665/2]*2;trace=[];outcome='HORIZON';row=None;maxres=0.;peakforce=0.;start_contact=None
 for i in range(round(horizon/dt)):
  drive_demands=(.2,.2);pinch_demands=(0.,0.) if scenario=="no-pinch" else (.12,.12);modes=("drive","drive")
  if scenario=="command-off" and plant.t>=.6:drive_demands=(0.,0.);pinch_demands=(0.,0.);modes=("coast","coast")
  try:row=plant.step(dt,drive_demands,pinch_demands,loads,modes=modes)
  except (kernel.DomainExit,ValueError) as exc:outcome='UNKNOWN_DOMAIN: '+str(exc);break
  row['commands']=dict(drive=list(drive_demands),pinch=list(pinch_demands),modes=list(modes))
  maxres=max(maxres,abs(row['energyJ']['residual']));peakforce=max(peakforce,sum(f*f for f in row['reactionForceWorldN'])**.5)
  if row['contacts'] and start_contact is None:start_contact=dict(timeS=row['timeS'],relativeRobotXY=row['relativeRobotXY'])
  if i%5==0:trace.append(row)
  if i%100==0:print(i,row['timeS'],row['relativeRobotXY'],flush=True)
 if row is not None and (not trace or trace[-1]['timeS']!=row['timeS']):trace.append(row)
 if any(sha(R/p)!=h for p,h in pins.items()):raise ValueError('Inputs changed')
 report=dict(schema='frc-coupled-drive-acquisition/1',inputs=pins,stepS=dt,horizonS=horizon,scenario=scenario,outcome=outcome,firstContact=start_contact,trace=trace,final=row,maxAbsEnergyResidualJ=maxres,peakReactionForceN=peakforce,acceptance=False,limits=['One flat loose NOTE, planar chassis and wheel slip with prescribed normal-load distribution.','No prescribed chassis velocity, virtual capture, NOTE position reset or independently stiff mechanism battery.','Tire/foam compliance/friction,45kg mass and inertia are hypotheses; no physical calibration.','Known conservative hard-obstacle candidates stop the model rather than declaring actual collision or clearance.','No lift, launcher, field collision, closed-loop acquisition success, score or realtime claim.'])
 output.parent.mkdir(parents=True,exist_ok=True);temp=output.with_suffix(output.suffix+'.tmp');temp.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');temp.replace(output);print(outcome,maxres,peakforce,flush=True);return report
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--output',type=pathlib.Path,required=True);p.add_argument('--step',type=float,default=.001);p.add_argument('--horizon',type=float,default=1.);p.add_argument('--scenario',default='nominal');a=p.parse_args();run(a.output,a.step,a.horizon,a.scenario)

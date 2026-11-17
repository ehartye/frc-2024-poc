"""Free planar NOTE response to hypothetical finite shear patches, not rigid attachment."""
import pathlib,json,hashlib,math,sys
import numpy as np
from wrench import quintic
R=pathlib.Path(__file__).resolve().parents[2];sys.path.insert(0,str(R/'cad/assembly'));import extract_retention_patch as src
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def simulate(c,geo,scenario):
 c=dict(c)
 if type(c['lockedBelts']) is not bool:raise ValueError('Boolean belt boundary required')
 for k,value in c.items():
  if k not in ['lockedBelts','assumptions'] and (type(value) not in [int,float] or not math.isfinite(value)):raise ValueError('Finite configuration:'+k)
 for k in ['stepS','majorM','minorM','massKg','rotationS','liftS','shearStiffnessPerSide','beltRadiusM','rotorInertiaKgM2','groundStiffnessNpm']:
  if c[k]<=0:raise ValueError('Positive configuration:'+k)
 if min(c['mu'],c['normalSideN'],c['groundDampingNspm'])<0:raise ValueError('Nonnegative contact parameters')
 if scenario=='coast':c['lockedBelts']=False
 elif scenario=='fast':c['rotationS']=.15;c['liftS']=.2
 elif scenario=='low-preload':c['normalSideN']=2.
 elif scenario=='half-step':c['stepS']/=2
 elif scenario=='quarter-step':c['stepS']/=4
 elif scenario!='locked-hypothesis':raise ValueError('Scenario')
 dt=c['stepS'];major=c['majorM'];minor=c['minorM'];mass=c['massKg'];inertia=mass*(major**2/2+5*minor**2/8);pitch=c['pitchRad'];tr=c['rotationS'];tl=c['liftS'];stage=c['stageX'];d=geo['noteLongitudinalOffsetMm']/1000
 p=np.array([[v/1000 for v in q['cradlePointMm']] for q in geo['points']]);sides=np.array([q['side'] for q in geo['points']]);shear=np.zeros((len(p),2));K=c['shearStiffnessPerSide']/15;cap=c['mu']*c['normalSideN']/15
 center=np.array([stage+d,minor]);v=np.array([c.get('initialWorldVx',0.),0.]);phi=0.;omega=0.;rotors=np.zeros(2);t=0.;trace=[];tick=0;external=friction=groundheat=0.;maxres=0.;maxpenetration=0.;maxhold=0.;outcome='WITHIN_NOMINAL_PATCH_DOMAIN_HORIZON'
 while t<tr+tl-1e-10:
  if t<=tr:
   q,dq,ddq=quintic(t,tr);theta=pitch*q;tw=pitch*dq;zc=minor+major*math.sin(theta);vz=major*math.cos(theta)*tw
  else:
   q,dq,ddq=quintic(t-tr,tl);theta=pitch;tw=0.;zc=minor+major*math.sin(pitch)+c['liftDistance']*q;vz=c['liftDistance']*dq
  co,si=math.cos(theta),math.sin(theta);rot=np.array([[co,-si],[si,co]]);lever=p[:,[0,2]]@rot.T;points=lever+np.array([stage,zc]);cradle_v=np.column_stack((-tw*lever[:,1],tw*lever[:,0]+vz))
  offset=points-center;cp,sp=math.cos(phi),math.sin(phi);local=offset@np.array([[cp,-sp],[sp,cp]])
  implicit=(np.sqrt(local[:,0]**2+p[:,1]**2)-major)**2+local[:,1]**2-minor**2
  lost=np.where(implicit>1e-10)[0]
  if len(lost):outcome='NOMINAL_FOOTPRINT_DOMAIN_LOST';break
  body_v=v+np.column_stack((-omega*offset[:,1],omega*offset[:,0]));belt_axis=np.array([co,si]);material_v=cradle_v.copy()
  for index,side in enumerate([-1,1]):material_v[sides==side]+=c['beltRadiusM']*rotors[index]*belt_axis
  relative=material_v-body_v;trial=shear+dt*relative;mag=np.linalg.norm(trial,axis=1);factor=np.minimum(1.,cap/(K*np.maximum(mag,1e-30)));new_s=trial*factor[:,None];forces=K*new_s;plastic=trial-new_s;friction+=float(np.sum(forces*plastic));shear=new_s
  for index,side in enumerate([-1,1]):
   torque=-c['beltRadiusM']*float(np.sum(forces[sides==side]@belt_axis));maxhold=max(maxhold,abs(torque))
   if not c['lockedBelts']:rotors[index]+=dt*torque/c['rotorInertiaKgM2']
  sign=0. if abs(math.sin(phi))<1e-12 else math.copysign(1.,math.sin(phi));xcontact=-major*sign*math.cos(phi);gap=center[1]-major*abs(math.sin(phi))-minor;penetration=max(0.,-gap);maxpenetration=max(maxpenetration,penetration);vlower=v[1]+xcontact*omega
  elastic=c['groundStiffnessNpm']*penetration;ground=max(0.,elastic-c['groundDampingNspm']*vlower) if penetration>0 else 0.;groundheat+=-(ground-elastic)*vlower*dt
  total=forces.sum(axis=0)+np.array([0.,ground-mass*9.80665]);torque=float(np.sum(offset[:,0]*forces[:,1]-offset[:,1]*forces[:,0]))+xcontact*ground
  nv=v+dt*total/mass;nw=omega+dt*torque/inertia;center+=dt*(v+nv)/2;phi+=dt*(omega+nw)/2;v=nv;omega=nw
  external+=float(np.sum(forces*cradle_v))*dt;t+=dt
  kinetic=.5*mass*float(v@v)+.5*inertia*omega**2+.5*c['rotorInertiaKgM2']*float(rotors@rotors);potential=mass*9.80665*(center[1]-minor);spring=.5*K*float(np.sum(shear**2));newgap=center[1]-major*abs(math.sin(phi))-minor;floorenergy=.5*c['groundStiffnessNpm']*min(0.,newgap)**2;residual=external-friction-groundheat-kinetic-potential-spring-floorenergy;maxres=max(maxres,abs(residual))
  row=dict(timeS=t,noteCenterXZ=center.tolist(),notePitchRad=phi,cradlePitchRad=theta,noteVelocityXZ=v.tolist(),noteOmegaRadps=omega,rotorOmegaRadps=rotors.tolist(),maxShearM=float(np.linalg.norm(shear,axis=1).max()),groundPenetrationM=max(0.,-newgap),energyJ=dict(prescribedCradleWork=external,slipHeat=friction,groundHeat=groundheat,kinetic=kinetic,potential=potential,shearElastic=spring,groundElastic=floorenergy,residual=residual))
  if tick%20==0:trace.append(row)
  tick+=1
 if not trace or trace[-1]['timeS']!=row['timeS']:trace.append(row)
 return dict(scenario=scenario,outcome=outcome,domainLostPointIndices=lost.tolist() if len(lost) else [],final=row,trace=trace,maxAbsEnergyResidualJ=maxres,maxGroundPenetrationM=maxpenetration,maxBeltHoldTorqueNm=maxhold,acceptance=False)

def run(output):
 if output.exists():raise ValueError('Fresh output required')
 gp=R/'cad/assembly/evidence/retention-patch.json';g=json.loads(gp.read_text());cp=R/'sim/retention/finite-slip-config.json';c=json.loads(cp.read_text())
 if g['inputs']!=src.pins():raise ValueError('Stale patch evidence')
 cad=json.loads((R/'cad/assembly/intake_components.json').read_text());major=cad['note']['majorRadius']/1000;minor=cad['note']['tubeRadius']/1000;pitch=math.radians(cad['pitchDeg']);expected={'majorM':major,'minorM':minor,'pitchRad':pitch,'stageX':cad['stageCenterX']/1000,'liftDistance':cad['handoffCenterZ']/1000-minor-major*math.sin(pitch),'beltRadiusM':cad['pinch']['teeth']*cad['pinch']['pitch']/(2000*math.pi)}
 if any(not math.isclose(c[k],v,rel_tol=0,abs_tol=1e-12) for k,v in expected.items()):raise ValueError('Retention/CAD geometry mismatch')
 geo=next(x for x in g['cases'] if x['state']=='floor' and x['noteLongitudinalOffsetMm']==9.88835038313412)
 ground_path=R/'sim/acquisition/evidence.json';ground=json.loads(ground_path.read_text());final=next(x for x in ground['cases'] if x['case']=='braked-hypothesis')['final']
 if any(sha(R/p)!=h for p,h in ground['inputs'].items()):raise ValueError('Stale ground state')
 if not math.isclose(final['relativeRobotX'],c['stageX']+geo['noteLongitudinalOffsetMm']/1000,abs_tol=1e-12) or final['robotSpeedMps']!=0:raise ValueError('Ground handoff registration')
 c['initialWorldVx']=final['pieceWorldSpeedMps']
 if len(geo['points'])!=30 or any(sum(x['side']==side for x in geo['points'])!=15 for side in [-1,1]):raise ValueError('Complete patch points required')
 files=[pathlib.Path(__file__),R/'sim/retention/wrench.py',gp,cp,ground_path];pins=dict(src.pins(),**ground['inputs'],**{p.relative_to(R).as_posix():sha(p) for p in files})
 cases=[simulate(c,geo,n) for n in ['locked-hypothesis','coast','fast','low-preload','half-step','quarter-step']]
 if any(sha(R/p)!=h for p,h in pins.items()):raise ValueError('Inputs changed')
 report=dict(schema='frc-finite-patch-slip/1',inputs=pins,cases=cases,acceptance=False,limits=c['assumptions']);output.mkdir(parents=True);(output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');return report
if __name__=='__main__':
 import argparse;a=argparse.ArgumentParser();a.add_argument('--output',type=pathlib.Path,required=True);args=a.parse_args();print([(x['scenario'],x['outcome'],x['final']['timeS']) for x in run(args.output)['cases']])

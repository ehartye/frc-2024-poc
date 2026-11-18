"""Continue the exact attained ground stop under declared static friction."""
import argparse,copy,hashlib,json,pathlib,sys,time
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def run(a):
 sys.path[:0]=[str(a.project/f'sim/{x}') for x in ['pressure','integrated','drivetrain','actuation','acquisition']]
 import native_static_lift as lift
 import native_dry_friction as dry
 from signed_pressure_acquisition import DomainExit
 load=lambda p:json.loads(p.read_bytes());source=load(a.source);binding=load(a.binding);contract=load(a.contract);base=load(a.base_contract);dc=load(a.project/'sim/drivetrain/config.json');nc=load(a.project/'sim/acquisition/free-planar-config.json')['model'];cad=load(a.project/'cad/assembly/intake_components.json');ac=load(a.project/'sim/actuation/config.json')
 assert source['outcome']=='ACTIVE_STOP_HORIZON' and source['settledLast100ms'] and source['bindingSha256']==sha(a.binding);m=lift.NativeStaticLift(source['final'],dc,nc,cad,binding,ac,base,friction_mu=a.mu,solver_tolerance=contract['solverGapToleranceJ'],solver_iterations=contract['solverMaxIterations']);initial=copy.deepcopy(m.s);join=m.start_mechanical-source['trace'][-1]['energyJ']['kinetic']-source['trace'][-1]['energyJ']['spring'];assert abs(join)<1e-12
 files={'runner':pathlib.Path(__file__),'model':pathlib.Path(lift.__file__),'friction':pathlib.Path(dry.__file__),'sourceReceipt':a.source,'binding':a.binding,'contract':a.contract,'baseContract':a.base_contract};pins={k:sha(p) for k,p in files.items()};assert not a.output.exists();a.output.mkdir(parents=True);rows=[];events=[];bus=12.;start=time.perf_counter();outcome='HORIZON';reason=None;command=None
 for step in range(round(a.duration/a.dt)):
  if step%round(.01/a.dt)==0:
   tick=step//round(.01/a.dt);t=tick*.01
   if tick<20:zdes=m.z0+.05*t*t;vdes=.1*t;ades=.1
   elif tick<50:zdes=m.z0+.002+.02*(t-.2);vdes=.02;ades=0.
   else:zdes=m.z0+.008;vdes=ades=0.
   s=m.s;d=s['chassis'];con=m.contact(s,{});terms=m.act.terms([d['x'],s['zPivot'],s['theta']],[d['vx'],s['vPivot'],s['thetaRate']]);desired=[0.,ades+100*(zdes-s['zPivot'])+20*(vdes-s['vPivot']),-100*s['theta']-20*s['thetaRate']];reaction=[con['q']['cradleReactionWorldN'][0]-m.last_friction_force[0],con['q']['cradleReactionWorldN'][2]-m.last_friction_force[2],-con['q']['cradleMomentWorldNm'][1]-m.last_friction_moment[1]];required=[sum(terms['massMatrix'][i][j]*desired[j] for j in range(3))-terms['biasForce'][i]-reaction[i] for i in range(3)];duties=[]
   for i in range(2):
    amps=required[i+1]/terms['transmissionGains'][i]/ac['motorK'];voltage=terms['rotorOmegaRadps'][i]*ac['motorK']+amps*ac['resistanceOhm'];duties.append(float(max(-.35,min(.35,voltage/bus))))
   command=dict(drive=[0.,0.],driveModes=['brake','brake'],pinch=[0.,0.],actuator=duties,actuatorModes=['coast','coast'] if a.coast else ['drive','drive'],requestedPivotZM=zdes,requestedPivotVelocityMps=vdes);events.append(dict(timeS=s['timeS'],command=copy.deepcopy(command)))
  before=copy.deepcopy(m.s)
  try:row=m.step(a.dt,command)
  except (ValueError,DomainExit) as e:assert m.s==before;outcome='MODEL_DOMAIN_EXIT';reason=str(e);break
  rows.append(row);bus=row['busV']
  if abs(row['energyJ']['residual']-source['trace'][-1]['energyJ']['residual'])>contract['comparisonBounds']['incrementalEnergyJ']:outcome='ENERGY_GATE_EXIT';break
  if (step+1)%round(.05/a.dt)==0:print(json.dumps(dict(elapsedS=row['timeS']-initial['timeS'],riseM=row['state']['noteZ']-m.z0,pivotRiseM=row['state']['zPivot']-m.z0,relativeVzMps=row['state']['noteVz']-row['state']['vPivot'],residualJ=row['energyJ']['residual'],iterations=row['friction']['iterations'])),flush=True)
 assert all(sha(p)==pins[k] for k,p in files.items());s=m.s;gate=contract['retentionGate'];retained=outcome=='HORIZON' and a.duration>=.8 and s['noteZ']-m.z0>=gate['minimumFinalRiseM'] and abs(s['noteZ']-s['zPivot'])<=gate['maximumRelativeVerticalDriftM'] and abs(s['noteVz']-s['vPivot'])<=gate['maximumFinalRelativeVerticalSpeedMps'];r=dict(schema='native-static-lift-run/1',inputs=pins,stepS=a.dt,mu=a.mu,actuatorsCoast=a.coast,requestedDurationS=a.duration,initial=initial,final=copy.deepcopy(m.s),initialMechanicalDifferenceJ=join,inheritedResidualJ=source['trace'][-1]['energyJ']['residual'],outcome=outcome,reason=reason,retentionGatePassed=retained,events=events,trace=rows,runtimeSeconds=time.perf_counter()-start,acceptance=False,limits=contract['limits']);(a.output/'receipt.json').write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps({k:r[k] for k in ['outcome','reason','runtimeSeconds','retentionGatePassed']}),flush=True)
if __name__=='__main__':
 p=argparse.ArgumentParser()
 for n in ['project','source','binding','contract','base-contract','output']:p.add_argument('--'+n,type=pathlib.Path,required=True)
 p.add_argument('--dt',type=float,default=.0005);p.add_argument('--duration',type=float,default=.002);p.add_argument('--mu',type=float,default=.6);p.add_argument('--coast',action='store_true');a=p.parse_args();assert a.dt in [.0005,.00025,.000125] and a.duration in [.002,.1,.8,2.] and a.mu in [0.,.1,.6];run(a)

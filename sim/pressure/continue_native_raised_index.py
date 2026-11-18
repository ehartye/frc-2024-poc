"""Finite belt motion from an attained raised NOTE; no pose or energy reset."""
import argparse,copy,hashlib,json,math,pathlib,sys,time
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def run(a):
 sys.path[:0]=[str(a.project/f'sim/{x}') for x in ['pressure','integrated','drivetrain','actuation','acquisition']]
 import native_static_lift as lift
 import native_dry_friction as dry
 from signed_pressure_acquisition import DomainExit
 load=lambda p:json.loads(p.read_bytes());source=load(a.source);lc=load(a.lift_contract);c=load(a.contract);binding=load(a.binding);base=load(a.base_contract);dc=load(a.project/'sim/drivetrain/config.json');nc=load(a.project/'sim/acquisition/free-planar-config.json')['model'];cad=load(a.project/'cad/assembly/intake_components.json');ac=load(a.project/'sim/actuation/config.json');assert source['retentionGatePassed'] and source['outcome']=='HORIZON' and source['requestedDurationS']==2. and source['inputs']['contract']==sha(a.lift_contract) and source['inputs']['binding']==sha(a.binding)
 m=lift.NativeStaticLift(source['initial'],dc,nc,cad,binding,ac,base,friction_mu=source['mu'],solver_tolerance=lc['solverGapToleranceJ'],solver_iterations=lc['solverMaxIterations']);m.s=copy.deepcopy(source['trace'][-2]['state']);m.ledger=copy.deepcopy(m.s['ledger']);last=m.step(source['stepS'],source['events'][-1]['command']);assert m.s==source['final'];replay_residual_difference=last['energyJ']['residual']-source['trace'][-1]['energyJ']['residual'];assert abs(replay_residual_difference)<1e-12;left=copy.deepcopy(last);right=copy.deepcopy(source['trace'][-1]);left['energyJ'].pop('residual');right['energyJ'].pop('residual');assert left==right;initial=copy.deepcopy(m.s);join=m.mechanical(m.s,m.contact(m.s,{}))-source['trace'][-1]['energyJ']['mechanical'];assert abs(join)<1e-12;initial_x=initial['note']['p'][0]-initial['chassis']['x'];target_x=initial_x+c['targetRelativeDisplacementM'];assert .178<target_x<.20
 files={'runner':pathlib.Path(__file__),'model':pathlib.Path(lift.__file__),'friction':pathlib.Path(dry.__file__),'sourceReceipt':a.source,'binding':a.binding,'contract':a.contract,'liftContract':a.lift_contract,'baseContract':a.base_contract};pins={k:sha(p) for k,p in files.items()};assert not a.output.exists();a.output.mkdir(parents=True);rows=[];events=[];bus=source['trace'][-1]['busV'];start=time.perf_counter();outcome='HORIZON';reason=None
 for step in range(round(c['durationS']/a.dt)):
  if step%round(.01/a.dt)==0:
   tick=step//round(.01/a.dt);t=tick*.01
   if tick<20:dx=-.025*t*t;vx=-.05*t;ax=-.05
   elif tick<40:dx=-.001-.01*(t-.2);vx=-.01;ax=0.
   elif tick<60:u=t-.4;dx=-.003-.01*u+.025*u*u;vx=-.01+.05*u;ax=.05
   else:dx=-.004;vx=ax=0.
   s=m.s;d=s['chassis'];n=s['note'];con=m.contact(s,{});terms=m.act.terms([d['x'],s['zPivot'],s['theta']],[d['vx'],s['vPivot'],s['thetaRate']]);desired=[0.,100*(m.z0+.008-s['zPivot'])-20*s['vPivot'],-100*s['theta']-20*s['thetaRate']];reaction=[con['q']['cradleReactionWorldN'][0]-m.last_friction_force[0],con['q']['cradleReactionWorldN'][2]-m.last_friction_force[2],-con['q']['cradleMomentWorldNm'][1]-m.last_friction_moment[1]];required=[sum(terms['massMatrix'][i][j]*desired[j] for j in range(3))-terms['biasForce'][i]-reaction[i] for i in range(3)];duties=[]
   for i in range(2):
    amps=required[i+1]/terms['transmissionGains'][i]/ac['motorK'];voltage=terms['rotorOmegaRadps'][i]*ac['motorK']+amps*ac['resistanceOhm'];duties.append(float(max(-.35,min(.35,voltage/bus))))
   omega_target=-(vx+c['positionGainPerS']*(initial_x+dx-(n['p'][0]-d['x'])))/nc['radiusM'];pinch=[]
   for w in n['w']:
    alpha=-ax/nc['radiusM']+c['rotorGainPerS']*(omega_target-w);torque=(nc['rotorInertiaKgM2']+.5*nc['pieceMassKg']*nc['radiusM']**2)*alpha+.5*nc['radiusM']*(con['q']['noteForceWorldN'][0]+con['floorForce'][0]);amps=nc['kvRadpsPerV']*torque;voltage=w/nc['kvRadpsPerV']+amps*nc['motorResistanceOhm']
    if amps*voltage<0:voltage=-math.copysign(.001,w)
    pinch.append(float(max(-c['pinchDutyLimit'],min(c['pinchDutyLimit'],voltage/bus))))
   command=dict(drive=[0.,0.],driveModes=['brake','brake'],pinch=pinch,actuator=duties,actuatorModes=['drive','drive'],requestedRelativeXM=initial_x+dx,requestedRelativeVelocityMps=vx);events.append(dict(timeS=s['timeS'],command=copy.deepcopy(command)))
  before=copy.deepcopy(m.s)
  try:row=m.step(a.dt,command)
  except (ValueError,DomainExit) as e:assert m.s==before;outcome='MODEL_DOMAIN_EXIT';reason=str(e);break
  rows.append(row);bus=row['busV']
  if abs(row['energyJ']['residual']-source['trace'][-1]['energyJ']['residual'])>lc['comparisonBounds']['incrementalEnergyJ']:outcome='ENERGY_GATE_EXIT';break
  if (step+1)%round(.1/a.dt)==0:print(json.dumps(dict(elapsedS=row['timeS']-initial['timeS'],relativeXM=row['state']['note']['p'][0]-row['state']['chassis']['x'],riseM=row['state']['noteZ']-m.z0,residualJ=row['energyJ']['residual'])),flush=True)
 assert all(sha(p)==pins[k] for k,p in files.items());r=dict(schema='native-raised-index/1',inputs=pins,stepS=a.dt,initial=initial,final=copy.deepcopy(m.s),initialRelativeXM=initial_x,targetRelativeXM=target_x,initialMechanicalDifferenceJ=join,lastSourceStateReplayedExactly=True,lastSourceResidualDifferenceJ=replay_residual_difference,initialControllerReactionReconstructed=True,inheritedResidualJ=source['trace'][-1]['energyJ']['residual'],outcome=outcome,reason=reason,trace=rows,events=events,runtimeSeconds=time.perf_counter()-start,acceptance=False,limits=c['limits']);(a.output/'receipt.json').write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps({k:r[k] for k in ['outcome','reason','runtimeSeconds']}),flush=True)
if __name__=='__main__':
 p=argparse.ArgumentParser()
 for name in ['project','source','binding','lift-contract','base-contract','contract','output']:p.add_argument('--'+name,type=pathlib.Path,required=True)
 p.add_argument('--dt',type=float,default=.0005);a=p.parse_args();assert a.dt in [.0005,.00025];run(a)

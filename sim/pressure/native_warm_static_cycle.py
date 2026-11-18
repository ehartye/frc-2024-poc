"""Identical cycle equations with a checked numerical friction warm start.

The cache seeds the optimizer only; it is not contact elasticity or physical
memory. The same original cone, optimality-gap and work checks still apply.
"""
import copy
import numpy as np
from native_static_cycle import NativeStaticCycle as ColdCycle,lift_duties
from native_pressure_lift import LiftDomainError
from native_dry_friction import contact_jacobian
from native_warm_friction import WarmSolve
from shared_supply import solve_supply

class NativeStaticCycle(ColdCycle):
 def __init__(self,*args,**kwargs):
  super().__init__(*args,**kwargs)
  self.friction_solver=WarmSolve()
 def step(self,dt,cmd):
  if dt not in [.0005,.00025,.000125]:raise ValueError('declared timestep')
  start=self.s;cache={};contact=self.contact(start,cache)
  def port(bus):
   de,pe,ae=self.electrical(start,bus,cmd);return {'busPowerW':de['busPowerW']+sum(x['terminalPowerW'] for x in pe)+ae['busPowerW']}
  supply=solve_supply({'native-static-lift':port},12.,.02);r=self.rates(start,supply['busV'],cmd,cache,contact)
  if r['infeasible']:raise LiftDomainError('ACTUATOR_VOLTAGE_DOMAIN')
  p=contact['pressure'];j,basis=contact_jacobian(p,self.nc['radiusM']);mass=self.mass(start);acc=np.array([*r['acc'],*r['noteAccel'],r['notePitchAccel'],*r['pinchAccel']]);old_velocity=self.velocity(start);free=old_velocity+dt*acc
  result=self.friction_solver(j,mass,free,dt*self.friction_mu*p['normalN'],gap_tolerance=self.solver_tolerance,max_iterations=self.solver_iterations);velocity=result['velocity'];impulses=np.einsum('ni,nij->nj',result['impulse'],basis);force=np.sum(impulses,axis=0)/dt;note_moment=np.sum(np.cross(p['noteLeverWorldM'],impulses),axis=0)/dt;cradle_moment=np.sum(np.cross(p['cradleLeverWorldM'],-impulses),axis=0)/dt
  if max(abs(force[1]),abs(note_moment[0]),abs(note_moment[2]),abs(cradle_moment[0]),abs(cradle_moment[2]))>1e-8:raise LiftDomainError('UNSUPPORTED_DRY_NONPLANAR_WRENCH')
  end=copy.deepcopy(start);end['timeS']+=dt;end['chassis']['t']+=dt
  d=end['chassis'];n=end['note'];d['vx']=float(velocity[0]);end['vPivot']=float(velocity[1]);end['thetaRate']=float(velocity[2]);n['v'][0]=float(velocity[3]);end['noteVz']=float(velocity[4]);end['notePitchRate']=float(velocity[5]);n['w']=velocity[6:].tolist();d['wheel']=[w+dt*a for w,a in zip(start['chassis']['wheel'],r['wheelAccel'])]
  d['x']+=dt*d['vx'];d['y']+=dt*d['vy'];d['heading']+=dt*d['yaw_rate'];d['wheel_angle']=[a+dt*w for a,w in zip(d['wheel_angle'],d['wheel'])];n['p']=[x+dt*v for x,v in zip(n['p'],n['v'])];n['yaw']+=dt*n['spin']
  for q,v in [('zPivot','vPivot'),('theta','thetaRate'),('noteZ','noteVz'),('notePitch','notePitchRate')]:end[q]+=dt*end[v]
  final=self.contact(end,{});rates=dict(r['energyRates'],sourceWork=supply['sourcePowerW'],sourceHeat=supply['sourceHeatW']);ledger={k:self.ledger[k]+dt*rates[k] for k in self.ledger};ledger['beltSlip']+=result['heatJ'];mechanical=self.mechanical(end,final);residual=self.initial['initialEnergyJ']+ledger['sourceWork']-sum(v for k,v in ledger.items() if k!='sourceWork')-mechanical
  end['ledger']=ledger
  row=dict(timeS=end['timeS'],state=copy.deepcopy(end),busV=supply['busV'],sourcePowerResidualW=supply['powerResidualW'],pressureWorkResidualW=final['q']['workResidualW'],floorForceN=final['floorForce'].tolist(),normalContactForceN=final['q']['noteForceWorldN'],appliedDryForceN=force.tolist(),normalN=[x['normalMagnitudeN'] for x in final['pressure']['sideContacts']],contactCount=final['pressure']['contacts'],energyJ=dict(ledger,mechanical=mechanical,pressure=final['pressure']['potentialJ'],floor=final['floorEnergy'],residual=residual),friction=dict(iterations=result['iterations'],gapJ=result['gapJ'],heatJ=result['heatJ'],projectionDissipationJ=result['projectionDissipationJ'],projectionIdentityJ=result['projectionIdentityJ'],maximumConeExcessNs=result['maximumConeExcessNs'],maximumFinalTangentialSpeedMps=float(np.max(np.linalg.norm(np.einsum('nik,k->ni',j,velocity),axis=1),initial=0))),acceptance=False)
  # Commit only after all geometry, constitutive and electrical checks pass.
  self.s=end;self.ledger=ledger;self.last_friction_force=force;self.last_friction_moment=cradle_moment;return row

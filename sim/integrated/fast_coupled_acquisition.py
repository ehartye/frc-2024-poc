"""Additive explicit fast coupled step; reference model remains unchanged."""
import copy,math
from coupled_acquisition import CoupledAcquisition,acquisition_rates
from shared_supply import solve_supply
from electrical_port import make_port
from free_planar import DomainExit

class FastCoupledAcquisition(CoupledAcquisition):
 def step(self,dt,drive_demands,pinch_demands,normal_loads,source_v=12.,source_r=.02,modes=('drive','drive')):
  self.drive._validate_step(dt,normal_loads,(0.,0.),0.)
  if len(pinch_demands)!=2 or any(type(d) not in (int,float) or not math.isfinite(d) or not 0<=d<=1 for d in pinch_demands):raise ValueError('pinch duties')
  cache={}
  def cached_geometry(q):
   key=tuple(q)
   if key not in cache:cache[key]=self.geometry(q)
   return cache[key]
  port=make_port(self,dt,drive_demands,pinch_demands,normal_loads,modes,geometry=cached_geometry)
  supply=solve_supply({'robot':port},source_v,source_r);p=self.preview(dt,drive_demands,pinch_demands,supply['busV'],normal_loads,modes,geometry=cached_geometry)
  if p['b']['electrical']['infeasibleCurrentLimitVoltage'] or p['startB']['electrical']['infeasibleCurrentLimitVoltage']:raise DomainExit('Drive controller voltage domain')
  # Reject a domain exit before mutating ANY persistent state.
  end=acquisition_rates(p['note'],p['drive'],self.c,self.geometry,supply['busV'],pinch_demands)
  if self.initial_energy is None:self.initial_energy=self.drive.kinetic()+self.note_kinetic(self.note)+p['startA']['springJ']
  b=p['b']['energyRates'];a=p['a'];rates=dict(sourceWork=supply['sourcePowerW'],sourceHeat=supply['sourceHeatW'],driveWinding=b['windingHeat'],driveBrake=b['controllerBrakeHeat'],driveDrag=b['motorDragHeat']+b['gearDragHeat'],tireSlip=b['tireSlipHeat'],pinchWinding=a['windingHeatW'],normalHeat=a['normalHeatW'],beltSlip=a['slipHeatW'],groundHeat=a['groundHeatW'])
  for k,v in rates.items():self.energy[k]+=dt*v
  self.drive.state=p['drive'];self.note=p['note'];self.t+=dt;kinetic=self.drive.kinetic()+self.note_kinetic(self.note);residual=self.initial_energy+self.energy['sourceWork']-sum(v for k,v in self.energy.items() if k!='sourceWork')-kinetic-end['springJ']
  return dict(timeS=self.t,chassis=copy.deepcopy(self.drive.state),note=copy.deepcopy(self.note),relativeRobotXY=end['relativeRobotXY'],busV=supply['busV'],busCurrentA=supply['busCurrentA'],contacts=a['contacts'],reactionForceWorldN=a['reactionForceWorldN'],reactionYawNm=a['reactionYawNm'],energyJ=dict(self.energy,kinetic=kinetic,spring=end['springJ'],initial=self.initial_energy,residual=residual),acceptance=False)

"""Fresh floor acquisition using one declared XZ contact model throughout.

The four native belt/tread solids define the pressure field. The extended
vertical/pitch corridor is a simulation hypothesis, not native clearance proof.
"""
import copy
import numpy as np
from native_static_lift import NativeStaticLift
from native_pressure_lift import LiftDomainError
from patch_registration import rotation
from plant import DrivePlant
import native_pressure_arrays as arrays

class NativeStaticCycle(NativeStaticLift):
 def __init__(self,drive,note,cad,binding,actuator,contract,mu=.6):
  checkpoint=dict(timeS=0.,chassis=DrivePlant(drive).state,note=dict(p=[contract['initialNoteXM'],0.],v=[0.,0.],yaw=0.,spin=0.,w=[0.,0.]),ledger={k:0. for k in ['sourceWork','sourceHeat','driveWinding','driveBrake','driveDrag','tireSlip','pinchWinding','normalHeat','beltSlip','groundHeat']},initialEnergyJ=0.)
  super().__init__(checkpoint,drive,note,cad,binding,actuator,contract,friction_mu=mu,solver_tolerance=contract['solverGapToleranceJ'],solver_iterations=contract['solverMaxIterations'])
  assert self.start_mechanical==0. and self.contact(self.s,{})['pressure']['contacts']==0
 def geometry(self,s,cache):
  d=s['chassis'];n=s['note'];x=n['p'][0]-d['x']
  if max(abs(d['heading']),abs(n['yaw']),abs(d['vy']),abs(d['yaw_rate']),abs(n['v'][1]),abs(n['spin']))>1e-8:raise LiftDomainError('UNSUPPORTED_NONPLANAR_STATE')
  if not .178-1e-12<=x<=.658+1e-12 or abs(s['zPivot']-self.z0)>.02 or max(abs(s['theta']),abs(s['notePitch']))>.02:raise LiftDomainError('DECLARED_CYCLE_ENVELOPE_EXIT')
  key=(d['x'],*n['p'],s['zPivot'],s['theta'],s['noteZ'],s['notePitch'])
  if key not in cache:
   cache[key]=arrays.prepare(self.cad,self.binding['effectiveGeometryMm'],note_center=[*n['p'],s['noteZ']],note_basis=rotation(0.,s['notePitch']),cradle_origin=[d['x']+self.cad['stageCenterX']/1000,0.,s['zPivot']],cradle_basis=rotation(0.,s['theta']),resolution=tuple(self.c['resolution']),k_area=1e7)
  return cache[key]

def lift_duties(m,bus,zdes,vdes=0.,ades=0.):
 s=m.s;d=s['chassis'];con=m.contact(s,{});terms=m.act.terms([d['x'],s['zPivot'],s['theta']],[d['vx'],s['vPivot'],s['thetaRate']]);desired=[0.,ades+100*(zdes-s['zPivot'])+20*(vdes-s['vPivot']),-100*s['theta']-20*s['thetaRate']]
 reaction=[con['q']['cradleReactionWorldN'][0]-m.last_friction_force[0],con['q']['cradleReactionWorldN'][2]-m.last_friction_force[2],-con['q']['cradleMomentWorldNm'][1]-m.last_friction_moment[1]]
 required=np.array(terms['massMatrix'])@desired-np.array(terms['biasForce'])-reaction
 duties=[]
 for i in range(2):
  amps=required[i+1]/terms['transmissionGains'][i]/m.ac['motorK'];voltage=terms['rotorOmegaRadps'][i]*m.ac['motorK']+amps*m.ac['resistanceOhm'];duties.append(float(np.clip(voltage/bus,-.35,.35)))
 return duties

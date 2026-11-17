"""Diagnostic-only exception enrichment; no altered force or step equations."""
import copy,math
from coupled_lift_separated_phase import CoupledLift as Reference
import patch_registration as registration
import coupled_lift_guide_adapter as guide
class CoupledLift(Reference):
 def rates(self,s,bus,cmd,cache):
  try:return super().rates(s,bus,cmd,cache)
  except ValueError as error:
   # A failed pure evaluation carries its exact trial input, not an accepted state.
   error.rejectedEvaluation=dict(state=copy.deepcopy(s),busV=bus,commands=copy.deepcopy(cmd),reason=str(error),accepted=False)
   raise

def diagnose(p,rejected):
 if rejected is None:return None
 s=rejected['state'];d=s['drive'];n=s['note'];heading=d['heading'];dx=n['p'][0]-d['x'];dy=n['p'][1]-d['y'];x=math.cos(heading)*dx+math.sin(heading)*dy;y=-math.sin(heading)*dx+math.cos(heading)*dy
 origin=[d['x']+.178*math.cos(heading),d['y']+.178*math.sin(heading),s['zPivot']]
 patch=registration.evaluate(p.patch,origin,registration.rotation(heading,s['theta']),[*n['p'],s['noteZ']],registration.rotation(n['yaw'],s['notePitch']),.1524,.0254)
 result=copy.deepcopy(rejected);result['patchWitnesses']=patch;result['guideQueryInput']=dict(centerRobotXZM=[x,s['noteZ']],pitchRad=s['notePitch'],yM=y);result['guideQuery']=guide.query_round_xz([x,s['noteZ']],s['notePitch'],p.guide,y_m=y)
 result['meaning']='Exact first rejected pure evaluation in supply/step order; not a chronological collision witness or accepted state.'
 return result

"""Draft two-travel rates fork; same friction/rotor law, independent inward velocities."""
import math
import finite_area as a
def rates(prepared,*,note_velocity=(0.,0.,0.),note_omega=(0.,0.,0.),cradle_velocity=(0.,0.,0.),cradle_omega=(0.,0.,0.),u_velocities=(0.,0.),rotor_omega=(0.,0.),pitch_radius_m,mu=.6,slip_scale=.01):
 """Positive acquisition rotor turns drive rearward on the straight inner faces.
 Constant belt speed=pitch radius*rotor speed follows the inherited kinematic
 hypothesis; outer-layer strain around wraps is not a calibrated material model.
 """
 nv=a.vec(note_velocity);nw=a.vec(note_omega);cv=a.vec(cradle_velocity);cw=a.vec(cradle_omega);uds=[a.scalar(x) for x in u_velocities];
 if len(uds)!=2:raise ValueError('positive/negative travel velocities required')
 radius=a.scalar(pitch_radius_m);friction=a.scalar(mu);vscale=a.scalar(slip_scale)
 if len(rotor_omega)!=2:raise ValueError('two rotors')
 rotor=[a.scalar(v) for v in rotor_omega]
 if radius<=0 or friction<0 or vscale<=0:raise ValueError('positive radius/scale and nonnegative friction')
 force=[0.,0.,0.];moment=[0.,0.,0.];reaction=[0.,0.,0.];reactionmoment=[0.,0.,0.];torque=[0.,0.];qu=[0.,0.];normalwork=0.;heat=0.;sidefriction=[0.,0.]
 for c in prepared['contacts']:
  n=c['normalWorld'];t=c['acquisitionTangentWorld'];index=c['motor'];base=a.add(cv,a.cross(cw,c['cradleLeverWorldM']))
  ui=0 if c['side']==1 else 1;direction=a.scale(prepared['cassetteDirectionWorld'],1 if ui==0 else -1);base=a.add(base,a.scale(direction,uds[ui]))
  body=a.add(nv,a.cross(nw,c['noteLeverWorldM']));beltvelocity=a.add(base,a.scale(t,radius*rotor[index]));diff=a.sub(beltvelocity,body);slip=a.sub(diff,a.scale(n,a.dot(diff,n)));speed=math.hypot(*slip)
  if not math.isfinite(speed):raise a.DomainError('NONFINITE_SLIP_SPEED')
  fn=c['normalN'];normal=a.scale(n,fn)
  factor=0. if speed==0 else friction*fn*math.tanh(speed/vscale)/speed;ff=a.scale(slip,factor);f=a.add(normal,ff);back=a.scale(f,-1.)
  force=a.add(force,f);moment=a.add(moment,a.cross(c['noteLeverWorldM'],f));reaction=a.add(reaction,back);reactionmoment=a.add(reactionmoment,a.cross(c['cradleLeverWorldM'],back));torque[index]-=radius*a.dot(ff,t);heat+=a.dot(ff,slip);normalwork+=a.dot(normal,a.sub(body,base));sidefriction[index]+=a.dot(ff,t)
  qu[ui]+=a.dot(back,direction)
 bodypower=a.dot(force,nv)+a.dot(moment,nw);basepower=a.dot(reaction,cv)+a.dot(reactionmoment,cw);upower=a.dot(qu,uds);rotorpower=a.dot(torque,rotor);potentialrate=-normalwork
 result=dict(noteForceWorldN=force,noteMomentWorldNm=moment,cradleReactionWorldN=reaction,cradleMomentWorldNm=reactionmoment,cassetteGeneralizedForceN=qu,rotorContactTorqueNm=torque,bodyPowerW=bodypower,cradlePowerW=basepower,cassettePowerW=upower,rotorPowerW=rotorpower,normalPotentialRateW=potentialrate,slipHeatW=heat,workResidualW=bodypower+basepower+upower+rotorpower+potentialrate+heat,sideBeltAxisTractionN=sidefriction,acceptance=False)
 def check(v):
  if isinstance(v,dict):
   for x in v.values():check(x)
  elif isinstance(v,list):
   for x in v:check(x)
  elif isinstance(v,float) and not math.isfinite(v):raise a.DomainError('NONFINITE_WORK_RESULT')
 check(result);return result

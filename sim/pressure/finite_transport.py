"""Additive finite outer-wrap contact and memoryless belt-friction power interface.
No motor state, spring equilibrium or hidden battery; geometry and rates are pure.
"""
import math
import finite_area as a

def gradient(point,side,g):
 try:return a.shallow_gradient(point,side,g)
 except a.DomainError as error:
  if str(error)!='UNSUPPORTED_ENTRY_WRAP_CONTACT':raise
 # Original helper already rejected deep/cavity; perform its remaining tie check.
 p=a.scale(point,1000.);x,y,z=p;ax=min(g['frontX'],max(g['rearX'],x));dx=x-ax;dy=y-side*g['axisYAbs'];rad=math.hypot(dx,dy);radial=rad-g['outerRadius'];height=abs(z)-g['halfHeight']
 if abs(radial-height)<1e-9:raise a.DomainError('UNSUPPORTED_SDF_GRADIENT_TIE')
 q=a.belt.query(p,side,g)
 return dict(penetrationM=-q['signedDistanceToMaterialMm']/1000.,normal=[dx/rad,dy/rad,0.] if radial>height else [0.,0.,1. if z>0 else -1.],feature='outer-wrap' if radial>height else 'finite-height-cap',sourceSdfM=q['signedDistanceToMaterialMm']/1000.)

def prepare(config,*,note_center,note_basis=a.I,cradle_origin=(0.,0.,0.),cradle_basis=a.I,u_positive_m=0.,k_area=1e7,resolution=(128,128)):
 center=a.vec(note_center);origin=a.vec(cradle_origin);nb=a.basis(note_basis);cb=a.basis(cradle_basis);u=a.scalar(u_positive_m);k=a.scalar(k_area)
 if not 0<=u<=.0015 or k<=0:raise ValueError('source travel and positive penalty density')
 g0=a.belt.geometry(config);contacts=[];potential=0.;features={}
 for local,area in a.material_points(config['note']['majorRadius']/1000.,config['note']['tubeRadius']/1000.,*resolution):
  lever=a.mv(nb,local);point=a.add(center,lever);arm=a.sub(point,origin);localpoint=a.mtv(cb,arm)
  for side in (-1,1):
   g=dict(g0)
   if side==1:g['axisYAbs']-=u*1000.
   q=gradient(localpoint,side,g);depth=q['penetrationM']
   if depth==0:continue
   wall=a.belt.query(a.scale(localpoint,1000.),side,g)['outerSidewall']
   if wall is None or not wall['noteFacingHalf']:raise a.DomainError('UNSUPPORTED_BELT_TANGENT')
   normal=a.mv(cb,q['normal']);tangent=a.mv(cb,wall['acquisitionUnitTangent'])
   if abs(a.dot(normal,tangent))>1e-10:raise a.DomainError('NORMAL_TANGENT_NOT_ORTHOGONAL')
   fn=k*depth*area;energy=.5*k*depth*depth*area
   if not all(math.isfinite(x) for x in [fn,energy]):raise a.DomainError('NONFINITE_CONTACT_RESULT')
   potential+=energy;features[q['feature']]=features.get(q['feature'],0)+1
   contacts.append(dict(side=side,motor=0 if side==-1 else 1,pointWorldM=point,noteLeverWorldM=lever,cradleLeverWorldM=arm,normalWorld=normal,acquisitionTangentWorld=tangent,normalN=fn,areaM2=area,penetrationM=depth,feature=q['feature']))
 return dict(contacts=contacts,potentialJ=potential,features=features,cassetteDirectionWorld=a.mv(cb,[0.,-1.,0.]),resolution=list(resolution),kAreaNpm3=k,acceptance=False)

def rates(prepared,*,note_velocity=(0.,0.,0.),note_omega=(0.,0.,0.),cradle_velocity=(0.,0.,0.),cradle_omega=(0.,0.,0.),u_velocity=0.,rotor_omega=(0.,0.),pitch_radius_m,mu=.6,slip_scale=.01):
 """Positive acquisition rotor turns drive rearward on the straight inner faces.
 Constant belt speed=pitch radius*rotor speed follows the inherited kinematic
 hypothesis; outer-layer strain around wraps is not a calibrated material model.
 """
 nv=a.vec(note_velocity);nw=a.vec(note_omega);cv=a.vec(cradle_velocity);cw=a.vec(cradle_omega);ud=a.scalar(u_velocity);radius=a.scalar(pitch_radius_m);friction=a.scalar(mu);vscale=a.scalar(slip_scale)
 if len(rotor_omega)!=2:raise ValueError('two rotors')
 rotor=[a.scalar(v) for v in rotor_omega]
 if radius<=0 or friction<0 or vscale<=0:raise ValueError('positive radius/scale and nonnegative friction')
 force=[0.,0.,0.];moment=[0.,0.,0.];reaction=[0.,0.,0.];reactionmoment=[0.,0.,0.];torque=[0.,0.];qu=0.;normalwork=0.;heat=0.;sidefriction=[0.,0.]
 for c in prepared['contacts']:
  n=c['normalWorld'];t=c['acquisitionTangentWorld'];index=c['motor'];base=a.add(cv,a.cross(cw,c['cradleLeverWorldM']))
  if c['side']==1:base=a.add(base,a.scale(prepared['cassetteDirectionWorld'],ud))
  body=a.add(nv,a.cross(nw,c['noteLeverWorldM']));beltvelocity=a.add(base,a.scale(t,radius*rotor[index]));diff=a.sub(beltvelocity,body);slip=a.sub(diff,a.scale(n,a.dot(diff,n)));speed=math.hypot(*slip)
  if not math.isfinite(speed):raise a.DomainError('NONFINITE_SLIP_SPEED')
  fn=c['normalN'];normal=a.scale(n,fn)
  factor=0. if speed==0 else friction*fn*math.tanh(speed/vscale)/speed;ff=a.scale(slip,factor);f=a.add(normal,ff);back=a.scale(f,-1.)
  force=a.add(force,f);moment=a.add(moment,a.cross(c['noteLeverWorldM'],f));reaction=a.add(reaction,back);reactionmoment=a.add(reactionmoment,a.cross(c['cradleLeverWorldM'],back));torque[index]-=radius*a.dot(ff,t);heat+=a.dot(ff,slip);normalwork+=a.dot(normal,a.sub(body,base));sidefriction[index]+=a.dot(ff,t)
  if c['side']==1:qu+=a.dot(back,prepared['cassetteDirectionWorld'])
 bodypower=a.dot(force,nv)+a.dot(moment,nw);basepower=a.dot(reaction,cv)+a.dot(reactionmoment,cw);upower=qu*ud;rotorpower=a.dot(torque,rotor);potentialrate=-normalwork
 result=dict(noteForceWorldN=force,noteMomentWorldNm=moment,cradleReactionWorldN=reaction,cradleMomentWorldNm=reactionmoment,cassetteGeneralizedForceN=qu,rotorContactTorqueNm=torque,bodyPowerW=bodypower,cradlePowerW=basepower,cassettePowerW=upower,rotorPowerW=rotorpower,normalPotentialRateW=potentialrate,slipHeatW=heat,workResidualW=bodypower+basepower+upower+rotorpower+potentialrate+heat,sideBeltAxisTractionN=sidefriction,acceptance=False)
 def check(v):
  if isinstance(v,dict):
   for x in v.values():check(x)
  elif isinstance(v,list):
   for x in v:check(x)
  elif isinstance(v,float) and not math.isfinite(v):raise a.DomainError('NONFINITE_WORK_RESULT')
 check(result);return result

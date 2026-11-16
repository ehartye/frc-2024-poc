import numpy as np,math,json,pathlib
alpha=math.atan2((82.9-78.13),18.11);u=np.array([math.cos(alpha),0,math.sin(alpha)]);n=np.array([-math.sin(alpha),0,math.cos(alpha)]);lo=np.array([0,0,78.13*.0254]);length=math.hypot(18.11,82.9-78.13)*.0254;width=41.38*.0254
rows=[]
for h in [.7,.9,1.1]:
 for angle in range(10,81,5):
  a=math.radians(angle);e=np.array([math.cos(a),0,math.sin(a)]);axis=np.array([-math.sin(a),0,math.cos(a)]);ey=np.array([0,1,0]);pts=[]
  for theta in np.arange(64)*2*math.pi/64:
   radial=math.cos(theta)*e+math.sin(theta)*ey
   for phi in np.arange(16)*2*math.pi/16:pts.append((.1524+.0254*math.cos(phi))*radial+.0254*math.sin(phi)*axis)
  q=np.array(pts)
  for speed in np.arange(4,20.1,.5):
   vx=speed*math.cos(a);vz=speed*math.sin(a);zc=(lo[2]+82.9*.0254)/2
   disc=vz*vz-2*9.80665*(zc-h)
   if disc<0:continue
   t=(vz-math.sqrt(disc))/9.80665;d=vx*t-(18.11*.0254)/2
   if d<.15 or d>3:continue
   p=np.array([-d,0,h]);v=np.array([vx,0,vz]);A=-.5*9.80665*n[2];B=v@n;C=(p+q-lo)@n;D=B*B-4*A*C
   if np.any(D<0):continue
   times=(-B+np.sqrt(D))/(2*A)
   if np.any(times<0):continue
   pos=p+q+times[:,None]*v;pos[:,2]-=.5*9.80665*times*times
   along=(pos-lo)@u;lat=pos[:,1];margin=float(min(along.min(),length-along.max(),width/2-np.abs(lat).max()))
   rows.append({'sampledMarginM':margin,'heightM':h,'angleDeg':angle,'speedMps':float(speed),'distanceM':float(d),'centerCrossingS':t})
r={'schema':'frc-a2-release-geometric-study/1','status':'hypothetical fixed-attitude sampled thin-plane screen, not plant or scoring validation','sampling':{'theta':64,'phi':16},'fixedBodyPitchEqualsVelocityElevation':True,'best':sorted(rows,key=lambda x:x['sampledMarginM'],reverse=True)[:10], 'practicalCandidates':sorted([x for x in rows if x['sampledMarginM']>.015 and x['speedMps']<=10 and x['distanceM']>=.7],key=lambda x:(x['speedMps'],-x['sampledMarginM']))[:10],'limitations':['Prescribed speed is a design requirement, not a motor-generated result','Sampling can miss crossings between samples; no continuous certificate','No spin, deformation, aerodynamic forces, complete roof/walls or verified installed registration']}
pathlib.Path('artifacts').mkdir(exist_ok=True);pathlib.Path('artifacts/a2-release-search.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))

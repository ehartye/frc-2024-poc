"""Pure-math, finite planar-face query for the unadopted rounded retention guide."""
import math,json,pathlib,hashlib

def load_contract(root=None):
 root=pathlib.Path(root) if root is not None else pathlib.Path(__file__).resolve().parents[2]
 path=root/'cad/assembly/evidence/rounded-retention-guide.json';r=json.loads(path.read_text())
 for f,digest in r['inputs'].items():
  if hashlib.sha256((root/f).read_bytes()).hexdigest()!=digest:raise ValueError('stale guide source '+f)
 cad=json.loads((root/'cad/assembly/intake_components.json').read_text())
 return {'guide':r['guide'],'majorRadiusMm':cad['note']['majorRadius'],'tubeRadiusMm':cad['note']['tubeRadius'],'receiptSha256':hashlib.sha256(path.read_bytes()).hexdigest(),'apiSha256':hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest(),'sourceInputs':r['inputs'],'nativeConsistencyUnknown':any(not s['booleanWitnessConsistent'] for s in r['samples']),'knownDisagreements':[{'centerRobotMm':s['noteCenterRobotMm'],'pitchDeg':s['pitchDeg']} for s in r['samples'] if not s['booleanWitnessConsistent']]}

def query(center_mm,pitch_rad,contract,*,yaw_rad=0.,roll_rad=0.):
 """mm/radians; normal points from guide toward piece. Does not compute force.
 Only a pitched nominal torus with shallow contact on the intact flat face is supported.
 The full plane-section patch is conservatively enclosed before accepting face geometry.
 """
 if len(center_mm)!=3 or not all(math.isfinite(float(v)) for v in [*center_mm,pitch_rad,yaw_rad,roll_rad]):raise ValueError('finite pose required')
 x,y,z=map(float,center_mm);g=contract['guide'];R=contract['majorRadiusMm'];r=contract['tubeRadiusMm']
 base={'forceLaw':None,'adopted':False,'nativeMaterialStatus':'UNKNOWN_BOOLEAN_WITNESS_DISAGREEMENT' if contract['nativeConsistencyUnknown'] else 'NOT_QUALIFIED','receiptSha256':contract['receiptSha256']}
 def unknown(reason,**kw):return dict(base,status='UNKNOWN',reason=reason,**kw)
 if abs(yaw_rad)>1e-12 or abs(roll_rad)>1e-12 or not 0<=pitch_rad<=math.pi/3:return unknown('unsupported attitude')
 cp=math.cos(pitch_rad);sp=math.sin(pitch_rad);face=g['guideFaceXmm'];depth=x+R*cp+r-face
 if depth<0:return dict(base,status='SEPARATED_FROM_ENTIRE_GUIDE',gapMm=-depth,normalRobot=[-1,0,0])
 if depth>=r or R*cp<=0:return unknown('deep contact or major ring reaches face',planePenetrationMm=depth)
 # All contributing ring centers lie behind the plane in this shallow domain.
 # Enclose their plane-section disks, rather than testing only the support point.
 cmin=max(-1.,1-depth/(R*cp));disk=math.sqrt(max(0.,2*r*depth-depth*depth));hy=R*math.sqrt(max(0.,1-cmin*cmin))+disk
 bounds=[y-hy,z+R*cmin*sp-disk,y+hy,z+R*sp+disk]
 low=g['bottomZmm']+g['longEdgeRadiusMm'];high=g['topZmm']-g['longEdgeRadiusMm'];width=g['halfWidthMm']
 info={'planePenetrationMm':depth,'conservativePlanePatchYZmm':bounds,'normalRobot':[-1,0,0]}
 if bounds[0]<=-width or bounds[2]>=width or bounds[1]<=low or bounds[3]>=high:return unknown('finite face edge or rounded region may participate',**info)
 holes=g['mountHoleHypothesis'];hr=holes['diameterMm']/2
 for hy0 in holes['Ymm']:
  hz=holes['Zmm'];dy=max(bounds[0]-hy0,0,hy0-bounds[2]);dz=max(bounds[1]-hz,0,hz-bounds[3])
  if math.hypot(dy,dz)<=hr:return unknown('contact patch may reach mount hole',**info)
 point=[face,y,z+R*sp];lever=[face-x,0,R*sp]
 known=any(abs(math.degrees(pitch_rad)-v['pitchDeg'])<1e-7 and max(abs(center_mm[j]-v['centerRobotMm'][j]) for j in range(3))<1e-7 for v in contract['knownDisagreements'])
 return dict(base,**info,status='UNKNOWN' if known else 'ANALYTIC_FINITE_FACE_CANDIDATE',reason='matches archived Boolean/witness discrepancy' if known else 'nominal torus/plane support geometry only',guidePointRobotMm=point,leverFromNoteCenterMm=lever,positivePitchMomentArmForInwardNormalMm=R*sp,pointIsPressureCentroid=False,frictionTangentsRobot=[[0,1,0],[0,0,1]])

def wrench_at_support(force_robot_n,lever_mm):
 """Returns force and pitch torque for +pitch about robot -Y; contact location is a hypothesis."""
 if len(force_robot_n)!=3 or len(lever_mm)!=3 or not all(math.isfinite(float(v)) for v in [*force_robot_n,*lever_mm]):raise ValueError('finite wrench')
 fx,fy,fz=force_robot_n;rx,ry,rz=[v/1000 for v in lever_mm]
 return {'forceRobotN':list(force_robot_n),'momentRobotNm':[ry*fz-rz*fy,rz*fx-rx*fz,rx*fy-ry*fx],'positivePitchTorqueNm':rx*fz-rz*fx}

def query_round_xz(center_m,pitch_rad,contract,*,y_m=0.):
 """SI candidate contact, flat face/lower round only; no force or acceptance.
 For front-up pitch, all other major-ring centers are left/below the front center.
 While that front center is below the upper cap, it minimizes distance to this
 left-facing vertical capsule. Finite side edges and mount holes are separately gated.
 """
 if len(center_m)!=2 or not all(math.isfinite(float(v)) for v in [*center_m,pitch_rad,y_m]):raise ValueError('finite XZ pose')
 x,z=[float(v)*1000 for v in center_m];y=y_m*1000;g=contract['guide'];R=contract['majorRadiusMm'];r=contract['tubeRadiusMm'];base={'forceLaw':None,'adopted':False,'nativeMaterialStatus':'UNKNOWN_BOOLEAN_WITNESS_DISAGREEMENT' if contract['nativeConsistencyUnknown'] else 'NOT_QUALIFIED','receiptSha256':contract['receiptSha256']}
 def unknown(reason,**extra):return dict(base,status='UNKNOWN',geometryDomainValid=False,reason=reason,**extra)
 if not 0<=pitch_rad<=math.pi/3:return unknown('unsupported attitude')
 cp=math.cos(pitch_rad);sp=math.sin(pitch_rad);face=g['guideFaceXmm'];rg=g['longEdgeRadiusMm'];low=g['bottomZmm']+rg;high=g['topZmm']-rg;mx=x+R*cp;mz=z+R*sp
 plane_depth=mx+r-face
 if plane_depth<0:return dict(base,status='SEPARATED_FROM_ENTIRE_GUIDE',geometryDomainValid=True,penetrationM=0.,gapM=-plane_depth/1000)
 if plane_depth>=r or mx>=face:return unknown('major ring reaches face or deep contact')
 if mz>=high:return unknown('upper cap may determine minimum')
 axisx=face+rg;axisz=max(low,mz);dx=mx-axisx;dz=mz-axisz;distance=math.hypot(dx,dz)
 if distance<=rg:return unknown('ring enters guide')
 depth=r+rg-distance
 # If all ring centers are left/below M, this infinite-width capsule minimum
 # is a lower bound for the finite solid (holes only remove material).
 if depth<0:return dict(base,status='SEPARATED_FROM_ENTIRE_GUIDE',geometryDomainValid=True,penetrationM=0.,gapM=-depth/1000)
 cmin=max(-1.,1-plane_depth/(R*cp));disk=math.sqrt(max(0.,2*r*plane_depth-plane_depth**2));hy=R*math.sqrt(max(0.,1-cmin*cmin))+disk;bounds=[y-hy,z+R*cmin*sp-disk,y+hy,z+R*sp+disk]
 if bounds[0]<=-g['halfWidthMm'] or bounds[2]>=g['halfWidthMm'] or bounds[3]>=high:return unknown('finite side/top edge possible',conservativePatchYZm=[v/1000 for v in bounds])
 holes=g['mountHoleHypothesis']
 for yy in holes['Ymm']:
  dy=max(bounds[0]-yy,0,yy-bounds[2]);zz=holes['Zmm'];dz0=max(bounds[1]-zz,0,zz-bounds[3])
  if math.hypot(dy,dz0)<=holes['diameterMm']/2:return unknown('mount hole possible')
 nx=dx/distance;nz=dz/distance;point=[(axisx+rg*nx)/1000,(axisz+rg*nz)/1000];lever=[point[0]-center_m[0],point[1]-center_m[1]]
 jac=[-nx,-nz,(nx*R*sp-nz*R*cp)/1000]
 known=any(abs(math.degrees(pitch_rad)-v['pitchDeg'])<1e-7 and max(abs(q-v['centerRobotMm'][j]) for q,j in [(x,0),(y,1),(z,2)])<1e-7 for v in contract['knownDisagreements'])
 return dict(base,status='UNKNOWN' if known else 'ANALYTIC_FINITE_GUIDE_CANDIDATE',geometryDomainValid=not known,reason='archived Boolean/witness disagreement' if known else 'shallow nominal torus versus lower/straight guide capsule',penetrationM=depth/1000,guidePointWorldXZm=point,normalWorldXZ=[nx,nz],tangentWorldXZ=[nz,-nx],leverWorldXZm=lever,penetrationJacobianXZPitch=jac,positivePitchTorquePerNormalN=lever[0]*nz-lever[1]*nx,conservativePatchYZm=[v/1000 for v in bounds],surface='lower rounded edge' if mz<low else 'planar wear face',pointIsPressureCentroid=False)

"""Uninstalled original eccentric cam; no positive latch or servo mounting claimed."""
import math,json,pathlib
import spring_cassette_candidate as s
h=s.h
CONTRACT=pathlib.Path(__file__).with_name('cassette-opening-source-candidate.json')
def evaluate(theta):
 if not math.isfinite(theta) or not 0<=theta<=math.pi:raise ValueError('cam angle domain')
 c=json.loads(CONTRACT.read_text());spring=c['spring'];cam=c['openingCam'];e=cam['eccentricityMm'];u=e*(1-math.cos(theta));compression=max(0,spring['nominalFreeLengthMm']-7-u);force=spring['countPerSide']*spring['nominalRateNPerMm']*compression
 return {'thetaRad':theta,'contactTravelMm':u,'nominalSpringPairForceN':force,'potentialJ':.5*force*compression/1000,'openingTorqueMagnitudeNm':force*e/1000*math.sin(theta),'positiveLatch':False,'contactConstraintNotActualState':True}
def build(theta):
 c=json.loads(CONTRACT.read_text())['openingCam'];r=evaluate(theta);x,y,z=c['shaftCenterMm'];e=c['eccentricityMm'];width=c['discWidthMm']
 cam=h.cyl((x-width/2,y+e*math.cos(theta),z+e*math.sin(theta)),(1,0,0),c['discRadiusMm'],width)
 cam=s.cut(cam,h.cyl((x-width/2-1,y,z),(1,0,0),1.6,width+2))
 follower=h.box(x-8,203-r['contactTravelMm'],94,16,3,12)
 return [('original uninstalled eccentric cam',cam),('original unattached follower coupon',follower)]
if __name__=='__main__':
 out=s.a.ROOT/'artifacts/spring-cassette';out.mkdir(exist_ok=True)
 for label,t in [('open',0),('mid',math.pi/2),('released',math.pi)]:
  w=h.STEPControl_Writer()
  for n,shape in build(t):
   if not h.BRepCheck_Analyzer(shape).IsValid():raise ValueError(n)
   w.Transfer(shape,h.STEPControl_AsIs)
  w.Write(str(out/('uninstalled-cam-'+label+'.step')))
 print(json.dumps([evaluate(t) for t in [0,math.pi/2,math.pi]],indent=2))

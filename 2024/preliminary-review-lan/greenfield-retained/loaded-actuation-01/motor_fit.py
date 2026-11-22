"""Locate a drawing-matched NEO 550 in the preliminary slide package."""
from motor_source import *
from layout import box,cyl,cut,fuse,X,G,write_step
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
def vol(s):
    a=GProp_GProps();BRepGProp.VolumeProperties_s(s,a);return a.Mass()
fp=root/'guided-hoist-01/service-locked-robot.step';R={name(n):s for n,s in load_solids(fp)[0].items()}
def shaft(x,y,z,r,L):return cyl((x,y,z),X,r,L)
def near(s,h):
    B=np.array(bounds(s));C=np.array(bounds(h));return np.all(B[3:]+.0001>=C[:3]) and np.all(C[3:]+.0001>=B[:3])
def move(dx=0,dy=0,dz=0):return mat(0,dx,dy,dz)
def transform(theta):
    co=math.cos(theta);si=math.sin(theta)
    return np.array([[1.,0.,0.,244.5],[0.,co,-si,275],[0.,si,co,428],[0,0,0,1.]])
def overlap(a,b):return common(a,b) if near(a,b) else 0.
fixture=root/'endgame-route-01/forward-slide.step'
study={name(k):v for k,v in load_solids(fixture)[0].items()}
old=study['Forward slide motor yoke'];shaftEnd=study['Slide screw extension'];stationary={k:v for k,v in R.items() if k!='Slide drive motor'}
package=shaft(233.5,275,428,9,18)
# Two clamp lands separated by the untouched annular shoulder. This is an occupied hub blank; clamp slots/bolts remain to size.
hub=cut(cut(package,shaft(233.4,275,428,4.025,10)),shaft(244.05,275,428,1.605,7.6))
allResults=[]
for theta in [0,math.pi/4,math.pi/2,math.pi]:
    motor=place(bodies[0],transform(theta));yoke=cut(old,shaft(252.8,275,428,6.55,3.4))
    for sign in [-1,1]:
        y=275+12.5*math.cos(theta)*sign;z=428+12.5*math.sin(theta)*sign
        yoke=cut(yoke,shaft(252.8,y,z,1.7,3.4))
    parts={'Screw spindle extension':shaftEnd,'Purchased slide motor':motor,'Bored and drilled support':yoke,'Stepped hub stock':hub};bad=[]
    for n,v in parts.items():
        for k,h in stationary.items():
            if (occ:=overlap(v,h))>.01:bad.append([n,k,occ])
    for n,k in [('Purchased slide motor','Bored and drilled support'),('Purchased slide motor','Stepped hub stock'),('Bored and drilled support','Stepped hub stock')]:
        if (occ:=overlap(parts[n],parts[k]))>.01:bad.append([n,k,occ])
    driver=[]
    for sign in [-1,1]:
        y=275+12.5*math.cos(theta)*sign;z=428+12.5*math.sin(theta)*sign
        tool=shaft(177.8,y,z,2.25,75.)
        for k,h in (stationary|{'revised yoke':yoke,'new coupling':hub}).items():
            if (occ:=overlap(tool,h))>.01:driver.append([sign,k,occ])
    area=overlap(place(motor,move(-.02)),yoke)/.02
    test=dict(clockDeg=round(math.degrees(theta),2),originalPilotMm3=overlap(motor,old),collisions=bad,driver=driver,seatAreaEstimateMm2=area,placedMotorBoundsMm=bounds(motor))
    allResults.append(test)
    if not bad and not driver and not (p/'slide-motor-fit.step').exists():
        G.clear();G.update(parts);write_step(p/'slide-motor-fit.step')
d=dict(sourceMotor=hashlib.sha256(source.read_bytes()).hexdigest(),frontBracketStudy=hashlib.sha256(fixture.read_bytes()).hexdigest(),replaced=['Slide drive motor'],parent=hashlib.sha256(fp.read_bytes()).hexdigest(),trials=allResults,scope='Exact purchased motor and its terminals. Estimated contact area from 0.02 mm normal overdrive. Straight driver test assumes rear installation with the screw spindle in place. Two separate shaft registers still require a split clamp, fasteners, rated screw/nut and cable egress. Rotation/chain and disconnected service must be rechecked.')
(p/'slide-motor-fit.json').write_text(json.dumps(d,indent=2))
for r in allResults:print(r['clockDeg'],r['originalPilotMm3'],r['collisions'],r['driver'])


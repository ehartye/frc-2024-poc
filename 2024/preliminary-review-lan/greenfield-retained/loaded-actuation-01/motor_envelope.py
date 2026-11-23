"""Fast physical envelopes for the purchased can, connector region, shaft, pilot and service tool."""
from pathlib import Path
import json,sys,math,numpy as np,hashlib
p=Path(__file__).resolve().parent;root=p.parent
sys.path.insert(0,str(root/'assembly-recheck-01'));from sweep import load_solids,name,place,bounds,common,mat
from layout import box,cyl,cut,fuse,X
def read(f):return json.loads(f.read_bytes())
R={name(k):s for k,s in load_solids(root/'guided-hoist-01/service-locked-robot.step')[0].items()}
results=read(p/'purchased-motor.json');assert results['source'].startswith('4fdcdd97')
stock={k:s for k,s in R.items() if k!='Slide drive motor'}
def peg(x,y,z,r,h):return cyl((x,y,z),X,r,h)
def near(s,h):
    a,b=np.array(bounds(s)),np.array(bounds(h));return np.all(a[3:]+.0001>=b[:3]) and np.all(b[3:]+.0001>=a[:3])
def measure(a,b):return common(a,b) if near(a,b) else 0.
# Existing forward crosshead study dimensions, deliberately kept separate from the current saved motor.
cy=275;cz=428
oldFront=cut(box(253,255,411,3,40,34),peg(252,cy,cz,5,5))
back=cut(box(230,255,411,3,40,34),peg(229,cy,cz,5,6))
web=fuse(fuse(back,box(232.9,255,411,20.2,3,34)),box(232.9,292,411,20.2,3,34))
pilot=peg(251.75,cy,cz,6.5,4.25);stem=peg(244.6,cy,cz,1.585,6.9)
screw=peg(230,cy,cz,4,12.8);hub=cut(cut(peg(233.5,cy,cz,9,18),peg(233.4,cy,cz,4.025,10)),peg(244.05,cy,cz,1.605,7.6))
cases=[]
for sign in ('forward','rear'):
 transform=np.eye(4) if sign=='forward' else np.array([[-1.,0,0,181.],[0,-1.,0,550.],[0,0,1.,0.],[0,0,0,1.]])
 def tr(s):return place(s,transform)
 for clock in (0,45):
    cf=math.cos(math.radians(clock));sf=math.sin(math.radians(clock))
    front=cut(oldFront,peg(252.8,cy,cz,6.55,3.4))
    tools=[]
    for side in [-1,1]:
        y,z=cy+12.5*cf*side,cz+12.5*sf*side
        front=cut(front,peg(252.8,y,z,1.7,3.4))
        tools.append(peg(177.8,y,z,2.25,75))
    support=fuse(web,front)
    parts={'can reservation':peg(256,cy,cz,17.5,44.5),'terminals maximum radial reservation':peg(282,cy,cz,math.hypot(25.5,25.5),18.5),'pilot':pilot,'output shaft':stem,'hub':hub,'screw spindle':screw,'support':support}
    bad=[];drivers=[]
    for n,s in parts.items():
        for k,h in stock.items():
            if (v:=measure(tr(s),h))>.01:bad.append([n,k,v])
    for i,t in enumerate(tools):
        for k,h in stock.items():
            if (v:=measure(tr(t),h))>.01:drivers.append([i,k,v])
        for k,s in [('back wall',back),('side webs',web),('hub',hub)]:
            if (v:=measure(t,s))>.01:drivers.append([i,k,v])
    cases.append(dict(orientation=sign,clockDeg=clock,originalPilotCollision=measure(oldFront,pilot),pilotAfter=measure(support,pilot),shaftHub=measure(stem,hub),spindleHub=measure(screw,hub),obstructions=bad,toolObstructions=drivers))
    (p/'motor-envelope.json').write_text(json.dumps(dict(source=results['source'],root=hashlib.sha256((root/'guided-hoist-01/service-locked-robot.step').read_bytes()).hexdigest(),cases=cases,scope='Manufacturer shaft, pilot and case are represented by conservative occupied cylinders; the terminal disk is a keepout, and any interference there requires actual connector narrowing. The 75 mm straight driver starts under the proposed motor plate. Retain a separate incoming support attachment and cable service check.'),indent=2))
    print(sign,clock,round(cases[-1]['originalPilotCollision'],2),bad,drivers,flush=True)


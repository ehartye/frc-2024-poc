"""Package a common spring-return gate yoke, backdrivable pinion and stationary drive mast on the cassette."""
from pathlib import Path
import sys,hashlib,json,numpy as np
p=Path(__file__).resolve().parent;root=p.parent;sys.path.insert(0,str(root/'assembly-recheck-01'))
from sweep import load_solids,name,park,ip
from layout import box,cyl,ann,prism,cut,fuse,common,place,distance,mat,write_step,viewmesh,G,bounds,Y,Z
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
def V(s):g=GProp_GProps();BRepGProp.VolumeProperties_s(s,g);return g.Mass()
base=root/'forward-drive-02/forward-drive.step';s0,_=load_solids(base);raw={name(n):s for n,s in s0.items()}
source=root/'chain-entry-01/hook-and-guided-keepers.step';local,_=load_solids(source)
G.clear();G.update(raw);parts={};rig={};removed=[];deltaMass=0;seats={}
def install(n,s,role,rho):
    global deltaMass
    assert BRepCheck_Analyzer(s).IsValid()
    parts[n]=s;rig[n]=role;G[n]=place(s,park);deltaMass+=V(s)*rho
# Both broached keeper tongues and rack are rigid members of one translating yoke.
bridge=box(201,-104.88,273,5,209.76,5);rack=box(197,-4,270,4,8,60)
link=box(198,-4,273,8,8,5);bridge=fuse(fuse(bridge,link),rack)
for side in ['left','right']:
    n='Chain hook · '+side;gate='Hook keeper · '+side
    for t in [n,gate]:deltaMass-=V(G[t])*7.85e-6;removed.append(t);G.pop(t)
    install(n,local[side+' hook'],'arm',7.85e-6);bridge=fuse(bridge,local[side+' gate'])
install('Common keeper yoke and sliding rack',bridge,'keeper',7.85e-6)
# Single mast carries the pinion reaction to the upper yoke of the cassette. Flange lands on the existing crosspiece.
foot=box(126,9,192,62,12,7);post=box(126,9,199,10,12,122);bearing=box(126,9,315,74,12,16)
mast=fuse(fuse(foot,post),bearing);axleBore=cyl((189,8,323),Y,3.2,14);mast=cut(mast,axleBore)
install('Keeper drive mast',mast,'arm',2.70e-6)
install('Keeper pinion output',cyl((189,-4,323),Y,3,30),'arm',7.85e-6)
install('Keeper return pinion',ann((189,-4,323),Y,8,3,8),'arm',7.85e-6)
# Compact coiled torsion spring / gearmotor envelope. Reaction connects at the mast face.
install('Keeper motor and return spring',cut(cyl((189,21,323),Y,14,38),cyl((189,20,323),Y,3.25,7)),'arm',0.)
for n in ['Keeper drive mast','Common keeper yoke and sliding rack']:
    seats[n]=V(parts[n])
cross=place(raw['Shared lift crosspiece'],ip)
seats['mast lands mm2']=common(cross,place(mast,mat(0,0,0,-.1)))/.1
seats['rack face overlap closed and open mm']=[min(330+v-323,323-270-v) for v in [0,44]]
seats['spring at pitch radius Nm']=[(3.6+360*x)*.008 for x in [0.,.044]]
seats['open peak commanded Nm']=45*.008
write_step(p/'keeper-drive.step')
pkg=json.loads((root/'forward-drive-02/package.json').read_bytes());pkg['removed']+=removed;pkg['added'].update({n:0 for n in parts})
step=hashlib.sha256((p/'keeper-drive.step').read_bytes()).hexdigest()
pkg['layout']=step;(p/'package.json').write_text(json.dumps(pkg,indent=2))
r=dict(parent=hashlib.sha256(base.read_bytes()).hexdigest(),hookSource=hashlib.sha256(source.read_bytes()).hexdigest(),step=step,removed=removed,rig=rig,seats=seats,
netSolidMassKg=deltaMass,extraIncludingActuatorKg=[deltaMass+.11+.018,deltaMass+.23+.065],scope='Spring and backdrivable motor are an allocation; opening torque, friction, lost motion, shaft and positive stop strength require selected components. Pinion and rack are pitch envelopes. Gate travel 0–44 mm.')
(p/'result.json').write_text(json.dumps(r,indent=2));(p/'mesh.json').write_text(json.dumps({n:viewmesh(G[n]) for n in parts},separators=(',',':')))
print(seats,r['extraIncludingActuatorKg'])


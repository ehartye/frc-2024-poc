"""Relocate the two launch motors to the upper faces of the forward pulleys, preserving all tire paths."""
from pathlib import Path
import sys,json,hashlib,numpy as np
p=Path(__file__).resolve().parent;root=p.parent;sys.path.insert(0,str(root/'assembly-recheck-01'))
from sweep import load_solids,name,park,ip
from layout import box,cyl,cut,fuse,G,bounds,common,distance,mat,Y,Z,place,write_step,viewmesh
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
def volume(s):
    a=GProp_GProps();BRepGProp.VolumeProperties_s(s,a);return a.Mass()
prev=root/'frame-pocket-01';base=prev/'frame-and-feeder.step';f=hashlib.sha256(base.read_bytes()).hexdigest()
assert f==json.loads((prev/'result.json').read_bytes())['step']
G.clear();G.update({name(n):s for n,s in load_solids(base)[0].items()})
modified=[];masses={};seats={};cal={}
def setstock(n,s):
    v=volume(G[n]) if n in G else 0.;G[n]=place(s,park);modified.append(n);masses[n]=volume(s)-v
for sign,side in ((1,'left'),(-1,'right')):
    y=195.8*sign
    motor='Launch Belt motor · '+side;shelf='Launch Belt motor shelf · '+side;shaft='Launch Belt drive shaft · '+side
    axle='Launch idler axle · '+side;old=G[motor]
    setstock(motor,cyl((600,y,75),Z,18,33))
    # Drive the forward pulley from the bridge. The rear pulley becomes a free idler on the former motor shelf.
    setstock(shaft,cyl((600,y,19),Z,5,56))
    setstock(axle,cyl((407,y,-10),Z,5,74))
    ys=(220.4 if sign>0 else -225.4)
    tower=box(585,ys,19,21,5,49)
    top=box(580,(177 if sign>0 else -225.4),68,40,48.4,7)
    mount=cut(fuse(tower,top),cyl((600,y,67),Z,5.4,9))
    key='Forward launcher motor bridge · '+side;setstock(key,mount)
    bar=place(G['Runner crossbar 599'],ip);seats[side]=dict(crossbarTopMm2=common(bar,place(mount,mat(0,0,0,-.1)))/.1,motorSeatMm2=common(mount,place(cyl((600,y,75),Z,18,33),mat(0,0,0,-.1)))/.1)
    bumper=G['Blue bumper set'];deploy=mat(-49.7445989264,169.362589,0,501)
    cal[side]=dict(before=common(place(old,deploy@ip),bumper),after=common(place(G[motor],deploy@ip),bumper))
write_step(p/'forward-drive.step')
pkg=json.loads((root/'split-cycle-01/package.json').read_bytes())
pkg['added'].update({n:0 for n in modified if n not in pkg['added']});pkg['categories'].update({n:'pulley support' for n in modified if n not in pkg['categories']})
step=hashlib.sha256((p/'forward-drive.step').read_bytes()).hexdigest();pkg['layout']=step
(p/'package.json').write_text(json.dumps(pkg,indent=2))
r=dict(parent=f,step=step,modified=modified,seats=seats,bumperAtPickup=cal,netVolumeMm3=masses)
(p/'result.json').write_text(json.dumps(r,indent=2));(p/'mesh.json').write_text(json.dumps({n:viewmesh(G[n]) for n in modified},separators=(',',':')))
print(seats,cal)


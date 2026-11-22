"""Bench-mounted motor cartridge. Dimensions are millimetres in the registered robot frame."""
from pathlib import Path
import hashlib,json,sys,itertools,numpy as np,math
p=Path(__file__).resolve().parent;root=p.parent
sys.path.insert(0,str(root/'assembly-recheck-01'))
from sweep import load_solids,name,bounds,common,place,mat
from layout import box,cyl,cut,fuse,write_step,G,X
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
def massvolume(s):
    q=GProp_GProps();BRepGProp.VolumeProperties_s(s,q);return q.Mass()
def ax(x,y,z,r,l):return cyl((x,y,z),X,r,l)
def az(x,y,z,r,l):return cyl((x,y,z),(0,0,1),r,l)
def digest(f):return hashlib.sha256(f.read_bytes()).hexdigest()
fp=root/'guided-hoist-01/service-locked-robot.step'
src=Path('C:/Users/ehart/Documents/FRC Native/plugin-qa/toe-actuator-20261121/REV-21-1651.STEP')
assert digest(src).startswith('4fdcdd97') and digest(fp).startswith('a29a146d')
stock={name(k):v for k,v in load_solids(fp)[0].items()};assert len(stock)==232
del stock['Slide drive motor']
bb={k:bounds(v) for k,v in stock.items()}
def overlap(a,b):return all(a[i+3]>=b[i]-.0001 and b[i+3]>=a[i]-.0001 for i in range(3))
checks={};bad=[]
def screen(label,subjects,obstacles):
    hits=[];n=0
    for k,s in subjects.items():
        b=bounds(s)
        for j,h in obstacles.items():
            if not overlap(b,bb[j] if j in bb else bounds(h)):continue
            n+=1;v=common(s,h)
            if v>.001:hits.append([k,j,v])
    checks[label]=dict(subjects=len(subjects),obstacles=len(obstacles),narrow=n,positive=hits)
    return hits
# The two rear pads attach to the existing bearing block. Holes are machining allowances.
block='Slide bearing mount 220'
rear=cut(box(230,248,416,3,54,23),ax(229,275,428,5,5))
host=stock[block];mounts={};mountTools={};pilotVolumes=[]
for i,(y,z) in enumerate([(259,421),(265,437.5)]):
    rear=cut(rear,ax(229,y,z,2.2,5));before=massvolume(host);host=cut(host,ax(222,y,z,1.65,8.1));removed=before-massvolume(host);assert abs(removed-math.pi*1.65**2*8)<.001;pilotVolumes.append(removed)
    mounts[str(i)]=fuse(ax(224,y,z,2,9),ax(233,y,z,3.5,4))
    mountTools[str(i)]=ax(237,y,z,2.5,75)
stock[block]=host
frame=rear
cap=cut(box(253,248,411,3,54,34),ax(252,275,428,6.55,5))
for y in [262.5,287.5]:cap=cut(cap,ax(252,y,428,1.7,5))
ret={};tools={};tabs={};addons={}
for i,y in enumerate([253,297]):
    rail=box(233,y-5,416,20,10,23)
    rail=cut(rail,az(247,y,430.8,1.65,8.3));frame=fuse(frame,rail)
    tab=cut(box(242,y-5,439,11,10,3),az(247,y,438.9,2.2,3.2));cap=fuse(cap,tab)
    ret[str(i)]=fuse(az(247,y,433,2,9),az(247,y,442,3.5,4))
    tools[str(i)]=az(247,y,446,2.5,75)
capBolts={str(i):fuse(ax(250,y,428,2.75,3),ax(253,y,428,1.5,6.5)) for i,y in enumerate([262.5,287.5])}
def motorRes(draw):
    ext=lambda x,y,z,r,l:ax(x,y,z,r,l+draw)
    return {'shaft':ext(244.6,275,428,1.585,6.9),'pilot':ext(251.75,275,428,6.5,4.25),
        'can':ext(256,275,428,17.5,44.5),'terminal keepout':ext(282,275,428,math.hypot(25.5,25.5),18.5)}
def shift(s,dx):return place(s,mat(0,dx,0,0))
spindle=ax(230,275,428,4,12.8)
hub=cut(cut(ax(233.5,275,428,9,18),ax(233.4,275,428,4.025,10)),ax(244.05,275,428,1.605,7.6))
screen('installed support',{**{'receiver':frame,'cap':cap},**motorRes(0)},stock)
surround=stock|{'receiver':frame,'spindle':spindle,'hub':hub}|{'mount'+k:v for k,v in mounts.items()}
screen('75 mm attachment drivers before motor installation',mountTools,stock|{'receiver':frame})
screen('75 mm cartridge drivers with motor seated',tools,surround|{'cap':cap}|motorRes(0))
# Conservative solid sweeps bound cap, tabs and attachment heads. Sweep has internal shaft passage.
draw=45
capSweep=cut(box(253,248,411,3+draw,54,34),ax(252,275,428,6.55,draw+5))
sweeps={'cap':capSweep,**motorRes(draw)}
for i,y in enumerate([253,297]):sweeps['tab'+str(i)]=box(242,y-5,439,11+draw,10,3)
for i,y in enumerate([262.5,287.5]):sweeps['M3'+str(i)]=ax(250,y,428,2.75,9.5+draw)
screen('axial cartridge withdrawal, clamp loosened and retainer screws removed',sweeps,surround)
screen('withdrawal with retainers still installed',{'tab'+str(i):shift(cap,4) for i in [0]},ret)
screen('bench screws attempted in mounted position',{str(i):ax(175,y,428,2.25,75) for i,y in enumerate([262.5,287.5])},surround)
shaft=next(iter(load_solids(src)[0].values()))
parts={'Stationary rail receiver':frame,'Removable cap and ears':cap,'Machined bearing block':host,'Purchased slide motor':shift(place(shaft,mat(0,0,275,428)),244.5),'Trial coupling blank':hub,'Spindle':spindle}
for prefix,group in [('M3 on cap',capBolts),('M4 cap retention',ret),('M4 receiver attachment',mounts)]:
    parts.update({prefix+' '+k:v for k,v in group.items()})
G.clear();G.update(parts);out=p/'slide-cartridge.step';write_step(out)
receipt=dict(parent=digest(fp),motor=digest(src),fixture=digest(out),checks=checks,fullPilotCutVolumesMm3=pilotVolumes,capAndReceiverMassKg=2.7e-6*(massvolume(frame)+massvolume(cap)),extraHardwareKg=7.85e-6*sum(massvolume(v) for v in list(mounts.values())+list(ret.values())+list(capBolts.values())),sequence=['On the bench install motor on cap using its two M3 screws','Attach receiver to prepared block before inserting hub','Place spindle and coupling, insert motor cartridge axially; tighten hub','Install two top M4 retainers; reverse sequence for service'],open=['Coupling clamp and its driver, purchased threaded engagement, wiring bend and withdrawal beyond first 45 mm','Remaining operational poses and load under bearing/rail deflection'])
(p/'fit.json').write_text(json.dumps(receipt,indent=2));print({k:v['positive'] for k,v in checks.items()})


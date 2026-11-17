"""Additive raised lower-carrier and source-dimensioned track-bearing candidate."""
import json,math,hashlib,pathlib
import build_connected_candidate as a
from bearing_seat_candidate import flip
from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse
ic=a.assembly.intake_components;h=a.h

def pins():
 p=a.input_pins()
 for f in ['cad/assembly/seated_bearing_assembly.py','cad/assembly/bearing_seat_candidate.py','cad/assembly/evidence/bearing-seat-candidate.json']:p[f]=hashlib.sha256((a.ROOT/f).read_bytes()).hexdigest()
 return p

def replacements(c,deg,zc):
 p=c['pinch'];r=p['teeth']*p['pitch']/(2*math.pi);center=(p['beltTeethHypothesis']-p['teeth'])*p['pitch']/2;track=p['innerFaceYAbs']+r+1
 rear=p['beltCenterX']-center/2;front=p['beltCenterX']+center/2;result={}
 def put(name,shape):
  if not h.BRepCheck_Analyzer(shape).IsValid():raise ValueError(name)
  result['intake candidate '+name]=h.pitch(shape,math.radians(deg),[c['stageCenterX'],0,zc])
 body=ic.ring_z(0,0,-3.96875,14.2875,6.875,6.35);flange=ic.ring_z(0,0,2.38125,15.5575,6.875,1.5875);bearing=BRepAlgoAPI_Fuse(body,flange).Shape()
 for side in [-1,1]:
  y=side*track;weld=[]
  for x in [rear,front]:
   ident=str((side,x))
   for zz in [p['lowerBearingZ'],p['upperBearingZ']]:
    lower=zz==p['lowerBearingZ'];shift=-.38125 if lower else -3.96875
    put('bearing representation '+ident+str(zz),h.move(flip(bearing) if lower else bearing,x,y,zz+shift))
    platez=zz+(2 if lower else 0)
    plate=ic.ring_z(x,y,platez-4.7625,17,14.3,3.175)
    if x<0:
     wx=max(x,-95);arm=h.box(x-10,min(side*185.3,side*205.3),platez-4.7625,wx-(x-10),20,3.175)
    else:wx=x;arm=h.box(x-10,min(side*185.3,side*207),platez-4.7625,20,21.7,3.175)
    plate=ic.cut(BRepAlgoAPI_Fuse(plate,arm).Shape(),h.cyl((x,y,platez-6),(0,0,1),14.3,7));weld.append(plate)
    bottom=min(-12.7,platez-4.7625);top=max(12.7,platez-1.5875);weld.append(h.box(wx-10,side*c['cradleSideYAbs']-3.175,bottom,20,6.35,top-bottom))
  start=min(-40,rear-20);stock=h.box(start,side*c['cradleSideYAbs']-3.175,-12.7,front+20-start,6.35,25.4)
  for shape in weld:stock=BRepAlgoAPI_Fuse(stock,shape).Shape()
  for x in [rear,front]:stock=ic.cut(stock,h.cyl((x,y,-30),(0,0,1),14.3,70))
  stock=ic.cut(stock,h.cyl((0,side*c['cradleSideYAbs']-10,0),(0,1,0),7.1,20));put('original cradle side weldment '+str(side),stock)
 if len(result)!=10:raise ValueError('replacement inventory')
 return result

def apply(parts,c,deg,zc):
 repl=replacements(c,deg,zc);names={n for n,s,k in parts}
 if not set(repl)<=names:raise ValueError('missing part')
 return [(n,repl.get(n,s),k) for n,s,k in parts],set(repl)

def build_pose(c,deg,zc,base=None):
 base=base if base is not None else [(n,s,k) for n,s,k in a.assembly.build('floor') if not n.startswith('intake candidate')]
 parts=base+ic.build(c=c,pose=(deg,zc));parts,lo=a.upper.lower.apply(parts,c);parts,up=a.upper.apply(parts,zc,c);parts,pad=a.pads.apply(parts,zc,c)
 return apply(parts,c,deg,zc)

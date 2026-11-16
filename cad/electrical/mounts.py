"""Original source-flat aft supports. Shaft proxies are not complete selected fasteners."""
from a2_primitives import box,cyl,tube
from fasteners import stack
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut,BRepAlgoAPI_Fuse

def drill(shape,centers,z,height):
 for x,y in centers:shape=BRepAlgoAPI_Cut(shape,cyl((x,y,z-1),(0,0,1),2.75,height+2)).Shape()
 return shape

def build(d):
 p=d['mountingPlan'];z=p['railTopMm'];bottom=z+4;top=bottom+25.4;out=[]
 for x in p['supportStationsX']:
  beam=drill(tube(x-12.7,-244.475,bottom,25.4,488.95,25.4,3.175,'y'),[(x,y) for y in [-228.6,-80,80,228.6]],bottom,25.4)
  out.append((f'aft-electrical-beam-{x}',beam,'mount'))
  for sign in [-1,1]:
   y=sign*260.35;ty=sign*228.6
   bar=box(x-20,248.5 if sign>0 else -270,z,40,21.5,4)
   tongue=box(x-10,220 if sign>0 else -248.5,z,20,28.5,4)
   plate=drill(BRepAlgoAPI_Fuse(bar,tongue).Shape(),[(x,y),(x,ty)],z,4)
   out.append((f'aft-adapter-{x}-{sign}',plate,'mount'))
   sleeve=BRepAlgoAPI_Cut(cyl((x,ty,bottom+3.175),(0,0,1),4.7625,19.05),cyl((x,ty,bottom+2),(0,0,1),2.75,21.5)).Shape()
   out.append((f'aft-crush-sleeve-{x}-{sign}',sleeve,'mount'))
   out+=stack(f'aft-bolt-rail-{x}-{sign}',x,y,z+4,p['railBottomMm'],19.05)
   out+=stack(f'aft-bolt-tube-{x}-{sign}',x,ty,top,z,44.45,True)
  for yy in [-80,80]:
   sleeve=BRepAlgoAPI_Cut(cyl((x,yy,bottom+3.175),(0,0,1),4.7625,19.05),cyl((x,yy,bottom+2),(0,0,1),2.75,21.5)).Shape()
   out.append((f'baseplate-crush-sleeve-{x}-{yy}',sleeve,'mount'))
   out+=stack(f'baseplate-bolt-{x}-{yy}',x,yy,top+3,bottom,44.45,True)
 # Stand-off posts outside the PDH and roboRIO footprints carry removable upper decks.
 for x,y in p['postCentersXY']:
  for a,b,label in [(168.115,277.615,'lower'),(280.615,360.115,'upper')]:
   post=BRepAlgoAPI_Cut(cyl((x,y,a),(0,0,1),3.5,b-a),cyl((x,y,a-1),(0,0,1),2.75,b-a+2)).Shape()
   out.append((f'stack-post-{label}-{x}-{y}',post,'mount'))
 # Bolted two-piece clamp: removable top bar, feet and upper attachment flanges.
 bar=drill(box(-215,-308,335.993,30,136,3),[(-200,y) for y in [-298,-182]],335.993,3)
 out.append(('battery-top-strap',bar,'mount'))
 out.append(('battery-insulating-pad',box(-215,-277,334.993,30,74,1),'insulation-assumption'))
 for y,lo in [(-286.5,-308),(-193.5,-195)]:
  foot=drill(box(-215,lo,165.115,30,23,3),[(-200,-298 if y< -240 else -182)],165.115,3)
  topflange=drill(box(-215,lo,332.993,30,23,3),[(-200,-298 if y< -240 else -182)],332.993,3)
  leg=box(-215,y-1.5,168.115,30,3,164.878)
  out.append((f'battery-strap-leg-{y}',BRepAlgoAPI_Fuse(BRepAlgoAPI_Fuse(foot,leg).Shape(),topflange).Shape(),'mount'))
  yy=-298 if y< -240 else -182
  out+=stack(f'battery-foot-bolt-{yy}',-200,yy,168.115,162.115,15.875)
  out+=stack(f'battery-top-bolt-{yy}',-200,yy,338.993,332.993,15.875)
 for x,y in tray_holes():out+=stack(f'battery-tray-bolt-{x}-{y}',x,y,168.115,162.115,15.875)
 for x,y in pdh_holes():out+=stack(f'pdh-bolt-{x}-{y}',x,y,174.465,162.115,25.4)
 out.append(('radio-power-deck-tab',box(-77.5,200,165.115,97.5,60,3),'mount'))
 return out

def holes_for(c,d):
 if c['id']=='controller-deck':return [[x,y] for x in d['mountingPlan']['supportStationsX'] for y in [-80,80]]+[[-200,y] for y in [-298,-182]]+tray_holes()+pdh_holes()
 if c['id']=='battery-tray':return tray_holes()
 if c['id']=='pdh-deck':return d['mountingPlan']['postCentersXY']+pdh_holes()
 if c['id'] in ('pdh-deck','rio-removable-deck','radio-removable-deck'):return d['mountingPlan']['postCentersXY']
 return []

def pdh_holes():return [[x,y] for x in [-308.8,-92.9] for y in [179.2,280.8]]

def tray_holes():return [[-275,-285],[-220,-195],[-180,-285],[-180,-195]]

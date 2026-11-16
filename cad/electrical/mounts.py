"""Original source-flat aft supports. Shaft proxies are not complete selected fasteners."""
from a2_primitives import box,cyl,tube
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut,BRepAlgoAPI_Fuse

def drill(shape,centers,z,height):
 for x,y in centers:shape=BRepAlgoAPI_Cut(shape,cyl((x,y,z-1),(0,0,1),2.75,height+2)).Shape()
 return shape

def build(d):
 p=d['mountingPlan'];z=p['railTopMm'];bottom=z+4;top=bottom+25.4;out=[]
 for x in p['supportStationsX']:
  beam=drill(tube(x-12.7,-244.475,bottom,25.4,488.95,25.4,3.175,'y'),[(x,y) for y in [-228.6,228.6]],bottom,25.4)
  out.append((f'aft-electrical-beam-{x}',beam,'mount'))
  for sign in [-1,1]:
   y=sign*260.35;ty=sign*228.6
   bar=box(x-20,248.5 if sign>0 else -270,z,40,21.5,4)
   tongue=box(x-10,220 if sign>0 else -248.5,z,20,28.5,4)
   plate=drill(BRepAlgoAPI_Fuse(bar,tongue).Shape(),[(x,y),(x,ty)],z,4)
   out.append((f'aft-adapter-{x}-{sign}',plate,'mount'))
   sleeve=BRepAlgoAPI_Cut(cyl((x,ty,bottom+3.175),(0,0,1),4.7625,19.05),cyl((x,ty,bottom+2),(0,0,1),2.75,21.5)).Shape()
   out.append((f'aft-crush-sleeve-{x}-{sign}',sleeve,'mount'))
   for label,yy,lo,hi in [('rail',y,z-10,z+4),('tube',ty,z-5,top+3)]:out.append((f'aft-bolt-{label}-{x}-{sign}',cyl((x,yy,lo),(0,0,1),2.5,hi-lo),'fastener-candidate'))
 # Stand-off posts outside the PDH and roboRIO footprints carry removable upper decks.
 for x,y in p['postCentersXY']:
  for a,b,label in [(168.115,277.615,'lower'),(280.615,360.115,'upper')]:
   post=BRepAlgoAPI_Cut(cyl((x,y,a),(0,0,1),3.5,b-a),cyl((x,y,a-1),(0,0,1),2.75,b-a+2)).Shape()
   out.append((f'stack-post-{label}-{x}-{y}',post,'mount'))
 out.append(('battery-top-strap',box(-215,-288,335.993,30,96,3),'mount'))
 for y in [-286.5,-193.5]:out.append((f'battery-strap-leg-{y}',box(-215,y-1.5,168.115,30,3,167.878),'mount'))
 out.append(('radio-power-deck-tab',box(-77.5,200,165.115,97.5,60,3),'mount'))
 return out

def holes_for(c,d):
 if c['id']=='controller-deck':return [[x,y] for x in d['mountingPlan']['supportStationsX'] for y in [-228.6,228.6]]
 if c['id'] in ('pdh-deck','rio-removable-deck','radio-removable-deck'):return d['mountingPlan']['postCentersXY']
 return []

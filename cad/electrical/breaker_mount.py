"""Original pedestal and selected hardware for source-pinned historical CB185."""
import json,pathlib
from a2_primitives import box,cyl
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut,BRepAlgoAPI_Fuse
from fasteners import stack

def evidence():return json.loads((pathlib.Path(__file__).parent/'breaker-mount-evidence.json').read_text())
def base_holes():return [[x,y] for x in [-306,-244] for y in [-180,-158]]
def build():
 e=evidence();z=e['baseZ']-3;plate=box(-320,-201,z,90,52,3)
 for x,y in e['worldHoleCentersXY']:plate=BRepAlgoAPI_Cut(plate,cyl((x,y,z-1),(0,0,1),2.75,5)).Shape()
 for x in [-320,-253]:
  foot=box(x,-201,165.115,23,52,3)
  for hx,hy in base_holes():foot=BRepAlgoAPI_Cut(foot,cyl((hx,hy,164),(0,0,1),2.75,5)).Shape()
  web=box(-320 if x==-320 else -233,-201,168.115,3,52,z-168.115)
  plate=BRepAlgoAPI_Fuse(plate,BRepAlgoAPI_Fuse(foot,web).Shape()).Shape()
 out=[('breaker-pedestal',plate,'mount')]
 for x,y in base_holes():out+=stack(f'breaker-base-bolt-{x}-{y}',x,y,168.115,162.115,15.875)
 for x,y in e['worldHoleCentersXY']:
  out+=stack(f'breaker-mount-bolt-{x}-{y}',x,y,e['earSeatZ'],z,25.4)
 return out

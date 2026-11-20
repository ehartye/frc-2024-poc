"""Original A2 spatial CAD in mm. Mechanism envelopes, not fabrication drawings."""
import json,math,pathlib,argparse
import a2_primitives as h
import a2_kinematics as kin

from OCP.BRepPrimAPI import BRepPrimAPI_MakeTorus
from OCP.gp import gp_Ax2,gp_Pnt,gp_Dir
ROOT=pathlib.Path(__file__).resolve().parent

def build(c,state):
 g=c['geometry'];parts=[]
 def add(n,s,k):
  if not h.BRepCheck_Analyzer(s).IsValid():raise ValueError(n)
  parts.append((n,s,k))
 for y in [-355,330]:add('frame longitudinal '+str(y),h.tube(-355,y,100,710,25,50,3,'x'),'frame')
 for x in [-355,330]:add('frame crossrail '+str(x),h.tube(x,-330,100,25,660,50,3,'y'),'frame')
 for y in [-430,355]:add('side bumper '+str(y),h.box(-430,y,40,860,75,150),'bumper')
 for x in [-430,355]:add('end bumper '+str(x),h.box(x,-355,40,75,710,150),'bumper')
 for y in [-300,300]:
  for x in [-240,0,240]:add('wheel '+str((x,y)),h.cyl((x,y-19,76.2),(0,1,0),76.2,38),'drive')
  for x in [-270,270]:add('bumper bracket '+str((x,y)),h.box(x-20,y-20,100,40,70,40),'support')
 for name in ['batteryReserve','electronicsReserve']:
  b=g[name];xyz=[v*1000 for v in b['center']];dims=[v*1000 for v in b['size']];add(name,h.box(*[xyz[j]-dims[j]/2 for j in range(3)],*dims),'battery' if name=='batteryReserve' else 'frame')
  add(name+' service lid',h.box(xyz[0]-dims[0]/2,xyz[1]-dims[1]/2,xyz[2]+dims[2]/2+8,dims[0],dims[1],5),'service')
 i=g['intake'];degree=i['floorAngleDeg'] if state=='deployed' else i['stowAngleDeg'];pose=kin.pose(i,degree);pivot=[v*1000 for v in i['pivot']];a=pose['angleRad'];L=pose['length']*1000
 for ym in i['armYOrigins']:
  y=ym*1000;aw,ah=[v*1000 for v in i['armSection']];sw,sh=[v*1000 for v in i['sleeveSection']]
  add('intake pivot tower '+str(y),h.box(pivot[0]-30,y,150,60,aw,pivot[2]-150),'support')
  add('intake inner telescoping arm '+str(y),h.pitch(h.box(0,y,-ah/2,L,aw,ah),a,pivot),'mechanism')
  add('telescoping sleeve reserve '+str(y),h.pitch(h.box(0,y-(sw-aw)/2,-sh/2,i['sleeveLength']*1000,sw,sh),a,pivot),'support')
 tip=[v*1000 for v in pose['tip']]
 add('capture shaft reserve',h.cyl((tip[0],tip[1]-i['shaftWidth']*500,tip[2]),(0,1,0),i['shaftRadius']*1000,i['shaftWidth']*1000),'support')
 add('capture roller envelope',h.cyl((tip[0],tip[1]-i['rollerWidth']*500,tip[2]),(0,1,0),i['rollerRadius']*1000,i['rollerWidth']*1000),'mechanism')
 off=[v*1000 for v in i['retentionRollerOffset']]
 add('cradle retention roller envelope',h.cyl((tip[0]+off[0],tip[1]+off[1]-i['rollerWidth']*500,tip[2]+off[2]),(0,1,0),i['retentionRollerRadius']*1000,i['rollerWidth']*1000),'mechanism')
 for ym in i['linkYOrigins']:
  w,z=[v*1000 for v in i['linkSection']]
  add('leveling link reserve '+str(ym),h.pitch(h.box(0,ym*1000,i['linkNormalOffset']*1000,L,w,z),a,pivot),'service')
 for y in [-155,149]:
  add('inboard transfer tower '+str(y),h.box(-220,y,160,25,6,890),'support')
 path=c['transport']['transferAndFeed']
 for j,(p,q) in enumerate(zip(path,path[1:])):
  p=[v*1000 for v in p];q=[v*1000 for v in q];ang=math.atan2(q[2]-p[2],q[0]-p[0]);length=math.dist(p,q)
  for y in [-205,199]:add('transfer guide reserve '+str((j,y)),h.pitch(h.box(0,y,-3,length,6,6),ang,p),'service')
 sh=g['shooter'];p=[v*1000 for v in sh['pivot']];angle=math.radians(sh['releasePitchDeg']);length=sh['guideLength']*1000
 for y in [-216,210]:
  add('shooter plate '+str(y),h.pitch(h.box(-25,y,-80,length+50,6,160),angle,p),'mechanism')
  add('shooter support '+str(y),h.box(95,y,150,25,6,p[2]-150),'support')
 sep=1000*(sh['nominalGap']/2+sh['wheelRadius'])
 for side in [-1,1]:
  add('opposed launch roller '+str(side),h.pitch(h.cyl((length-40,-190,side*sep),(0,1,0),50.8,380),angle,p),'mechanism')
  add('launch shaft '+str(side),h.pitch(h.cyl((length-40,-220,side*sep),(0,1,0),6.35,440),angle,p),'support')
 add('reference NOTE',h.pitch(BRepPrimAPI_MakeTorus(gp_Ax2(gp_Pnt(25,0,0),gp_Dir(0,0,1)),152.4,25.4).Shape(),angle,p),'piece')
 return parts

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=pathlib.Path,default=ROOT.parent/'artifacts/a2-cad');a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True);c=json.loads((ROOT/'cycle-contract-A2.json').read_text());reports=[]
 for state in ['stowed','deployed']:
  out=a.output/state;out.mkdir(exist_ok=True);parts=build(c,state);w=h.STEPControl_Writer()
  for n,s,k in parts:
   if w.Transfer(s,h.STEPControl_AsIs)!=h.IFSelect_RetDone:raise ValueError(n)
  f=out/'a2-layout.step'
  if w.Write(str(f))!=h.IFSelect_RetDone:raise ValueError('write')
  r=h.STEPControl_Reader()
  if r.ReadFile(str(f))!=h.IFSelect_RetDone:raise ValueError('reopen')
  r.TransferRoots();ex=h.TopExp_Explorer(r.OneShape(),h.TopAbs_SOLID);count=0
  while ex.More():count+=1;ex.Next()
  if count!=len(parts):raise ValueError('solid roundtrip')
  render(parts,out,state);reports.append({'state':state,'originalParts':len(parts),'reopenedSolids':count,'partNames':[n for n,s,k in parts]})
 (a.output/'cad-report.json').write_text(json.dumps({'schema':'frc-a2-cad/1','status':'original spatial CAD; not build-ready','states':reports,'limitations':c['transport']['unresolved']},indent=2)+'\n')
 print(json.dumps(reports))

def render(parts,out,state):
 import matplotlib;matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 from mpl_toolkits.mplot3d.art3d import Poly3DCollection
 fig=plt.figure(figsize=(13,9),facecolor='#f5f7fa');ax=fig.add_subplot(111,projection='3d',computed_zorder=False);ax.set_facecolor('#f5f7fa')
 for name,s,k in parts:
  h.BRepMesh_IncrementalMesh(s,4,False,.25,True).Perform();expl=h.TopExp_Explorer(s,h.TopAbs_FACE);faces=[]
  while expl.More():
   face=h.TopoDS.Face(expl.Current());loc=h.TopLoc_Location();tri=h.BRep_Tool.Triangulation_s(face,loc)
   if tri:
    for i in range(1,tri.NbTriangles()+1):
     ids=tri.Triangle(i).Get();points=[]
     for idx in ids:
      p=tri.Node(idx).Transformed(loc.Transformation());points.append((p.X(),p.Y(),p.Z()))
     faces.append(points)
   expl.Next()
  ax.add_collection3d(Poly3DCollection(faces,facecolor=dict(h.COLORS,service='#8bc8b0')[k],edgecolor='none',alpha=.22 if k=='service' else (.82 if k=='piece' else 1)))
 ax.set_xlim(-550,750);ax.set_ylim(-500,500);ax.set_zlim(0,1250);ax.set_box_aspect((1.3,1,1.25));ax.view_init(elev=25,azim=-130)
 ax.set_xlabel('Forward X (mm)');ax.set_ylabel('Left Y (mm)');ax.set_zlabel('Up Z (mm)')




 fig.suptitle('2024 original A2 | '+state+' spatial study',fontsize=20);fig.text(.06,.045,'Green: reserved linkage/transfer/service space. Contact, swept clearance and structure unresolved.',fontsize=10)
 fig.savefig(out/'a2-layout.svg',metadata={'Date':None},bbox_inches='tight');fig.savefig(out/'a2-layout.png',dpi=140,bbox_inches='tight');plt.close(fig)

if __name__=='__main__':main()

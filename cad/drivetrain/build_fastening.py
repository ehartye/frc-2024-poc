"""Additive original drilled bridges; no vendor geometry redistributed."""
import pathlib,sys,json,hashlib,argparse
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
from a2_primitives import box,cyl,tube
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut,BRepAlgoAPI_Common,BRepAlgoAPI_Fuse
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.BRepCheck import BRepCheck_Analyzer
HERE=pathlib.Path(__file__).resolve().parent

def volume(s):
 g=GProp_GProps();BRepGProp.VolumeProperties_s(s,g);return g.Mass()
def drill(s,x,y,z,h,d):return BRepAlgoAPI_Cut(s,cyl((x,y,z),(0,0,1),d/2,h)).Shape()
def build(c):
 expected={'plateThicknessMm':4,'plateLengthXmm':96,'plateYinnerMm':220,'plateYouterMm':270,'tubeWallMm':3.175,'tubeBoltYabsMm':228.6,'holeDiameterMm':5.334,'nominalBoltDiameterMm':4.826,'sleeveLengthMm':44.45,'sleeveOuterDiameterMm':9.525,'nutOuterRadiusReserveMm':5.6,'headRadiusReserveMm':4,'headHeightReserveMm':5}
 for key,value in expected.items():
  if type(c[key]) not in (int,float) or c[key]!=value:raise ValueError('unsupported fixed specification: '+key)
 if len(c['bridgeCentersXmm'])!=2 or len(c['railBoltXmm'])!=2:raise ValueError('two stations required')
 if c['tubeSizeMm']!=[25.4,488.95,50.8] or c['boltLengthsMm']!={'rail':19.05,'tube':69.85}:raise ValueError('unsupported stock/fastener')
 for x,holes in zip(c['bridgeCentersXmm'],c['railBoltXmm']):
  if holes!=[x-38.1,x+38.1] and any(abs(a-b)>1e-8 for a,b in zip(holes,[x-38.1,x+38.1])):raise ValueError('hole registration')
 parts=[];z=c['railTopZmm'];pt=c['plateThicknessMm'];bottom=z+pt;top=bottom+50.8;d=c['holeDiameterMm'];r=c['nominalBoltDiameterMm']/2
 for i,x in enumerate(c['bridgeCentersXmm']):
  sh=tube(x-12.7,-244.475,bottom,25.4,488.95,50.8,3.175,'y')
  for sign in [-1,1]:sh=drill(sh,x,sign*228.6,bottom-1,52.8,d)
  parts.append((f'bridge-{i}',sh,'drilled-tube'))
  for sign in [-1,1]:
   y=sign*228.6
   bar=box(x-48,248.5 if sign>0 else -270,z,96,21.5,pt)
   tongue=box(x-20,220 if sign>0 else -248.5,z,40,28.5,pt)
   plate=BRepAlgoAPI_Fuse(bar,tongue).Shape()
   for hx in c['railBoltXmm'][i]:plate=drill(plate,hx,sign*260.35,z-1,pt+2,d)
   plate=drill(plate,x,y,z-1,pt+2,d);parts.append((f'adapter-{i}-{sign}',plate,'drilled-adapter'))
   sleeve=cyl((x,y,bottom+3.175),(0,0,1),4.7625,44.45);sleeve=drill(sleeve,x,y,bottom+3.175-1,46.45,d);parts.append((f'sleeve-{i}-{sign}',sleeve,'custom-crush-sleeve'))
   for label,hx,hy,seat,L,nutTop in [('tube',x,y,top,69.85,z)]+[('rail'+str(j),hx,sign*260.35,z+pt,19.05,c['railBottomZmm']) for j,hx in enumerate(c['railBoltXmm'][i])]:
    key=f'{i}-{sign}-{label}';parts.append(('bolt-'+key,cyl((hx,hy,seat-L),(0,0,1),r,L),'bolt-shank-proxy'));parts.append(('head-'+key,cyl((hx,hy,seat),(0,0,1),4,5),'head-reserve'))
    nt=c['nutThicknessMm'];nut=cyl((hx,hy,nutTop-nt),(0,0,1),5.6,nt);nut=drill(nut,hx,hy,nutTop-nt-1,nt+2,c['nominalBoltDiameterMm']);parts.append(('nut-'+key,nut,'nut-thread-proxy'))
 return parts

def native_check(c,cache):
 from OCP.BRep import BRep_Builder
 from OCP.BRepTools import BRepTools
 from OCP.TopoDS import TopoDS_Shape
 from OCP.gp import gp_Trsf
 from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
 from OCP.BRepExtrema import BRepExtrema_DistShapeShape
 assert hashlib.sha256((cache/'square.step').read_bytes()).hexdigest()==c['assemblySourceSha256']
 rails=[]
 for i in [1,2]:
  p=cache/f'innerrail-{i}.brep';assert hashlib.sha256(p.read_bytes()).hexdigest()==c['railBrepSha256'][i-1]
  s=TopoDS_Shape();BRepTools.Read_s(s,str(p),BRep_Builder());t=gp_Trsf();t.SetValues(0,0,1,-15.79782764615,1,0,0,89.993604498469,0,1,0,26.656576335723);rails.append(BRepBuilderAPI_Transform(s,t,True).Shape())
 parts=build(c);checks=[]
 for name,s,role in parts:
  assert BRepCheck_Analyzer(s).IsValid(),name
  # Check all original parts, including nut/head reserves, against both native rails.
  overlaps=[volume(BRepAlgoAPI_Common(s,r).Shape()) for r in rails];checks.append(dict(id=name,maxOverlapMm3=max(overlaps),distanceMm=min(BRepExtrema_DistShapeShape(s,r).Value() for r in rails)))
 assert max(x['maxOverlapMm3'] for x in checks)<1e-5
 bad=cyl((63.5+1,-260.35,128),(0,0,1),c['nominalBoltDiameterMm']/2,10);negative=max(volume(BRepAlgoAPI_Common(bad,r).Shape()) for r in rails);assert negative>0.1
 return dict(checks=checks,shiftedOneMmBoltOverlapMm3=negative,partCount=len(parts),plateRailContact='zero distance at nominal surfaces; no preload/strength proof',limits=c['limits'])
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--cache',type=pathlib.Path,required=True);p.add_argument('--out',type=pathlib.Path,required=True);a=p.parse_args();c=json.loads((HERE/'fastening.json').read_text());files=[HERE/'fastening.json',pathlib.Path(__file__),HERE.parent/'a2_primitives.py'];pins={q.name:hashlib.sha256(q.read_bytes()).hexdigest() for q in files};report=native_check(c,a.cache);assert pins=={q.name:hashlib.sha256(q.read_bytes()).hexdigest() for q in files};report['inputs']=pins;from OCP.BRep import BRep_Builder
 from OCP.TopoDS import TopoDS_Compound
 from OCP.STEPControl import STEPControl_Writer,STEPControl_AsIs,STEPControl_Reader
 from OCP.IFSelect import IFSelect_RetDone
 from OCP.TopExp import TopExp_Explorer
 from OCP.TopAbs import TopAbs_SOLID
 compound=TopoDS_Compound();builder=BRep_Builder();builder.MakeCompound(compound)
 for _,shape,_ in build(c):builder.Add(compound,shape)
 step=a.out.with_suffix('.step');w=STEPControl_Writer();w.Transfer(compound,STEPControl_AsIs);assert w.Write(str(step))==IFSelect_RetDone
 rd=STEPControl_Reader();assert rd.ReadFile(str(step))==IFSelect_RetDone;rd.TransferRoots();ex=TopExp_Explorer(rd.OneShape(),TopAbs_SOLID);count=0
 while ex.More():count+=1;ex.Next()
 assert count==46;report['reopenedSolids']=count;report['stepSha256']=hashlib.sha256(step.read_bytes()).hexdigest();a.out.write_text(json.dumps(report,indent=2)+'\n',encoding='utf8');print(json.dumps({'count':count,'negativeMm3':report['shiftedOneMmBoltOverlapMm3'],'maxRailOverlapMm3':max(x['maxOverlapMm3'] for x in report['checks'])}))

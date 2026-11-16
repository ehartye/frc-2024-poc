"""Native envelope regression against an explicitly supplied committed drivetrain package."""
import argparse,hashlib,importlib.util,json,pathlib,itertools
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
import build_electrical as e

def hits(a,b):
 out=[]
 for n,s,r in a:
  for k,t,q in b:
   if n==k:continue
   x=Bnd_Box();y=Bnd_Box();BRepBndLib.AddOptimal_s(s,x);BRepBndLib.AddOptimal_s(t,y)
   if x.IsOut(y):continue
   g=GProp_GProps();BRepGProp.VolumeProperties_s(BRepAlgoAPI_Common(s,t).Shape(),g)
   if g.Mass()>1e-4:out.append({'electrical':n,'other':k,'role':r,'otherRole':q,'volumeMm3':g.Mass()})
 return out

def main():
 p=argparse.ArgumentParser();p.add_argument('--drivetrain',type=pathlib.Path,required=True);p.add_argument('--native-cache',type=pathlib.Path);p.add_argument('--output',type=pathlib.Path,required=True);a=p.parse_args();bound_files=[a.drivetrain/'candidate.json',a.drivetrain/'build_drivetrain.py',e.HERE/'manifest.json',e.HERE/'mounts.py',e.HERE/'fasteners.py',e.HERE/'fastening-spec.json',e.HERE/'breaker_mount.py',e.HERE/'breaker-mount-evidence.json',e.HERE/'build_electrical.py',pathlib.Path(__file__),e.HERE.parent/'a2_primitives.py'];before={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in bound_files};s=importlib.util.spec_from_file_location('dt',a.drivetrain/'build_drivetrain.py');dt=importlib.util.module_from_spec(s);s.loader.exec_module(dt);drive=dt.build(json.loads((a.drivetrain/'candidate.json').read_text()));elec=e.build();native_pins={};negative=None
 if a.native_cache:
  from OCP.BRep import BRep_Builder
  from OCP.BRepTools import BRepTools
  from OCP.TopoDS import TopoDS_Shape
  from OCP.gp import gp_Trsf
  from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
  fc=json.loads((a.drivetrain/'fastening.json').read_text());native_pins['fastening.json']=hashlib.sha256((a.drivetrain/'fastening.json').read_bytes()).hexdigest()
  assert hashlib.sha256((a.native_cache/'square.step').read_bytes()).hexdigest()==fc['assemblySourceSha256']
  for i in [1,2]:
   path=a.native_cache/f'innerrail-{i}.brep';h=hashlib.sha256(path.read_bytes()).hexdigest();assert h==fc['railBrepSha256'][i-1];native_pins[path.name]=h
   shape=TopoDS_Shape();BRepTools.Read_s(shape,str(path),BRep_Builder());tr=gp_Trsf();tr.SetValues(0,0,1,-15.79782764615,1,0,0,89.993604498469,0,1,0,26.656576335723);shape=BRepBuilderAPI_Transform(shape,tr,True).Shape()
   drive=[(n,shape if n==f'am-3922-{i}' else sh,r) for n,sh,r in drive]
  from a2_primitives import cyl
  bad=cyl((-265.7,-260.35,128),(0,0,1),2.5,10);negative=hits([('shifted-bolt',bad,'negative-control')],[v for v in drive if v[0]=='am-3922-1']);assert negative and negative[0]['volumeMm3']>0.1
 cross=hits(elec,[x for x in drive if x[2]!='original-interface']);internal=hits(elec,elec);internal=[v for v in internal if v['electrical']<v['other']]
 pins={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in bound_files};assert pins==before

 q={'schema':'electrical-drivetrain-crosscheck/1','inputs':pins,'electricalSolids':len(elec),'retainedDriveSolids':sum(x[2]!='original-interface' for x in drive),'omittedRole':'original-interface (retired bridge proposal)','crossOverlaps':cross,'internalOverlaps':internal,'shiftedOneMmBoltOverlap':negative,'nativeRailPins':native_pins,'nativeRailReplacement':bool(a.native_cache),'commonVolumeReportingThresholdMm3':1e-4,'qualification':False,'note':'Native rail replacement verifies nominal bolt clearance only; source native geometry remains private. No preload, tolerance, strength or complete robot qualification.'};a.output.write_text(json.dumps(q,indent=2)+'\n');print(json.dumps(q))
if __name__=='__main__':main()

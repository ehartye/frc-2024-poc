"""Private native PDH check and public original-mount mass/service estimates."""
import argparse,hashlib,json,pathlib
import build_electrical as e
from check_drivetrain import hits
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from a2_primitives import move

def estimates(parts):
 rows=[]
 for n,s,r in parts:
  if r!='mount':continue
  g=GProp_GProps();BRepGProp.VolumeProperties_s(s,g);rows.append({'id':n,'volumeMm3':g.Mass(),'material':'6061-T6 selected hypothesis','densityKgPerMm3':2.7e-6,'estimatedMassKg':g.Mass()*2.7e-6})
 return {'items':rows,'estimatedOriginalMountMassKg':sum(r['estimatedMassKg'] for r in rows),'sourceKnownDeviceMassUnchangedKg':e.report(e.load())['knownComponentMassKg'],'estimateIsNotMeasuredMass':True,'hardwareAndPadMassComplete':False}

def service(parts):
 battery=next(s for n,s,r in parts if n=='battery');remaining=[p for p in parts if not(p[0]=='battery' or p[0]=='battery-top-strap' or p[0]=='battery-insulating-pad' or p[0].startswith('battery-top-bolt'))]
 trace=[]
 for dz in [0,.5,1,5,10,25,50,100,170,220]:
  conflicts=hits([('moving-battery',move(battery,dz=dz),'service-test')],remaining);trace.append({'liftMm':dz,'overlaps':conflicts})
 return {'batteryLiftSamples':trace,'isContinuousSweepProof':False,'requiresDisconnectedLeadsAndRemovedTopBar':True,'wholeRobotNotIncluded':True}

def main():
 p=argparse.ArgumentParser();p.add_argument('--cache',type=pathlib.Path,required=True);p.add_argument('--output',type=pathlib.Path,required=True);a=p.parse_args();ev=json.loads((e.HERE/'pdh-mount-evidence.json').read_text());files=[e.HERE/n for n in ['manifest.json','mounts.py','fasteners.py','fastening-spec.json','build_electrical.py','check_drivetrain.py','check_fastening.py','pdh-mount-evidence.json','breaker_mount.py','breaker-mount-evidence.json','../a2_primitives.py']];pins={p.relative_to(e.HERE).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
 assert hashlib.sha256((a.cache/'pdh.step').read_bytes()).hexdigest()==ev['stepSha256'];assert hashlib.sha256((a.cache/'pdh.brep').read_bytes()).hexdigest()==ev['brepSha256']
 from OCP.BRep import BRep_Builder
 from OCP.BRepTools import BRepTools
 from OCP.TopoDS import TopoDS_Shape
 from OCP.gp import gp_Trsf
 from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
 s=TopoDS_Shape();BRepTools.Read_s(s,str(a.cache/'pdh.brep'),BRep_Builder());t=gp_Trsf();t.SetValues(0,0,1,-200.85,1,0,0,230,0,1,0,168.115);s=BRepBuilderAPI_Transform(s,t,True).Shape();parts=e.build();bolts=[v for v in parts if v[0].startswith('pdh-bolt')];native=hits(bolts,[('native-pdh',s,'vendor-private')]);be=json.loads((e.HERE/'breaker-mount-evidence.json').read_text());assert hashlib.sha256((a.cache/'breaker.step').read_bytes()).hexdigest()==be['stepSha256'];assert hashlib.sha256((a.cache/'breaker.brep').read_bytes()).hexdigest()==be['brepSha256']
 bs=TopoDS_Shape();BRepTools.Read_s(bs,str(a.cache/'breaker.brep'),BRep_Builder());bt=gp_Trsf();bt.SetValues(1,0,0,-280,0,0,-1,-175,0,1,0,be['zShiftMm']);bs=BRepBuilderAPI_Transform(bs,bt,True).Shape();breaker_hits=hits([v for v in parts if v[0].startswith('breaker-')],[('native-cb185',bs,'vendor-private')])
 out={'schema':'electrical-fastening-check/1','inputs':pins,'nativePDHFastenerOverlaps':native,'nativeBreakerMountOverlaps':breaker_hits,'materialEstimate':estimates(parts),'service':service(parts),'acceptance':False};assert pins=={p.relative_to(e.HERE).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files};a.output.write_bytes((json.dumps(out,indent=2)+'\n').encode());print(json.dumps({'nativePDHFastenerOverlaps':native,'nativeBreakerMountOverlaps':breaker_hits,'estimatedMountKg':out['materialEstimate']['estimatedOriginalMountMassKg'],'batteryLiftClear':all(not x['overlaps'] for x in out['service']['batteryLiftSamples'])}))
if __name__=='__main__':main()

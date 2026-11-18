"""Private source-wheel witnesses distinguish conservative shell from real profile."""
import pathlib,json,math,hashlib
import extract_geometry as e
from OCP.gp import gp_Ax1,gp_Pnt,gp_Dir,gp_Trsf,gp_Ax2
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.STEPControl import STEPControl_Reader
from OCP.BRepPrimAPI import BRepPrimAPI_MakeTorus

def run():
 before=e.a.input_pins();root=e.R;c=json.loads((root/'cad/cycle-contract-A2.json').read_text());i=json.loads((root/'cad/assembly/intake_components.json').read_text());tx=json.loads((root/'cad/components/a2-transmission.json').read_text());src=json.loads((root/'cad/components/a2-wheel-cartridge.json').read_text())['sources']['wheel'];p=pathlib.Path.home()/'.cache/frc-component-cartridge/am-3608.step';assert hashlib.sha256(p.read_bytes()).hexdigest()==src['sha256'];r=STEPControl_Reader();r.ReadFile(str(p));r.TransferRoots();native=r.OneShape();assert len(e.solids(native))==1 and e.BRepCheck_Analyzer(native).IsValid()
 sh=c['geometry']['shooter'];sh['pivot'][0]=i['shooterPivotXmm']/1000;angle=math.radians(sh['releasePitchDeg']);pivot=[v*1000 for v in sh['pivot']];t=tx['serviceInterfaces']['tread'];sep=sh['nominalGap']*500+t['nativeSeatRadiusMm']+t['thicknessMm'];station=sh['guideLength']*1000-40;tr=gp_Trsf();tr.SetRotation(gp_Ax1(gp_Pnt(0,0,0),gp_Dir(1,0,0)),-math.pi/2);rotated=BRepBuilderAPI_Transform(native,tr,True).Shape();wheels=[]
 for side in [-1,1]:
  for lane in [-152.4,152.4]:
   y=lane-t['assemblyTreadCenterOffsetMm'];wheels.append((f'{side}/{lane}',e.b.h.pitch(e.b.h.move(rotated,station,y-15.875,side*sep),angle,pivot)))
 rows=[]
 for advance in [140,220,300]:
  note=BRepPrimAPI_MakeTorus(gp_Ax2(gp_Pnt(i['stageCenterX']+advance*math.cos(angle),0,i['handoffCenterZ']+advance*math.sin(angle)),gp_Dir(-math.sin(angle),0,math.cos(angle))),i['note']['majorRadius'],i['note']['tubeRadius']).Shape();hits={n:e.common_volume(w,note) for n,w in wheels};rows.append({'advanceMm':advance,'nativeBareWheelOverlapMm3':hits});print(rows[-1],flush=True)
 pins=e.a.input_pins();assert before==pins;assert hashlib.sha256(p.read_bytes()).hexdigest()==src['sha256'];pins['cad/components/a2-wheel-cartridge.json']=hashlib.sha256((root/'cad/components/a2-wheel-cartridge.json').read_bytes()).hexdigest();pins['sim/component-launcher/check_native_wheel_profile.py']=hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest();pins['sim/component-launcher/extract_geometry.py']=hashlib.sha256((pathlib.Path(__file__).parent/'extract_geometry.py').read_bytes()).hexdigest();report={'schema':'frc-native-wheel-transfer-witness/1','source':src,'inputs':pins,'rows':rows,'limits':['Three poses and one nominal source clocking only','No tread deformation or fasteners included','Vendor CAD remains private; scalar witnesses only','Does not clear other cradle/carrier collisions']};out=root/'artifacts/current-transfer/native-wheel-profile.json';out.write_bytes((json.dumps(report,indent=2)+'\n').encode())
if __name__=='__main__':run()

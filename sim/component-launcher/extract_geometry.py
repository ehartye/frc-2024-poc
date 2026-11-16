"""Extract actual current four-tread geometry; occupancy is not calibrated force."""
import pathlib,sys,json,hashlib,math,copy
R=pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0,str(R/'cad/assembly'))
import build_intake_candidate as a
import build_robot as b
from OCP.BRepPrimAPI import BRepPrimAPI_MakeTorus
from OCP.gp import gp_Ax2,gp_Pnt,gp_Dir
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp

def solids(s):
 e=TopExp_Explorer(s,TopAbs_SOLID);out=[]
 while e.More():out.append(e.Current());e.Next()
 return out

def common_volume(x,y):
 op=BRepAlgoAPI_Common(x,y);op.Build()
 if not op.IsDone():raise ValueError('Boolean failed')
 z=op.Shape()
 if not BRepCheck_Analyzer(z).IsValid():raise ValueError('Invalid intersection')
 total=0
 for s in solids(z):
  g=GProp_GProps();BRepGProp.VolumeProperties_s(s,g);total+=abs(g.Mass())
 return total

def run():
 pins=a.input_pins();pins['sim/component-launcher/extract_geometry.py']=hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest()
 c=json.loads((R/'cad/cycle-contract-A2.json').read_text());i=json.loads((R/'cad/assembly/intake_components.json').read_text());tx=json.loads((R/'cad/components/a2-transmission.json').read_text());sh=c['geometry']['shooter'];sh['pivot'][0]=i['shooterPivotXmm']/1000
 angle=math.radians(sh['releasePitchDeg']);pivot=[v*1000 for v in sh['pivot']];station=sh['guideLength']*1000-40
 parts=b.shooter(c,tx);wheels=[(n,s) for n,s,k in parts if 'nominal tread annulus' in n]
 assert len(wheels)==4
 for n,s in wheels:assert len(solids(s))==1 and BRepCheck_Analyzer(s).IsValid(),n
 rows=[]
 for x in range(-240,241,10):
  note=b.h.pitch(BRepPrimAPI_MakeTorus(gp_Ax2(gp_Pnt(station+x,0,0),gp_Dir(0,0,1)),i['note']['majorRadius'],i['note']['tubeRadius']).Shape(),angle,pivot)
  hits={n:common_volume(s,note) for n,s in wheels};rows.append({'noteCenterFromWheelStationMm':x,'summedNominalOverlapMm3':sum(hits.values()),'byTreadMm3':hits})
 result={'schema':'frc-current-component-launcher-geometry/1','inputs':pins,'wheelRadiusMm':tx['serviceInterfaces']['tread']['nativeSeatRadiusMm']+tx['serviceInterfaces']['tread']['thicknessMm'],'contactSampling':{'two lane candidate':rows},'shooter':sh,'wheelStationMm':station,'handoffGuideMm':i['handoffGuideCoordinateMm'],'handoffFromStationMm':i['handoffGuideCoordinateMm']-station,'limits':['Actual authored annulus geometry with nominal uncompressed tread','Finite10mm occupancy samples, not stiffness/friction/contact calibration','Fixed guide-aligned NOTE attitude; no contact torque/spin solution','Prescribed handoff speed, not simulated cradle drive','No full field/robot swept release or scoring acceptance']}
 assert pins==dict(a.input_pins(),**{'sim/component-launcher/extract_geometry.py':hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest()})
 (R/'sim/component-launcher/current-geometry.json').write_bytes((json.dumps(result,indent=2)+'\n').encode());print('radius',result['wheelRadiusMm'],'handoff',result['handoffFromStationMm'],'positive',[r['noteCenterFromWheelStationMm'] for r in rows if r['summedNominalOverlapMm3']>1e-6])
if __name__=='__main__':run()

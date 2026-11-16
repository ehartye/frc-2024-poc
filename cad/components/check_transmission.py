"""Replay a private-native transmission fit experiment. No vendor exports."""
import pathlib,json,hashlib,math,argparse,itertools
from build_cartridge import *
from OCP.BRepExtrema import BRepExtrema_DistShapeShape

def place(s,x,z,phase=0):
 t=gp_Trsf();t.SetRotation(gp_Ax1(gp_Pnt(0,0,0),gp_Dir(0,0,1)),math.radians(phase));s=BRepBuilderAPI_Transform(s,t,True).Shape()
 t=gp_Trsf();t.SetTranslation(gp_Vec(x,0,z));return BRepBuilderAPI_Transform(s,t,True).Shape()

def center_distance(c):
 d=c['layout'];n=d['pulleyTeeth'];b=d['beltTeeth'];pitch=d['pitchMm']
 if not (n>0 and b>n and pitch>0):raise ValueError('invalid equal-pulley belt geometry')
 return (b-n)*pitch/2

def motor_originals(c):
 d=c['motorMount'];k=c['key'];x=center_distance(c);z=c['layout']['motorFaceZ'];w=d['plateWidthMm'];th=d['plateThicknessMm']
 plate=BRepPrimAPI_MakeBox(gp_Pnt(-w/2,-w/2,0),w,w,th).Shape()
 plate=BRepAlgoAPI_Cut(plate,BRepPrimAPI_MakeCylinder(d['pilotBoreMm']/2,th).Shape()).Shape()
 for xx,yy in [(d['holeCircleMm']/2,0),(-d['holeCircleMm']/2,0),(0,d['holeCircleMm']/2),(0,-d['holeCircleMm']/2)]:
  t=gp_Trsf();t.SetTranslation(gp_Vec(xx,yy,0));h=BRepBuilderAPI_Transform(BRepPrimAPI_MakeCylinder(d['clearanceHoleMm']/2,th).Shape(),t,True).Shape();plate=BRepAlgoAPI_Cut(plate,h).Shape()
 key=BRepPrimAPI_MakeBox(gp_Pnt(k['radialInnerMm'],-k['widthMm']/2,k['startFromMotorFaceMm']),k['radialThicknessMm'],k['widthMm'],k['lengthMm']).Shape()
 return {'original motor mount plate':place(plate,x,z),'original key envelope':place(key,x,z,k['clockDegrees'])}

def screw_stack(c):
 d=c['motorMount'];eng=d['candidateScrewUnderHeadMm']-d['candidateWasherMm']-d['plateThicknessMm']
 return {'nominalEngagementMm':eng,'bottomReserveMm':d['frontTappedDepthMm']-eng,'nominalDepthScreen':0<eng<=d['frontTappedDepthMm']-d['depthReserveMm'],'strengthAccepted':False}

def run_drive(c,base,cache):
 native={};records=[]
 for s in c['sources']+list(base['sources'].values()):
  p=cache/(s['part']+'.step');raw=p.read_bytes()
  if hashlib.sha256(raw).hexdigest()!=s['sha256']:raise ValueError('source mismatch '+s['part'])
  r=STEPControl_Reader()
  if r.ReadFile(str(p))!=IFSelect_RetDone or r.TransferRoots()!=1:raise ValueError('native import')
  native[s['part']]=r.OneShape();records.append({'part':s['part'],'sha256':s['sha256']})
 d=c['layout'];x=center_distance(c);z=d['pulleyZ'];o=originals(base,0)
 o['bare wheel']=shifted(native['am-3608'],-base['layout']['wheelWidth']/2)
 for side,bc,cc in zip([-1,1],base['layout']['bearingCenters'],base['layout']['collarCenters']):
  o['bearing '+str(side)]=shifted(native['REV-21-1916-PK4'],bc,side<0)
  o['collar '+str(side)]=shifted(native['REV-21-2900-PK4'],cc,side<0)
 o['motor']=place(native['REV-21-1650-V1.1'],x,d['motorFaceZ'])
 o['driver pulley']=place(native['REV-21-2200'],x,z,d['driverClockDegrees'])
 o['driven pulley']=place(native['REV-21-2201'],0,z,d['drivenClockDegrees'])
 o.update(motor_originals(c))
 pairs={a+' / '+b:overlap(o[a],o[b]) for a,b in itertools.combinations(o,2)}
 key=o['original key envelope'];distances={}
 for name in ['motor','driver pulley']:
  dist=BRepExtrema_DistShapeShape(key,o[name]);dist.Perform()
  if not dist.IsDone():raise ValueError('key distance failure')
  distances[name]=dist.Value()
 neg=overlap(o['original rounded hex shaft'],place(native['REV-21-2201'],0,z,30))
 return {'schema':'frc-native-transmission-evidence/1','inputSha256':hashlib.sha256(json.dumps(c,sort_keys=True,separators=(',',':')).encode()).hexdigest(),'baseSha256':hashlib.sha256(json.dumps(base,sort_keys=True,separators=(',',':')).encode()).hexdigest(),'sources':records,'equalPulleyCenterDistanceMm':x,'ratio':1,'keyNearestSurfaceDistancesMm':distances,'screwStack':screw_stack(c),'unclockedDriverKeyOverlapMm3':overlap(o['original key envelope'],place(native['REV-21-2200'],x,z,0)),'pairCommonVolumesMm3':pairs,'wrongDrivenClock30OverlapMm3':neg,'manufacturerMassSubtotalKg':sum(c['manufacturerMassKg'].values()),'massScope':'motor + two pulleys + belt only; no CoG or inertia inferred','acceptance':False,'limits':c['limits']}
if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('--vendor-cache',type=pathlib.Path,required=True);a.add_argument('--output',type=pathlib.Path,required=True);v=a.parse_args()
 c=json.loads((R/'cad/components/a2-transmission.json').read_text());b=json.loads((R/'cad/components/a2-wheel-cartridge.json').read_text());r=run_drive(c,b,v.vendor_cache);v.output.parent.mkdir(parents=True,exist_ok=True);v.output.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({'pairCount':len(r['pairCommonVolumesMm3']),'maxOverlap':max(r['pairCommonVolumesMm3'].values()),'negativeControl':r['wrongDrivenClock30OverlapMm3']}))

 # Optional useful original-only artifact beside the report; no vendor geometry.
 writer=STEPControl_Writer()
 for solid in motor_originals(c).values():
  if not BRepCheck_Analyzer(solid).IsValid():raise ValueError('invalid original motor interface')
  writer.Transfer(solid,STEPControl_AsIs)
 if writer.Write(str(v.output.parent/'original-motor-interfaces.step'))!=IFSelect_RetDone:raise ValueError('export')

"""Replay a private-native transmission fit experiment. No vendor exports."""
import pathlib,json,hashlib,math,argparse,itertools
from build_cartridge import *

def place(s,x,z,phase=0):
 t=gp_Trsf();t.SetRotation(gp_Ax1(gp_Pnt(0,0,0),gp_Dir(0,0,1)),math.radians(phase));s=BRepBuilderAPI_Transform(s,t,True).Shape()
 t=gp_Trsf();t.SetTranslation(gp_Vec(x,0,z));return BRepBuilderAPI_Transform(s,t,True).Shape()

def center_distance(c):
 d=c['layout'];n=d['pulleyTeeth'];b=d['beltTeeth'];pitch=d['pitchMm']
 if not (n>0 and b>n and pitch>0):raise ValueError('invalid equal-pulley belt geometry')
 return (b-n)*pitch/2

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
 pairs={a+' / '+b:overlap(o[a],o[b]) for a,b in itertools.combinations(o,2)}
 neg=overlap(o['original rounded hex shaft'],place(native['REV-21-2201'],0,z,30))
 return {'schema':'frc-native-transmission-evidence/1','inputSha256':hashlib.sha256(json.dumps(c,sort_keys=True,separators=(',',':')).encode()).hexdigest(),'baseSha256':hashlib.sha256(json.dumps(base,sort_keys=True,separators=(',',':')).encode()).hexdigest(),'sources':records,'equalPulleyCenterDistanceMm':x,'ratio':1,'pairCommonVolumesMm3':pairs,'wrongDrivenClock30OverlapMm3':neg,'manufacturerMassSubtotalKg':sum(c['manufacturerMassKg'].values()),'massScope':'motor + two pulleys + belt only; no CoG or inertia inferred','acceptance':False,'limits':c['limits']}
if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('--vendor-cache',type=pathlib.Path,required=True);a.add_argument('--output',type=pathlib.Path,required=True);v=a.parse_args()
 c=json.loads((R/'cad/components/a2-transmission.json').read_text());b=json.loads((R/'cad/components/a2-wheel-cartridge.json').read_text());r=run_drive(c,b,v.vendor_cache);v.output.parent.mkdir(parents=True,exist_ok=True);v.output.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({'pairCount':len(r['pairCommonVolumesMm3']),'maxOverlap':max(r['pairCommonVolumesMm3'].values()),'negativeControl':r['wrongDrivenClock30OverlapMm3']}))

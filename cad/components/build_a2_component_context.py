"""Original A2 integration study: contact geometry, not contact dynamics."""
import pathlib,json,sys,math,hashlib
R=pathlib.Path(__file__).resolve().parents[2];sys.path.insert(0,str(R/'cad'));sys.path.insert(0,str(R/'cad/components'))
import a2_primitives as h
from build_a2_layout import build,render
from build_cartridge import overlap
from OCP.BRepPrimAPI import BRepPrimAPI_MakeTorus
from OCP.gp import gp_Ax2,gp_Pnt,gp_Dir

def run(out):
 out.mkdir(parents=True,exist_ok=True);contract=json.loads((R/'cad/cycle-contract-A2.json').read_text());tx=json.loads((R/'cad/components/a2-transmission.json').read_text());base=json.loads((R/'cad/components/a2-wheel-cartridge.json').read_text());sh=contract['geometry']['shooter'];radius=tx['serviceInterfaces']['tread']['nativeSeatRadiusMm']+tx['serviceInterfaces']['tread']['thicknessMm'];sep=sh['nominalGap']*500+radius;lane=152.4;width=25.4
 cases={}
 for name,ys in [('rejected central wheel',[0]),('two lane candidate',[-lane,lane])]:
  wheels=[h.cyl((0,y-width/2,sep),(0,1,0),radius,width) for y in ys]
  rows=[]
  for x in range(-200,201,10):
   note=BRepPrimAPI_MakeTorus(gp_Ax2(gp_Pnt(x,0,0),gp_Dir(0,0,1)),152.4,25.4).Shape()
   rows.append({'noteCenterFromWheelStationMm':x,'summedNominalOverlapMm3':sum(overlap(w,note) for w in wheels)})
  cases[name]=rows
 parts=[p for p in build(contract,'stowed') if not p[0].startswith(('opposed launch roller','launch shaft','reference NOTE'))]
 angle=math.radians(sh['releasePitchDeg']);pivot=[v*1000 for v in sh['pivot']];station=sh['guideLength']*1000-40
 bom=[]
 for side in [-1,1]:
  for y in [-lane,lane]:
   ident=str((side,y));y-=tx['serviceInterfaces']['tread']['assemblyTreadCenterOffsetMm'];z=side*sep
   def add(n,s,k='mechanism'):parts.append((n+' '+ident,h.pitch(s,angle,pivot),k))
   add('original sourced-size wheel envelope',h.cyl((station,y-15.875,z),(0,1,0),46.99,31.75))
   add('original nominal tread envelope',h.cyl((station,y+tx['serviceInterfaces']['tread']['assemblyTreadCenterOffsetMm']-width/2,z),(0,1,0),radius,width))
   add('original shaft envelope',h.cyl((station,y-80,z),(0,1,0),6.875,160),'support')
   for d in [-40,40]:add('original support plate envelope',h.box(station-40,y+d-1.5875,z-40,80,3.175,80),'support')
   add('original motor body reserve',h.cyl((station+88.9,y-13.35,z),(0,1,0),30,58.35),'frame')
   add('motor axial service reserve',h.box(station+53.9,y+45,z-35,70,80,70),'service')
   bom.append({'id':ident,'wheel':'am-3608','tread':'am-2611a cut section','motor':'REV-21-1650 V1.1','pulleys':['REV-21-2200','REV-21-2201'],'belt':'REV-21-4040','shaftLengthMm':160,'bearings':2,'collars':3,'motorPushRings':2})
 note=BRepPrimAPI_MakeTorus(gp_Ax2(gp_Pnt(station,0,0),gp_Dir(0,0,1)),152.4,25.4).Shape();parts.append(('reference NOTE at station',h.pitch(note,angle,pivot),'piece'))
 writer=h.STEPControl_Writer()
 for n,s,k in parts:writer.Transfer(s,h.STEPControl_AsIs)
 writer.Write(str(out/'original-A2-component-context.step'));render(parts,out,'component candidate (envelopes, not mounted assembly)')
 result={'schema':'frc-a2-component-context/1','contractSha256':hashlib.sha256((R/'cad/cycle-contract-A2.json').read_bytes()).hexdigest(),'transmissionSha256':hashlib.sha256((R/'cad/components/a2-transmission.json').read_bytes()).hexdigest(),'wheelRadiusMm':radius,'opposedCenterHalfSeparationMm':sep,'laneCentersMm':[-lane,lane],'contactSampling':cases,'bomPerCartridge':bom,'status':'unaccepted spatial/contact hypothesis; four independent cartridge drives','limits':['Native overlap measures prescribed undeformed compression geometry, not force/friction or spin','10mm sampling is not continuous contact proof','Original envelope rendering only; vendor geometry not exported','Existing sideplate/support and moving-intake clearance not yet screened','160mm local shafts do not directly attach to previous sideplates','Old 240mm powered-contact plant cannot be reused without measured contact interval','Motor/tension service reserves may conflict; no access acceptance','Mount bridges, tension adjustment and tread fasteners unfinished']}
 (out/'context-report.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps({n:[r['noteCenterFromWheelStationMm'] for r in rows if r['summedNominalOverlapMm3']>1e-5] for n,rows in cases.items()}))
if __name__=='__main__':run(R/'artifacts/a2-component-context')

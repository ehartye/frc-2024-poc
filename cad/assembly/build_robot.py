"""Editable original component-context assembly. Supplier native solids are never exported."""
import json,math,pathlib,sys,hashlib
ROOT=pathlib.Path(__file__).resolve().parents[2]
for d in ['cad','cad/components','cad/drivetrain','cad/electrical']:sys.path.insert(0,str(ROOT/d))
import a2_primitives as h
from build_a2_layout import build as legacy_build,render
from build_drivetrain import build as drivetrain_build
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut,BRepAlgoAPI_Fuse
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib

def cut(a,b):return BRepAlgoAPI_Cut(a,b).Shape()
def ring(x,y,z,ro,ri,width):return cut(h.cyl((x,y,z),(0,1,0),ro,width),h.cyl((x,y-1,z),(0,1,0),ri,width+2))
def bounds(s):
 b=Bnd_Box();BRepBndLib.Add_s(s,b);a=b.CornerMin();z=b.CornerMax();return [a.X(),a.Y(),a.Z(),z.X(),z.Y(),z.Z()]

def shooter(c,tx):
 parts=[];sh=c['geometry']['shooter'];angle=math.radians(sh['releasePitchDeg']);pivot=[v*1000 for v in sh['pivot']]
 t=tx['serviceInterfaces']['tread'];r=t['nativeSeatRadiusMm']+t['thicknessMm'];sep=sh['nominalGap']*500+r;station=sh['guideLength']*1000-40
 for side in [-1,1]:
  for lane in [-152.4,152.4]:
   z=side*sep;y=lane-t['assemblyTreadCenterOffsetMm'];prefix=f'cartridge {side} {lane}'
   def add(name,s,role='mechanism'):parts.append((prefix+' '+name,h.pitch(s,angle,pivot),role))
   add('original wheel shell representation',ring(station,y-15.875,z,46.99,42.2275,31.75))
   add('nominal tread annulus',ring(station,lane-12.7,z,r,45.72,25.4))
   add('shaft round envelope (hex omitted)',h.cyl((station,y-80,z),(0,1,0),6.875,160),'support')
   for d in [-40,40]:
    plate=h.box(station-40,y+d-1.5875,z-40,80,3.175,80)
    plate=cut(plate,h.cyl((station,y+d-3,z),(0,1,0),14.3,6))
    for xx,zz in [(25.4,0),(-25.4,0),(0,25.4),(0,-25.4)]:plate=cut(plate,h.cyl((station+xx,y+d-3,z+zz),(0,1,0),2.5,6))
    add('drilled bearing carrier '+str(d),plate,'support')
    add('bearing envelope '+str(d),ring(station,y+d-3.175,z,14.2875,6.875,6.35),'drive')
   for d in [-47.14375,47.14375,73]:add('collar envelope '+str(d),ring(station,y+d-3.175,z,12.7,6.875,6.35),'drive')
   mx=station+88.9
   add('NEO body envelope',h.cyl((mx,y-13.35,z),(0,1,0),30,58.35),'frame')
   plate=h.box(mx-36,y+45,z-36,72,3.175,72)
   for xx,zz,rr in [(0,0,9.65),(25.4,0,2.6),(-25.4,0,2.6),(0,25.4,2.6),(0,-25.4,2.6)]:plate=cut(plate,h.cyl((mx+xx,y+44,z+zz),(0,1,0),rr,6))
   for xx in [-28,28]:
    for zz in [-28,28]:plate=cut(plate,h.cyl((mx+xx,y+44,z+zz),(0,1,0),2.6,6))
   add('drilled motor slider',plate,'support')
   # Original carrier bridge, 2 mm total adjustment; fastener/load qualification remains open.
   bridge=h.box(station-40,y+48.175,z-40,170,3.175,80)
   for cx,rr in [(station,20),(mx,10.65)]:bridge=cut(bridge,h.cyl((cx,y+47,z),(0,1,0),rr,7))
   for xx in [-28,28]:
    for zz in [-28,28]:
     slot=BRepAlgoAPI_Fuse(h.cyl((mx+xx-1,y+47,z+zz),(0,1,0),2.6,7),h.cyl((mx+xx+1,y+47,z+zz),(0,1,0),2.6,7)).Shape()
     slot=BRepAlgoAPI_Fuse(slot,h.box(mx+xx-1,y+47,z+zz-2.6,2,7,5.2)).Shape();bridge=cut(bridge,slot)
   # Tool access for four face screws; these are not clamping holes.
   for xx,zz in [(25.4,0),(-25.4,0),(0,25.4),(0,-25.4)]:bridge=cut(bridge,h.cyl((mx+xx,y+47,z+zz),(0,1,0),5,7))
   add('slotted carrier bridge proposal',bridge,'support')
   for cx in [station,mx]:add('pulley envelope',ring(cx,y+50.6528,z,13.144,7 if cx==station else 4,18.6944),'drive')
   # Smooth pitch-band representation, explicitly NOT teeth or belt engagement geometry.
   pr=12*6.35/(2*math.pi);outer=BRepAlgoAPI_Fuse(h.cyl((station,y+53.65,z),(0,1,0),pr+1.5,12.7),h.cyl((mx,y+53.65,z),(0,1,0),pr+1.5,12.7)).Shape();outer=BRepAlgoAPI_Fuse(outer,h.box(station,y+53.65,z-pr-1.5,88.9,12.7,2*(pr+1.5))).Shape()
   inner=BRepAlgoAPI_Fuse(h.cyl((station,y+52,z),(0,1,0),pr-1.5,16),h.cyl((mx,y+52,z),(0,1,0),pr-1.5,16)).Shape();inner=BRepAlgoAPI_Fuse(inner,h.box(station,y+52,z-pr+1.5,88.9,16,2*(pr-1.5))).Shape()
   add('belt pitch band representation',cut(outer,inner),'drive')
   add('motor withdrawal reserve',h.box(mx-35,y+80,z-35,70,80,70),'service')
 return parts

def build():
 c=json.loads((ROOT/'cad/cycle-contract-A2.json').read_text());tx=json.loads((ROOT/'cad/components/a2-transmission.json').read_text());dc=json.loads((ROOT/'cad/drivetrain/candidate.json').read_text())
 parts=[(n,s,'drive' if k in ['wheel','motor','belt','gearbox','shaft','hub','bearing'] else 'frame') for n,s,k in drivetrain_build(dc) if k!='original-interface']
 # Keep only provisional intake/transfer from A2. Do not present these as completed component mechanisms.
 for n,s,k in legacy_build(c,'stowed'):
  if n.startswith(('intake ','telescoping ','capture ','cradle ','leveling ','inboard transfer','transfer guide')):parts.append(('UNRESOLVED '+n,s,'service'))
 length,width=dc['frameNativeEnvelopeMm'][:2];th=75
 for sy in [-1,1]:parts.append(('bumper envelope side '+str(sy),h.box(-length/2-th,sy*width/2-(th if sy<0 else 0),40,length+2*th,th,150),'bumper'))
 for sx in [-1,1]:parts.append(('bumper envelope end '+str(sx),h.box(sx*length/2-(th if sx<0 else 0),-width/2,40,th,width,150),'bumper'))
 parts+=shooter(c,tx)
 try:
  import build_electrical
  parts += [(n,s,'service' if 'service' in k else 'battery' if 'battery' in n else 'frame') for n,s,k in build_electrical.build()]
 except ImportError:raise RuntimeError('Electrical package required; integrate worker commit before export')
 return parts,dc

def mass_ledger(parts,dc):
 from OCP.GProp import GProp_GProps
 from OCP.BRepGProp import BRepGProp
 electrical=json.loads((ROOT/'cad/electrical/manifest.json').read_text())
 known=[{'id':p['id'],'massKg':p['massKg']*p.get('quantity',1),'centerMm':p['centerMm'],'basis':'electrical manifest source'} for p in electrical['components'] if p['massKg'] is not None]
 for p in dc['parts']:
  if p['role']=='motor':known.append({'id':p['id'],'massKg':dc['sources']['motor']['massKg'],'centerMm':p['worldCenterMm'],'basis':'CIM manufacturer mass; envelope centroid assumption'})
 estimates=[]
 for n,s,k in parts:
  if not any(x in n for x in ['drilled bearing carrier','drilled motor slider','slotted carrier bridge']):continue
  gp=GProp_GProps();BRepGProp.VolumeProperties_s(s,gp);pt=gp.CentreOfMass()
  estimates.append({'id':n,'massKg':gp.Mass()*2.70e-6,'centerMm':[pt.X(),pt.Y(),pt.Z()],'basis':'original cut CAD volume x assumed aluminum density2700kg/m3; alloy unspecified'})
 def summary(rows):
  total=sum(x['massKg'] for x in rows)
  return {'subtotalKg':total,'partialCentroidMm':[sum(x['massKg']*x['centerMm'][i] for x in rows)/total for i in range(3)],'items':rows}
 return {'sourceKnown':summary(known),'estimatedOriginalHardware':summary(estimates),'wholeRobotCoG':None,'omissions':['drivetrain frame,wheels,gearboxes,belts,shafts and fasteners','shooter COTS,shafts,tread and mounting hardware not yet consolidated','intake,indexer,bumpers,wiring and restraint'],'warning':'Partial centroid is not whole-robot CoG; never density-fill supplier envelope boxes'}

def main():
 out=ROOT/'artifacts/whole-robot';out.mkdir(parents=True,exist_ok=True);parts,dc=build();w=h.STEPControl_Writer()
 for n,s,k in parts:
  if not h.BRepCheck_Analyzer(s).IsValid():raise ValueError(n)
  w.Transfer(s,h.STEPControl_AsIs)
 dest=out/'original-component-assembly.step';assert w.Write(str(dest))==h.IFSelect_RetDone
 reader=h.STEPControl_Reader();assert reader.ReadFile(str(dest))==h.IFSelect_RetDone;reader.TransferRoots()
 render(parts,out,'component integration | unfinished mounts / intake')
 report={'schema':'2024-component-assembly/1','massLedger':mass_ledger(parts,dc),'partCount':len(parts),'reopenedRoots':reader.NbRootsForTransfer(),'chassisMm':dc['frameNativeEnvelopeMm'],'trackMm':dc['centerTrackMm'],'parts':[{'name':n,'role':k,'boundsMm':bounds(s)} for n,s,k in parts],'acceptance':False,'limits':['Original source-sized representations, not redistributed vendor geometry','Green intake/transfer parts are unresolved legacy reserves, not working hardware','Carrier bridge has adjustment holes but clamp screws, jack, spacers and frame anchorage are unfinished','Belt shown as pitch band, not teeth/contact/preload','Tread attachment still fails selected rivet bare-stack grip','No whole robot collision, structure, manufacturing or rules acceptance','No complete mass/CoG; component mass omissions remain']}
 (out/'report.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps({'parts':len(parts),'roots':reader.NbRootsForTransfer()}))
if __name__=='__main__':main()

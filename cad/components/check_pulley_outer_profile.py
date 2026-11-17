"""Private native witness for an original axial outer-envelope approximation."""
import pathlib,sys,json,math,hashlib,argparse
ROOT=pathlib.Path(__file__).resolve().parents[2];sys.path[:0]=[str(ROOT/'cad'),str(ROOT/'cad/assembly')]
import a2_primitives as h
import build_robot as b
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.TopAbs import TopAbs_SOLID

def run(cache):
 path=cache/'REV-21-2201.step';digest=hashlib.sha256(path.read_bytes()).hexdigest()
 if digest!='c2825524b2293af12647d92ad57e9162379ad64105b86f867be2223037059c68':raise RuntimeError('source mismatch')
 reader=h.STEPControl_Reader();reader.ReadFile(str(path));reader.TransferRoots();native=reader.OneShape()
 if not h.BRepCheck_Analyzer(native).IsValid():raise RuntimeError('invalid source')
 solids=[];ex=h.TopExp_Explorer(native,TopAbs_SOLID)
 while ex.More():
  solid=ex.Current()
  if not h.BRepCheck_Analyzer(solid).IsValid():raise RuntimeError('invalid source solid')
  solids.append(solid);ex.Next()
 if len(solids)!=3:raise RuntimeError('unexpected source solids')
 bands=[]
 for lo,hi,proposed in [(-9.3473,-7.14375,13.2),(-7.14375,7.14375,12.0),(7.14375,9.3473,13.2)]:
  sections=[]
  for solid in solids:
   section=BRepAlgoAPI_Common(solid,h.box(-30,-30,lo,60,60,hi-lo)).Shape()
   if h.TopExp_Explorer(section,TopAbs_SOLID).More():sections.append(section)
  if not sections:raise RuntimeError('empty band')
  supports=[]
  # +/-X and +/-Y bounds at16orientations cover64directions, maxangulargap5.625deg.
  for j in range(16):
   tr=h.gp_Trsf();tr.SetRotation(h.gp_Ax1(h.gp_Pnt(0,0,0),h.gp_Dir(0,0,1)),j*math.pi/32)
   boxes=[b.bounds(h.BRepBuilderAPI_Transform(section,tr,True).Shape()) for section in sections];supports.append(max(abs(bounds[i]) for bounds in boxes for i in [0,1,3,4]))
  bound=max(supports)/math.cos(math.pi/64)
  if bound>proposed:raise RuntimeError('outer envelope too small')
  bands.append({'zMinMm':lo,'zMaxMm':hi,'proposedOuterRadiusMm':proposed,'nativeRadialUpperBoundMm':bound,'directionalSupportUpperBoundsMm':supports})
 return {'schema':'pulley-outer-axial-envelope/1','nativeSourceSha256':digest,'scriptSha256':hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest(),'part':'REV-21-2201','nativeSolidCount':len(solids),'bands':bands,'acceptance':False,'limits':['Outer radial containment bound only; bore/teeth/mating geometry omitted from public proxy','Nominal source CAD, not manufacturing tolerance or contact compliance','No vendor shape redistributed; original cylindrical bands are authored separately']}
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--vendor-cache',type=pathlib.Path,required=True);v=p.parse_args();q=run(v.vendor_cache);out=ROOT/'artifacts/intake-component-candidate/pulley-outer-profile.json';out.write_text(json.dumps(q,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps(q))

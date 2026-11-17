"""One-side spring-cassette interface prototype; all new stock is original authored geometry."""
import json,math,pathlib,hashlib
import build_connected_candidate as a
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut,BRepAlgoAPI_Fuse
h=a.h
CONFIG={'side':1,'inwardTravelMm':[0,1.5],'guideCentersXmm':[60,110],'guideZmm':85,'guideRodDiameterMm':6,'guideBoreDiameterMm':6.2,'movingBlockYmm':[203,211],'outerSpringSeatYmm':218,'springOuterDiameterEnvelopeMm':10,'springInnerDiameterEnvelopeMm':6.4,'springFreeLengthMm':None,'springRateNPerMm':None,'adjustableSeatTravelMm':1,'status':'Original interface candidate, not selected spring/bushing hardware or accepted preload'}
def pins():
 p=a.input_pins();p['cad/assembly/spring_cassette_candidate.py']=hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest();return p

def cut(s,t):
 op=BRepAlgoAPI_Cut(s,t)
 if not op.IsDone():raise ValueError('cut')
 return op.Shape()
def fuse(s,t):
 op=BRepAlgoAPI_Fuse(s,t)
 if not op.IsDone():raise ValueError('fuse')
 return op.Shape()

def build(deg,zPivot,travel=0):
 if not 0<=travel<=1.5:raise ValueError('travel outside stops')
 c=json.loads((a.ROOT/'cad/assembly/intake_components.json').read_text());local=[];names=[];stockname='intake candidate original cradle side weldment 1'
 for n,s,k in a.assembly.intake_components.build('floor',c):
  selected=(n.endswith(' 1') or '(1,' in n) and any(t in n for t in ['side pinch belt','vertical stub shaft','12T track pulley','bearing representation','pinch NEO body','pinch motor plate','pinch transfer pulley','remote drive shaft','pinch transfer belt','original cradle side weldment'])
  if not selected:continue
  s=h.move(s,-c['stageCenterX'],0,-c['floorCenterZ']);names.append(n)
  if n==stockname:
   s=cut(s,h.cyl((0,190,0),(0,1,0),13,30));s=cut(s,h.box(-7,190,0,14,30,15))
   s=fuse(s,h.box(-40,203.825,65,206.05,6.35,12))
   for x in [-35,130]:s=fuse(s,h.box(x-5,203.825,0,10,6.35,77))
  local.append((n,h.move(s,0,-travel,0),k,'cassette'))
 fixed=[];added=[]
 arm=cut(h.box(-6,203.5,0,12,3.175,135),h.cyl((0,202.5,0),(0,1,0),6.9,5.175))
 # Lower open arm and shaft clearance are an unresolved clamp/torque interface, not a weld to a shaft.
 fixed.append(('spring cassette original pivot backbone',arm,'original-stock','fixed-to-cradle-pivot'))
 fixed.append(('spring cassette original upper backbone',h.box(-6,203.5,110,126,3.175,12),'original-stock','fixed-to-cradle-pivot'))
 for x in CONFIG['guideCentersXmm']:
  upper=h.box(x-8,203.5,110,16,20.5,6)
  outer=cut(h.box(x-8,218,77,16,6,39),h.cyl((x,217,85),(0,1,0),3,8))
  fixed.append((f'spring cassette anchor bracket {x}',fuse(upper,outer),'original-stock','fixed-to-cradle-pivot'))
  fixed.append((f'spring cassette guide rod {x}',h.cyl((x,197,85),(0,1,0),3,27),'original-rod','fixed-to-cradle-pivot'))
  inner=cut(h.cyl((x,198.5,85),(0,1,0),6,3),h.cyl((x,197.5,85),(0,1,0),3,5))
  fixed.append((f'spring cassette inward stop collar {x}',inner,'original-stop-proxy','fixed-to-cradle-pivot'))
  fixed.append((f'spring cassette outward stop pin {x}',h.cyl((x+6,211,92),(0,1,0),1.5,7),'original-stop-proxy','fixed-to-cradle-pivot'))
  block=cut(h.box(x-8,203-travel,77,16,8,16),h.cyl((x,202-travel,85),(0,1,0),3.1,10))
  added.append((f'spring cassette sliding guide block {x}',block,'original-bushing-reserve','cassette'))
  spring=cut(h.cyl((x,211-travel,85),(0,1,0),5,7+travel),h.cyl((x,210-travel,85),(0,1,0),3.2,9+travel))
  added.append((f'spring cassette spring envelope {x}',spring,'spring-envelope-not-wire','elastic'))
 frame=None
 for n,s,k,m in fixed:
  if k=='original-stock':frame=s if frame is None else fuse(frame,s)
 fixed=[p for p in fixed if p[2]!='original-stock']+[('spring cassette original fixed backbone weldment',frame,'original-stock','fixed-to-cradle-pivot')]
 for n,s,k,m in local+fixed+added:
  if not h.BRepCheck_Analyzer(s).IsValid():raise ValueError('invalid '+n)
 parts=[(n,h.pitch(s,math.radians(deg),[c['stageCenterX'],0,zPivot]),k,m) for n,s,k,m in local+fixed+added]
 return parts,names

if __name__=='__main__':
 parts,names=build(0,25.4);out=a.ROOT/'artifacts/spring-cassette';out.mkdir(parents=True,exist_ok=True);w=h.STEPControl_Writer()
 for n,s,k,m in parts:w.Transfer(s,h.STEPControl_AsIs)
 dest=out/'original-one-side-candidate.step';w.Write(str(dest));r=h.STEPControl_Reader();r.ReadFile(str(dest));r.TransferRoots();print(len(parts),r.NbRootsForTransfer())

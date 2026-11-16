"""Original rotating side-pinch cradle and elevator component layout; millimetres."""
import pathlib,json,math
import a2_primitives as h
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut,BRepAlgoAPI_Fuse
HERE=pathlib.Path(__file__).resolve().parent

def cut(a,b):return BRepAlgoAPI_Cut(a,b).Shape()
def ring_z(x,y,z,r,ri,w):return cut(h.cyl((x,y,z),(0,0,1),r,w),h.cyl((x,y,z-1),(0,0,1),ri,w+2))
def belt_x(cx,cy,z,center,r,width):
 out=BRepAlgoAPI_Fuse(h.cyl((cx-center/2,cy,z),(0,0,1),r+1,width),h.cyl((cx+center/2,cy,z),(0,0,1),r+1,width)).Shape();out=BRepAlgoAPI_Fuse(out,h.box(cx-center/2,cy-r-1,z,center,2*r+2,width)).Shape()
 inner=BRepAlgoAPI_Fuse(h.cyl((cx-center/2,cy,z-1),(0,0,1),r-1,width+2),h.cyl((cx+center/2,cy,z-1),(0,0,1),r-1,width+2)).Shape();inner=BRepAlgoAPI_Fuse(inner,h.box(cx-center/2,cy-r+1,z-1,center,2*r-2,width+2)).Shape()
 return cut(out,inner)

def build(state='floor',c=None,pose=None,metadata=False):
 c=c or json.loads((HERE/'intake_components.json').read_text());p=c['pinch'];g=c['guide'];lift=c['lift'];r=p['teeth']*p['pitch']/(2*math.pi);center=(p['beltTeethHypothesis']-p['teeth'])*p['pitch']/2;track=p['innerFaceYAbs']+r+1
 deg=0 if state=='floor' else c['pitchDeg'];a=math.radians(deg);zc=c['floorCenterZ'] if state=='floor' else c['handoffCenterZ'] if state=='handoff' else c['note']['tubeRadius']+c['note']['majorRadius']*math.sin(a)
 if pose is not None:deg,zc=pose;a=math.radians(deg)
 pivot=[c['stageCenterX'],0,zc];parts=[];records=[]
 def add(n,s,k='mechanism',moving=True):
  shape=h.pitch(s,a,pivot) if moving else s
  motion='cradle' if moving else 'carriage' if n.startswith(('lift carriage','UHMW','pivot drop','pivot bearing','chain carriage','rotation')) else 'fixed'
  records.append({'id':f'intake-{len(parts):03d}','name':'intake candidate '+n,'shape':shape,'role':k,'motion':motion})
  parts.append(('intake candidate '+n,shape,k))
 for side in [-1,1]:
  y=side*track
  add('side pinch belt '+str(side),belt_x(0,y,-p['beltWidth']/2,center,r,p['beltWidth']),'drive')
  for x in [-center/2,center/2]:
   ident=str((side,x))
   add('vertical stub shaft '+ident,h.cyl((x,y,p['shaftLower']),(0,0,1),6.875,46 if x<0 else p['shaftLength']),'support')
   add('12T track pulley representation '+ident,ring_z(x,y,-9.3472,13.144,6.875,18.6944),'drive')
   for zz in [p['lowerBearingZ'],p['upperBearingZ']]:
    add('bearing representation '+ident+str(zz),ring_z(x,y,zz-3.175,14.2875,6.875,6.35),'drive')
    # Small original annular bearing carrier; tie attaches on outboard side.
    add('bearing carrier '+ident+str(zz),ring_z(x,y,zz-4.7625,17,14.3,3.175),'support')
    lo=min(y,side*c['cradleSideYAbs']);span=abs(side*c['cradleSideYAbs']-y)
    add('bearing carrier outboard tie '+ident+str(zz),h.box(x-10,lo,zz-4.7625,20,span,3.175),'support')
   if x>0:
    my=side*p['motorSideYAbs'];face=p['motorFaceZ'];mx=x-math.sqrt(p['motorOffsetY']**2-(my-y)**2)
    add('pinch NEO body '+str(side),h.cyl((mx,my,face),(0,0,1),30,58.35),'frame')
    add('pinch motor plate '+str(side),cut(h.box(mx-36,my-36,face-3.175,72,72,3.175),h.cyl((mx,my,face-4.175),(0,0,1),9.65,5.175)),'support')
    for px,yy in [(x,y),(mx,my)]:add('pinch transfer pulley '+str((side,yy)),ring_z(px,yy,p['drivePulleyZ']-9.3472,13.144,6.875 if yy==y else 4,18.6944),'drive')
    add('remote drive shaft coupling hypothesis '+str(side),h.cyl((mx,my,p['drivePulleyZ']),(0,0,1),4,face-25-p['drivePulleyZ']),'support')
    # Transfer belt rotated90deg in local XY from same source-family pitch geometry.
    b=belt_x(0,0,p['drivePulleyZ']-6.35,p['motorOffsetY'],r,12.7)
    tr=h.gp_Trsf();tr.SetRotation(h.gp_Ax1(h.gp_Pnt(0,0,0),h.gp_Dir(0,0,1)),math.atan2(my-y,mx-x))
    b=h.BRepBuilderAPI_Transform(b,tr,True).Shape();add('pinch transfer belt '+str(side),h.move(b,(x+mx)/2,(y+my)/2,0),'drive')
  add('cradle side stock '+str(side),h.tube(-center/2-20,side*c['cradleSideYAbs']-6.35,-12.7,center+40,12.7,25.4,1.5875,'x'),'support')
  # Short side pivot, never a solid shaft through NOTE.
  add('cradle side pivot '+str(side),h.cyl((0,-228 if side<0 else 200,0),(0,1,0),6.875,47 if side<0 else 28),'support')
  gx=g['x'];gy=side*g['yAbs'];cy=side*lift.get('chainYAbs',g['yAbs']);sq=g['square'];wall=g['wall']
  outer=h.box(gx-sq/2,gy-sq/2,g['bottom'],sq,sq,g['height']);inner=h.box(gx-sq/2+wall,gy-sq/2+wall,g['bottom']-1,sq-2*wall,sq-2*wall,g['height']+2)
  add('stationary aluminum guide '+str(side),cut(outer,inner),'support',False)
  # Original carriage plate + four sliding pad envelopes; bearing/slide loading not qualified.
  plateY=gy+20 if side>0 else gy-26.35
  plate=h.box(gx-30,plateY,zc+c['carriageOffsetMm']-5,125,6.35,90)
  if side<0:plate=cut(plate,h.cyl((c['stageCenterX'],plateY-1,zc+c['rotation']['centerDistance']),(0,1,0),10,8.35))
  add('lift carriage plate '+str(side),plate,'support',False)
  for zz in [zc+c['carriageOffsetMm']+5,zc+c['carriageOffsetMm']+65]:
   for xx in [-1,1]:add('UHMW slide pad '+str((side,zz,xx)),h.box(gx+xx*(sq/2+g['slideClearance']/2)-(6 if xx<0 else 0),gy-10,zz,6,20,20),'mechanism',False)
  bracketY=side*c['pivotBracketYAbs'] if side>0 else side*c['pivotBracketYAbs']-6.35
  bracket=h.box(c['stageCenterX']-25,bracketY,zc-20,50,6.35,c['carriageOffsetMm']+55)
  bracket=cut(bracket,h.cyl((c['stageCenterX'],bracketY-1,zc),(0,1,0),14.3,8.35))
  add('pivot drop bracket '+str(side),bracket,'support',False)
  bearing=cut(h.cyl((c['stageCenterX'],bracketY,zc),(0,1,0),14.2875,6.35),h.cyl((c['stageCenterX'],bracketY-1,zc),(0,1,0),6.875,8.35))
  add('pivot bearing representation '+str(side),bearing,'drive',False)
  for z in [lift['lowerPulleyZ'],lift['upperPulleyZ']]:add('lift16T sprocket envelope '+str((side,z)),h.cyl((lift['driveX'],cy-4.75,z),(0,1,0),lift['sprocketOuterRadius'],9.5),'drive',False)
  # Two thin straight runs plus end pulley envelopes distinguish belt from a solid vertical bar.
  for xx in [-lift['sprocketPitchRadius'],lift['sprocketPitchRadius']]:add('lift chain straight-run envelope '+str((side,xx)),h.box(lift['driveX']+xx-1,cy-6.35,lift['lowerPulleyZ'],2,12.7,lift['upperPulleyZ']-lift['lowerPulleyZ']),'drive',False)
  add('chain carriage anchor '+str(side),h.box(lift['driveX']-3,plateY,zc+lift['carriageAnchorAboveNote'],g['x']-lift['driveX']+3,6.35,6.35),'support',False)
 # Separate12:1 rotary cradle drive rides on carriage;40T equal12T belt gives88.9mm centers.
 ry=c['rotation']['chainPlaneY'];rx=c['stageCenterX'];ux=c['rotation']['upperX'];rz=zc+c['rotation']['upperZOffset']
 for xx,zz in [(rx,zc),(ux,rz)]:add('rotation16T sprocket '+str(zz),h.cyl((xx,ry-4.75,zz),(0,1,0),17.95,9.5),'drive',False)
 for off in [-16.4,16.4]:
  line=h.box(0,ry-4.75,off-1,c['rotation']['centerDistance'],9.5,2);line=h.pitch(line,math.atan2(rz-zc,ux-rx),[rx,0,zc]);add('rotation chain run '+str(off),line,'drive',False)
 add('rotation MAXPlanetary12to1 envelope',h.box(ux-25.4,-171,rz-30.95,50.8,63.5,61.9),'frame',False)
 add('rotation NEO envelope',h.cyl((ux,-107.5,rz),(0,1,0),30,58.35),'frame',False)
 # Source-sized two-stage geared NEO reservation at top; output retention still incomplete.
 add('lift top shaft',h.cyl((lift['driveX'],-290,lift['upperPulleyZ']),(0,1,0),6.875,580),'support',False)
 add('lift MAXPlanetary12to1 envelope',h.box(lift['driveX']-25.4,-348.5,lift['upperPulleyZ']-30.95,50.8,63.5,61.9),'frame',False)
 add('lift NEO envelope',h.cyl((lift['driveX'],-406.85,lift['upperPulleyZ']),(0,1,0),30,58.35),'frame',False)
 return records if metadata else parts

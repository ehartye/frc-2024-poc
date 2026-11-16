"""Contract-driven sampled geometry screens, not collision/rules qualification."""
import json,math,pathlib,sys,hashlib
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'cad'))
from a2_kinematics import pose,moving_bounds

def separated(poly,rect):
 axes=[(1,0),(0,1)]
 for p,q in zip(poly,poly[1:]+poly[:1]):axes.append((-(q[1]-p[1]),q[0]-p[0]))
 for a in axes:
  A=[p[0]*a[0]+p[1]*a[1] for p in poly];B=[p[0]*a[0]+p[1]*a[1] for p in rect]
  if max(A)<min(B) or max(B)<min(A):return True
 return False

def screen(c,step=.1):
 if step<=0 or not math.isfinite(step):raise ValueError('step')
 g=c['geometry'];i=g['intake'];b=g['bumper'];front=g['frame'][0]/2;back=front+b['depth'];rect=[(front,b['bottom']),(back,b['bottom']),(back,b['top']),(front,b['top'])]
 hits=[];payloadHits=[];maxForward=-math.inf;maxHeight=-math.inf;maxSide=0;maxRear=-math.inf;payloadMax=-math.inf;violations=[];travelFailures=[];N=math.ceil((i['stowAngleDeg']-i['floorAngleDeg'])/step)
 for j in range(N+1):
  deg=i['floorAngleDeg']+(i['stowAngleDeg']-i['floorAngleDeg'])*j/N
  try:p=pose(i,deg)
  except ValueError as e:travelFailures.append({'angle':deg,'reason':str(e)});continue
  co,si=math.cos(p['angleRad']),math.sin(p['angleRad']);px,py,pz=i['pivot'];L=p['length'];half=i['armSection'][1]/2
  poly=[(px+x*co-z*si,pz+x*si+z*co) for x,z in [(0,-half),(L,-half),(L,half),(0,half)]]
  if not separated(poly,rect):hits.append(deg)
  for bound in moving_bounds(i,p):
   mx,mn=bound['max'],bound['min'];maxForward=max(maxForward,mx[0]);maxHeight=max(maxHeight,mx[2]);maxRear=max(maxRear,-mn[0]);maxSide=max(maxSide,abs(mn[1]),abs(mx[1]))
   if mx[2]>i['maximumHeight']+1e-9 or mn[2]<0 or mx[0]>front+g['extensionLimit']+1e-9 or -mn[0]>front+g['extensionLimit']+1e-9 or max(abs(mn[1]),abs(mx[1]))>g['frame'][1]/2+g['extensionLimit']+1e-9:violations.append({'angle':deg,'component':bound['id']})
  cx,cy,cz=p['tip'];pr=c['transport']['payloadEnvelope']['outerDiameter']/2;ph=c['transport']['payloadEnvelope']['thickness']/2;payloadMax=max(payloadMax,cx+pr)
  if cx+pr>=front and cx-pr<=back and cz+ph>=b['bottom'] and cz-ph<=b['top']:payloadHits.append(deg)
 return {'schema':'frc-a2-intake-section-screen/2','inputContractSha256':hashlib.sha256(json.dumps(c,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest(),'sampledPoses':N+1,'angleStepMaximumDeg':step,'frontBumperArmSectionIntersections':hits,'payloadBoundingBoxBumperIntersections':payloadHits,'movingComponentEnvelopeViolations':violations,'unreachableTelescopePoses':travelFailures,'maxMovingComponentHeightM':maxHeight,'maxMovingComponentForwardM':maxForward,'maxMovingComponentRearM':maxRear,'maxMovingComponentLateralM':maxSide,'nominalForwardMarginM':front+g['extensionLimit']-maxForward,'maxHeldPieceForwardM':payloadMax,'scope':'Every modeled moving arm/sleeve/link/roller/shaft height and extension bound; fixed CAD parts checked separately; front-bumper collision only, not all assembly collision','limits':['sampled motion not continuous proof','ideal retained horizontal NOTE and prescribed two-axis motion','no tolerance, flex, joint hardware or actuator qualification']}
if __name__=='__main__':
 c=json.loads((R/'cad/cycle-contract-A2.json').read_text());r=screen(c);(R/'artifacts/a2-intake-screen.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))

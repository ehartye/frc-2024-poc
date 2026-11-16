"""Sampled 2D section screen, not complete assembly collision qualification."""
import json,math,pathlib
R=pathlib.Path(__file__).resolve().parents[1]
def separated(poly,rect):
 axes=[(1,0),(0,1)]
 for p,q in zip(poly,poly[1:]+poly[:1]):axes.append((-(q[1]-p[1]),q[0]-p[0]))
 for a in axes:
  A=[p[0]*a[0]+p[1]*a[1] for p in poly];B=[p[0]*a[0]+p[1]*a[1] for p in rect]
  if max(A)<min(B) or max(B)<min(A):return True
 return False
c=json.loads((R/'cad/cycle-contract-A2.json').read_text());i=c['geometry']['intake'];rect=[(.355,.04),(.430,.04),(.430,.190),(.355,.190)];hits=[];maxreach=0;payloadHits=[];payloadMax=0;floorPayloadClearance=None
for tenth in range(-400,1101):
 a=math.radians(tenth/10);L=min(.68,.52/math.cos(a)) if math.cos(a)>0 else .68;co,si=math.cos(a),math.sin(a)
 poly=[(.1+x*co-z*si,.48+x*si+z*co) for x,z in [(0,-.01),(L,-.01),(L,.01),(0,.01)]]
 if not separated(poly,rect):hits.append(tenth/10)
 maxreach=max(maxreach,.1+L*co+.035)
 cx,cz=.1+L*co,.48+L*si;payloadMax=max(payloadMax,cx+.1778)
 if tenth==-400:floorPayloadClearance=cx-.1778-.430
 # Conservative filled-disk bounding box contains the horizontal torus.
 if cx+.1778>=.355 and cx-.1778<=.430 and cz+.0254>=.04 and cz-.0254<=.190:payloadHits.append(tenth/10)
r={'schema':'frc-a2-intake-section-screen/1','angleStepDeg':.1,'sampledPoses':1501,'armSectionM':[.02,.02],'frontBumperSectionIntersections':hits,'maxRollerForwardM':maxreach,'extensionLimitM':.355+.3048,'nominalExtensionMarginM':.355+.3048-maxreach,'payloadBoundingBoxBumperIntersections':payloadHits,'maxHeldPieceForwardM':payloadMax,'initialHeldPieceToBumperHorizontalGapM':floorPayloadClearance,'payloadScope':'level NOTE full outer diameter355.6mm/thickness50.8mm bounding box; gamepiece extent separate from robot-component extension screen','limits':['2D arm section only; no full piece/cradle/linkage sweep','no allowance for flex, tolerances, telescoping joints, fasteners or backlash','fixed .68m arm rejected at intermediate poses; telescope is a new unvalidated mechanism']}
(R/'artifacts/a2-intake-screen.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))

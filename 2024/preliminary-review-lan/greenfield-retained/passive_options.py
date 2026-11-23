"""Matched passive-cover and shared-lift trades, evaluated against the current cassette."""
from travel import *
from math import asin,degrees,hypot
# Replace the capture bridge, uprights, socket, guide and pin in this alternative only.
def localpart(n):return place(G[n],inverse)
excluded={n for n,m in meta.items() if m['group']=='Capture latch'}
cover=box(36,-218,80,372,436,2); ring=place(torus,mat(0,215,0,50.4))
# Ring is clear of all body material along forward travel in the stock cassette.
# Continuous along s: rails and central/side belts are tangent; check a polygonal
# conservative sweep using OCC extrusion of torus faces for the actual test.
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.TopAbs import TopAbs_FACE
from OCP.gp import gp_Vec
def one(n,s):return dict(gap=distance(ring,s),raised80=common(place(ring,mat(0,0,0,80)),s))
nu=fuse(fuse(torus,ann((0,0,0),Z,177.8,127,80)),place(torus,mat(0,0,0,80)))
sweep=place(nu,mat(0,215,0,50.4));panelblock=common(sweep,cover)
samples=[]
for f in np.linspace(0,1,49):
    org=poses(cases[2],f);m=np.linalg.inv(mat(pick,pivot_x(pick),0,501))
    nr=place(G[held],m@org['piece']);samples.append(common(nr,cover))
# A 3 mm wire flap pivots across the leading rim. Two outside supports on the
# cassette carry its axle. Model the rigid obstruction and clear scoring position.
flap=box(398,-37,32,3,74,46)
# Pivot about upper edge: rotate flap toward +s and above the ring in open state.
hinge=mat(0,399.5,0,78)
opened=place(flap,hinge@mat(100,0,0,0)@np.linalg.inv(hinge))
run=[]
for delta in np.arange(0,551,2):
    piece=place(ring,mat(0,delta,0,0))
    if close(bounds(piece),bounds(flap)):
        run.append([float(delta),common(piece,flap)])
cleargap=distance(opened,ring);opencoll=max(common(place(ring,mat(0,k,0,0)),opened) for k in np.arange(0,551,10))
# Flat lid alone never occludes translation through the mouth, independently of
# pitch: n is invariant for this path and 80 > 50.4 + 25.4.
mass_kg=.372*.436*.002*1200
# Candidate hook on cassette side beams: choose its location via chain height,
# park extension and gross static pivot torque, then flag support/hold requirements.
rows=[]
for s,h in [(570,60),(235,60),(210,105)]:
    radius=hypot(s,h);phase=np.arctan2(h,s)
    theta=degrees(asin((717.55+13.335-501)/radius)-phase) if radius>=717.55+13.335-501 else None
    if theta is None:continue
    w=68.56*9.80665
    # At initial bearing contact and after pulling down through horizontal.
    lever=lambda a:(s*cos(radians(a))-h*sin(radians(a)))/1000
    rows.append(dict(s=s,n=h,chainContactAngleDeg=theta,pivotAtContactX=pivot_x(theta),hookAtContactX=pivot_x(theta)+1000*lever(theta),reachReserveMm=501+radius-(717.55+13.335),
        verticalTravelToHorizontalMm=717.55+13.335-(501+h),gravityTorqueContactNm=w*abs(lever(theta)),gravityTorqueHorizontalNm=w*abs(lever(0)),
        hookAtHorizontalX=pivot_x(0)+s,minimumBearingRadialReactionN=w,twiceStaticHorizontalNm=2*w*abs(lever(0))))
out=dict(step=sha(HERE/'retained-cassette.step'),cover=dict(sizeMm=[372,436,2],material='Polycarbonate planning density 1200 kg/m³, before flanges/mounts',massKg=mass_kg,seatedClearanceMm=distance(cover,ring),outwardNormalSweepObstructionMm3=panelblock,forwardExitClearanceMm=80-75.8,maxImposedIntakeCommonMm3=max(samples),atPickupMouthForceN=.25*9.80665*sin(radians(50))),gate=dict(closedFirstPositiveMm=next(k for k,v in run if v>.001),closedMaxIntrusionMm3=max(v for k,v in run),openAngleDeg=100,openedSampledExitCommonMm3=opencoll,limitation='One-way flap must intentionally open during outward scoring. Angle cam or force-rated flexure is a candidate. Hinge, inward acquisition, spring and positive release remain unqualified.'),sharedClimb=rows)
dump(HERE/'passive-options.json',out);print(out)


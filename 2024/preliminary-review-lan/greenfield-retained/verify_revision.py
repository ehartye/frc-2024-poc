from travel import *
from engineering import service
from math import hypot, atan2, degrees
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
view=HERE/'retained-cassette.step'
write_step(view)
def part(n):return place(G[n],inverse)
ring=place(torus,mat(0,215,0,50.4));pl=part(pin)
def slidepiece(s=0,h=0):return place(ring,mat(0,s,0,h))
sweep=place(fuse(fuse(torus,ann((0,0,0),Z,177.8,127,80)),place(torus,mat(0,0,0,80))),mat(0,215,0,50.4))
roof=part('Capture bridge');lower=part('Lower pin socket')
up=place(pl,mat(0,0,0,66))
assert common(up,slidepiece())<.001
escape=common(sweep,roof)
forward=common(slidepiece(75),pl)
assert escape>10 and forward>5 and common(slidepiece(75),up)<.001
bar=part('Runner crossbar 125');closedRad=hypot(215-130,55)
# A path along the pin through the ring's spanning disk crosses that disk once.
# Its return along the +Y upright crosses the NOTE plane outside its outer radius.
# Every structural interface is seated. Positive nominal overlaps used only as adverse controls.
chain=[('Lower pin socket','Runner crossbar 125'),('Runner crossbar 125','Cassette side beam · left'),('Runner crossbar 125','Latch upright · left'),('Latch upright · left','Capture bridge'),('Capture bridge','Latch solenoid and guide')]
connections=[dict(pair=[a,b],gapMm=distance(part(a),part(b))) for a,b in chain]
assert all(i['gapMm']<1e-5 for i in connections)
# Use radial clearance to the closed bore instead of claiming zero-clearance pin contact.
ret=dict(parkRingClearMm=distance(ring,pl),normalSweepObstructionMm3=escape,axial75mmObstructionMm3=forward,withdrawnAxialControlMm3=common(slidepiece(75),up),
 closedTipN=14,bottomSocketN=[19,25],guideN=[86,168],pinN=[14,98],socketDiametralPlayMm=.2,
 connectionGaps=connections,spanningDiskCrossingRadiusMm=closedRad,ringClearBoreRadiusMm=127,
 exteriorReturnRadiusMm=hypot(85,222),gravityLoadN=.25*9.80665,
 input='Nominal rigid ring. Six millimetres of lower socket engagement plus 5 mm in the lower bar. Clearance joints stay closed while seated.',
 limitation='Requires a verified closed-position switch, spring force, positive pin retention and tests with a deformed foam NOTE.')
def reachcheck():
    # Compute axis-aligned actual per-part cassette bounds in the local datum.
    ab=np.array([bounds(part(n)) for n in moving])
    def front(ang):
        # Include either end of plunger stroke even if the interlock is released.
        ab2=np.vstack([ab,bounds(up)]);c=cos(radians(ang));s=sin(radians(ang))
        return pivot_x(ang)+np.max(np.maximum(c*ab2[:,0],c*ab2[:,3])+np.maximum(-s*ab2[:,2],-s*ab2[:,5]))
    # Lipschitz bound from largest local radius of stock and generating slide rectangle.
    delta=.05;vals=[front(a) for a in np.arange(pick,park+.0001,delta)]
    rad=2*max(hypot(max(abs(a[0]),abs(a[3])),max(abs(a[2]),abs(a[5]))) for a in ab)
    bound=max(vals)+rad*radians(delta)/2
    return dict(upperBoundX=bound,frontLimitX=644.8,minimumReserveMm=644.8-bound,stepDeg=delta,interSampleAllowanceMm=rad*radians(delta)/2)
reachResult=reachcheck();assert reachResult['minimumReserveMm']>0
ms=poses(cases[5],0)
index=posegeom(G['Battery service pin'],ms['index'])
disk=posegeom(G['Service index disk'],ms['arm'])
wrong=place(G['Service index disk'],mat(84,pivot_x(80),0,501)@inverse)
lock=dict(selectedAngle=80,pinVsDiskCommon=common(index,disk),misindexedFourDegCommon=common(index,wrong),engagementMm=10,
 serviceSweep=[service(80),service(125)],limit='The pin locks rotation. Chock or hold the slide, verify loads and include handling and the disconnected harness before servicing.')
assert lock['pinVsDiskCommon']<.001 and lock['misindexedFourDegCommon']>0
# Cable extends from the drum tangent to the moving crosshead. Its fully extended volume
# contains the cable at every intermediate extension.
cord=cyl((-313,10,235),Z,1.6,468);cordpairs={}
for n,sh in G.items():
    if n in ['Climb cable','Climber crosshead','Chain saddle'] or not close(bounds(cord),bounds(sh)):continue
    if (v:=common(cord,sh))>.001:cordpairs[n]=v
climb=dict(freeChainMm=717.55,hookCrownUndersideMm=625,reachStrokeMm=142,unloadedEntryHeadroomMm=625+142-(717.55+13.335),throatMm=35,chainEnvelopeMm=26.67,extendedCableHits=cordpairs,
    operatingMassUpperKg=0,screenFactor=2,hold='Single rear hook, suspended attitude and ratchet strength need a loaded test before scoring credit.')
data=dict(sourceStep=sha(view),retention=ret,forwardExtension=reachResult,service=lock,climb=climb)
dump(HERE/'checks.json',data);print(ret,reachResult,lock,climb,flush=True)


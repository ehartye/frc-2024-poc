"""Frictionless, quasi-static held-angle screen. Test an actual open escape path."""
from layout import *
from math import radians, sin, cos
# The closed swept set of a torus translated along its symmetry axis consists
# of the two rounded end sections and the intervening full radial annulus.
stroke=80.0
motion=mat(0,0,0,stroke)
end=place(torus,motion)
bridge=ann((0,0,0),Z,177.8,127,stroke)
sweep=place(fuse(fuse(torus,bridge),end),seat)
hits=[]; checked=0
a=bounds(sweep)
for name,body in G.items():
    if name==held:continue
    b=bounds(body)
    if any(a[i]>b[i+3]+1e-6 or b[i]>a[i+3]+1e-6 for i in range(3)):continue
    checked+=1;v=common(sweep,body)
    if v>0.001:hits.append([name,v])
# A test coupon over the rim leaves a real gap at admission and blocks escape.
coupon=place(box(-12,142,30,24,20,5),seat)
m=.25;g=9.80665
forces=[dict(angleDeg=ang,gravityAlongChannelN=-m*g*sin(radians(ang)),gravityOutOfDeckN=-m*g*cos(radians(ang))) for ang in [-50,0,45,90,105,125]]
record=dict(sourceStep=sha(HERE/'sliding-pivot.step'),massKg=m,gravityMS2=g,angleDeg=park,strokeMm=stroke,
            gravity=forces,parkedNarrowPairs=checked,intersections=hits,
            outwardHeightLossMm=-stroke*cos(radians(park)),couponInitialGapMm=distance(G[held],coupon),
            couponSweepCommonMm3=common(sweep,coupon),claim='Open gravity-assisted path; fixed cassette, with no belt preload or friction credit. Directional escape test, not a falling-body trajectory.')
dump(HERE/'gravity-hold.json',record)
print(record)


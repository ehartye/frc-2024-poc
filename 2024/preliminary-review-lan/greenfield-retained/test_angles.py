import pickup_physics as sim
import math as m
for a in (30,35,40,43,46,50):
 for z in (.008,.017):
    sim.angle=a;sim.C=m.cos(m.radians(a));sim.S=-m.sin(m.radians(a));sim.X=.612-.625*sim.C+.013*sim.S;sim.Z=z-.625*sim.S-.013*sim.C
    sim.press=True;sim.topGeom=(.645,.091,.033);sim.stroke=.28;sim.shoe=0
    d=sim.run(32,.001,True,True);print(a,z,sim.X,sim.Z,d['minimumCenterS'],d['seatResidenceS'],d['trajectory'][-1][8:],flush=True)


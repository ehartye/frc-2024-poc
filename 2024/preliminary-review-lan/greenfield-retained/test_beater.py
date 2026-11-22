import pickup_physics as sim
import math as m
for s,h,r in [(.605,.113,.103),(.620,.12,.107),(.595,.114,.107),(.596,.134,.122),(.610,.128,.113),(.622,.135,.118)]:
 for phase in (0,35):
    sim.beater=True;sim.phase=phase;sim.press=True;sim.topGeom=(s,h,r);sim.stroke=.19
    d=sim.run(32,.001,True,True);print(s,h,r,phase,d['minimumCenterS'],d['seatResidenceS'],d['trajectory'][-1][8:],flush=True)


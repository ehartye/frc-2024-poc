from pickup_physics import *
import pickup_physics as sim
rows=[]
for angle in [50,51,52,52.5,53]:
 for s,h,rr in [(.625,.075,.025),(.640,.085,.032),(.645,.092,.039),(.636,.080,.034),(.622,.076,.033),(.615,.075,.032)]:
    sim.angle=angle;sim.C=m.cos(m.radians(angle));sim.S=-m.sin(m.radians(angle));sim.topGeom=s,h,rr
    row=sim.run(32,.001,True,True);rows.append([angle,s,h,rr,row['minimumCenterS'],row['seatResidenceS']]);print(rows[-1],flush=True)
(HERE/'pickup-search.json').write_text(json.dumps(rows))


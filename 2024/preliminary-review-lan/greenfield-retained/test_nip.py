import pickup_physics as sim
import math as m,json
rows=[]
for angle in [50,51,52]:
 for topN in [.058,.064,.069]:
  for stroke in [.095,.175]:
    sim.angle=angle;sim.C=m.cos(m.radians(angle));sim.S=-m.sin(m.radians(angle))
    sim.topGeom=(.640,topN+.032,.032);sim.stroke=stroke;sim.press=True
    d=sim.run(32,.001,True,True);rows.append([angle,topN,stroke,d['minimumCenterS'],d['seatResidenceS']]);print(rows[-1],flush=True)
(sim.HERE/'nip-search.json').write_text(json.dumps(rows))


import pickup_physics as sim
rows=[]
for enabled in (True,False):
 for mu in (.45,.75,1.05):
  for width in (.026,.11):
   sim.shoe=width;sim.press=True;sim.topGeom=(.640,.094,.032);sim.stroke=.34
   d=sim.run(32,.001,enabled,True,mu=mu);rows.append([enabled,mu,width,d['minimumCenterS'],d['seatResidenceS'],d['trajectory'][-1][8:]]);print(rows[-1],flush=True)


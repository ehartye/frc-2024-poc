"""Distance from the free-size rigid NOTE to powered surfaces along the imposed infeed."""
from travel import *
ms=poses(cases[2],0)
parts=['Nose pickup roller','Opposed traction belt · left','Opposed traction belt · right']
shapes=[posegeom(G[n],ms[group(n)]) for n in parts]
# Candidate upper run of a 45 mm-wide central traction belt. Test the contact surface only.
# An actual loop and its nose pulley would need a relief in the full-width pickup roller.
strip=place(box(91,-22.5,23,534,45,2),mat(pick,pivot_x(pick),0,pivot[2]))
rows=[]
for i in range(49):
    t=i/48;mx=poses(cases[2],t)['piece']
    ring=posegeom(G[held],mx);ds=[distance(ring,s) for s in shapes]
    rows.append(dict(fraction=t,installedClosestPoweredMm=min(ds),installedSurface=parts[ds.index(min(ds))],centralUpperRunMm=distance(ring,strip)))
worst=max(rows,key=lambda r:r['installedClosestPoweredMm'])
alt=max(rows,key=lambda r:min(r['installedClosestPoweredMm'],r['centralUpperRunMm']))
result=dict(samples=49,rows=rows,largestInstalledGap=worst,largestWithTrialCenterBeltMm=min(alt['installedClosestPoweredMm'],alt['centralUpperRunMm']),candidateBeltNoseCommonMm3=common(strip,shapes[0]))
dump(HERE/'drive-contact.json',result)
print({k:v for k,v in result.items() if k!='rows'})


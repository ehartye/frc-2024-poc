"""Slide law for the replaced lip and installed followers. Uses the installed mesh vertices."""
from pathlib import Path
import json,numpy as np,math
p=Path(__file__).resolve().parent
def mat(a,x=0,z=0):
    a=math.radians(a);c,s=math.cos(a),math.sin(a)
    return np.array([[c,0,-s,x],[0,1,0,0],[s,0,c,z],[0,0,0,1.]])
old=json.loads((p.parent/'pickup-lip/layout.json').read_bytes());verts=[]
for b in [*old['parts'].values(),*json.loads((p/'mesh.json').read_bytes()).values()]:
    verts.extend(b['v'])
a=np.array(verts);m=np.linalg.inv(mat(125,185,501));a=a@m[:3,:3].T+m[:3,3]
# Reduce the vertex cloud to the exposed convex section for the angular lookup.
points=sorted(set((x,z) for x,_,z in a))
def boundary(points):
    h=[]
    for x,z in points:
        while len(h)>1 and (h[-1][0]-h[-2][0])*(z-h[-2][1])<=(h[-1][1]-h[-2][1])*(x-h[-2][0]):h.pop()
        h.append((x,z))
    return h[:-1]
a=np.array([[x,0,z] for x,z in boundary(points)+boundary(points[::-1])])
angles=np.linspace(-52,166,4361)
def target(q):
    co=math.cos(math.radians(q));s=math.sin(math.radians(q))
    reach=max(-18*co,643*co)+max(44*s,-67*s)
    return min(185,634-reach,640-max(a[:,0]*co-a[:,2]*s))
slide=np.array([target(q) for q in angles])
def scheduled(q):return np.interp(q,angles,slide)


from pathlib import Path
import numpy as np, json, sys, hashlib
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
from OCP.gp import gp_Pnt
p=Path(__file__).resolve().parent;root=p.parent;sys.path.insert(0,str(root/'assembly-recheck-01'))
from sweep import load_solids,name
from layout import distance
def read(f):return json.loads(f.read_bytes())
robot={name(n):s for n,s in load_solids(root/'tread-underlay-01/field-review.step')[0].items()}
out=read(p/'hang-occupancy.json');source=read(root/'chain-contact-02/front48.json')
c=read(p/'original-climb.json')
skipped=['Common keeper yoke and sliding rack','Keeper drive mast','Chain hook · left','Chain hook · right']
rows={};partial=p/'chain-neighbors.json';horizon=source['trace'][-1]['t']
for n,pairs in out['chainCandidates'].items():
    if n in skipped:continue
    minGap=1e6;worst=None;count=0;hits=[]
    # Use 100 ms poses and include each body's first and last potentially occupied frame.
    for i,seg in pairs:
        if i%4 and i not in (pairs[0][0],pairs[-1][0]):continue
        r=source['trace'][i];m=np.array(c['series'][c['rig'][n]][i+30]).reshape(3,4)
        nodes=np.array(r['chain'][seg:seg+2])*1000;v=(nodes-m[:,3])@m[:,:3]
        edge=BRepBuilderAPI_MakeEdge(gp_Pnt(*v[0]),gp_Pnt(*v[1])).Edge()
        gap=distance(edge,robot[n])-13.335;count+=1
        if gap<minGap:minGap=gap;worst=[r['t'],seg,v.tolist()]
        if gap<-.002:hits.append([r['t'],seg,gap])
    rows[n]=dict(count=count,minimumGapMm=minGap,worst=worst,hits=hits)
    partial.write_text(json.dumps(dict(layout=out['layout'],scope='CAD distance to the capsule center segment minus the modeled 13.335 mm radius. Sampled every 100 ms plus first/last broad-phase candidates.',rows=rows),separators=(',',':')));print(n.encode('ascii','replace'),minGap,len(hits),flush=True)



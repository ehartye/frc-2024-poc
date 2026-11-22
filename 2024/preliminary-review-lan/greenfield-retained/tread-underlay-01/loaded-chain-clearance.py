"""Check the moving chain against the hardware omitted from the unilateral hook model."""
from pathlib import Path
import sys,json,hashlib,numpy as np
p=Path(__file__).resolve().parent;root=p.parent
sys.path.insert(0,str(root/'assembly-recheck-01'))
from sweep import load_solids,name,ip
from layout import mat,place,box,distance,bounds
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
from OCP.gp import gp_Pnt
raw={name(n):s for n,s in load_solids(p/'field-review.step')[0].items()}
rig=json.loads((root/'keeper-drive-02/result.json').read_bytes())['rig']
stock={n:place(raw[n],ip) for n in rig if not n.startswith('Chain hook')}
yoke='Common keeper yoke and sliding rack'
# The slide tongues are already modeled as contact surfaces. The high bridge and rack are not.
stock[yoke]=BRepAlgoAPI_Common(stock[yoke],box(160,-130,272,90,260,130)).Shape()
bb={n:np.array(bounds(s)) for n,s in stock.items()};summaries={}
def rotation(q):
    w,x,y,z=q;return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
for mode in ('front48','rear24'):
    file=root/'chain-contact-02'/(mode+'.json');record=json.loads(file.read_bytes());seen=0;out={};radius=13.335
    for r,g in zip(record['trace'],record['gateTrace']):
        rr=rotation(r['q']);arm=mat(r['angle'],185,0,501)
        chain=((np.array(r['chain'])*1000-np.array(r['p'])*1000)@rr-arm[:3,3])@arm[:3,:3]
        for n,s in stock.items():
            nodes=chain.copy()
            if n==yoke:nodes[:,2]-=1000*g[1]
            lo=np.minimum(nodes[1:],nodes[:-1]);hi=np.maximum(nodes[1:],nodes[:-1])
            b=bb[n];coarse=np.linalg.norm(np.maximum(0,np.maximum(b[:3]-hi,lo-b[3:])),axis=1)-radius
            # Check nearest occupied segment of each chain polyline until the next AABB bound exceeds the incumbent.
            for j in np.argsort(coarse):
                if n in out and coarse[j]>out[n][0]+1.e-6:break
                a=gp_Pnt(*nodes[j]);b2=gp_Pnt(*nodes[j+1]);edge=BRepBuilderAPI_MakeEdge(a,b2).Edge()
                val=distance(edge,s)-radius;seen+=1
                if n not in out or val<out[n][0]:out[n]=[val,r['t'],int(j)]
    summaries[mode]=dict(checks=seen,minimum=out,source=hashlib.sha256(file.read_bytes()).hexdigest(),samples=len(record['trace']))
(p/'loaded-chain-clearance.json').write_text(json.dumps(dict(assembly=hashlib.sha256((p/'field-review.step').read_bytes()).hexdigest(),radiusMm=radius,scope='Minimum over saved chain and chassis poses. The bridge check includes yoke material above local Z272 mm. No intersample or deformation allowance.',runs=summaries),indent=2));print(summaries)


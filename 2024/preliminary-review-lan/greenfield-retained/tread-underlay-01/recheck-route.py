"""Use the occupied volumes of the latest layout on the same scoring legs."""
from pathlib import Path
p=Path(__file__).resolve().parent;root=p.parent
s=(root/'six-piece-02/route.py').read_text(encoding='utf8')
prefix="""scene=read(root.parent/'www/data.json');variant=scene['concepts']['field'];rig=dict(cs['rig'])
for n in variant['remove']:rig.pop(n,None)
newrig=next(c for c in variant['cases'] if c['key']=='field-96')['rig']
rig.update({n:newrig[n] for n in variant['add']})
cs=dict(cs,rig=rig);design['bodies'].update(variant['add']);design['step']=variant['step']
"""
exec(compile(s[:s.index("if __name__=='__main__':")].replace("vs={role:",prefix+"vs={role:").replace('1.53,5.55','1.54,5.55'),'current robot through the field','exec'))
previous=read(root/'split-field-02/plan.json');checks=[];idle=order.copy()
for leg in previous['plan']:
    k=leg['id'];q=leg['yaw'];idle.remove(k);out=np.array(leg['out']);ret=np.array(leg['loaded']);G=Grid(idle,q,low=True)
    for j,(a,b) in enumerate(zip(out,out[1:])):
        checks.append([k,'out',j,G.clear(a,b)])
    checks.append([k,'pickup',0,G.clear(out[-1],ret[0])])
    for j,(a,b) in enumerate(zip(ret,ret[1:])):
        if k=='middle' and j==0:grid=G
        elif k=='middle' and j==1:grid=Grid(idle,q,turn=True)
        else:grid=Grid(idle,0.)
        checks.append([k,'carry',j,grid.clear(a,b)])
bad=[r for r in checks if not r[-1]]
(p/'route-review.json').write_text(json.dumps(dict(layout=design['step'],originalPlan=sha(root/'split-field-02/plan.json'),checks=checks,failures=bad,scope='Existing route segments checked against registered fixtures and dormant non-target NOTES on the updated occupied bounds. Peers and final docking errors are separate.'),indent=2));print(len(checks),bad)


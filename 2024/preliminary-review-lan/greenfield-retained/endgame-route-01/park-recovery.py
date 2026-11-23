from access import *
original=read(root/'six-piece-01/park-issued.json')
start=np.array(original['waypointsIn'][0])*25.4;goal=np.array(original['waypointsIn'][-1])*25.4
f=Field(-18);path=search(f,start,goal);out=dict(layout=layout['step'],goal=goal.tolist(),path=[v.tolist() for v in path],times=times(path),destination=f.hits(goal,goal,True),directNegative=f.hits(start,goal,True)[:5],segments=[f.hits(a,b,True) for a,b in zip(path[:-1],path[1:])])
if path:
    out['timeFromDecision']=2.9+sum(times(path));out['spare']=24-out['timeFromDecision']
    # The old bumper witness offset must still land on real bumper geometry.
    v=np.array(bodies['Blue bumper set']['v']);tri=np.array([[220.97,110.845],[220.97,212.405],[133.01,161.63]])*25.4
    def margin(pt):
        delta=np.roll(tri,-1,axis=0)-tri
        edge=np.cross(np.c_[delta,np.zeros(3)],np.c_[pt-tri,np.zeros(3)])[:,2]/np.linalg.norm(delta,axis=1);return min(edge)
    witness=max(v,key=lambda v:margin(goal+v[:2]));out['bumperWitness']=witness.tolist();out['minInteriorMm']=float(margin(goal+witness[:2]))
(p/'park-recovery.json').write_text(json.dumps(out,indent=2));print(out.get('timeFromDecision'),out.get('minInteriorMm'),out['destination'])


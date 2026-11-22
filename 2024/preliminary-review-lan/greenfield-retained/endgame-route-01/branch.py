from access import *
a=read(p/'access.json');route=a['climb']['terminal snapshot'];waiting=np.array(a['waiting']);mouth=np.array(a['mouth'])
gate=Field(a['angle'],0);openGate=Field(a['angle'],44)
# Park the same empty robot outside the chain, with a measured depth inside the blue upright triangle.
tri=np.array([[220.97,110.845],[220.97,212.405],[133.01,161.63]])*25.4
v=np.array(bodies['Blue bumper set']['v'])
def witness(goal):
    delta=np.roll(tri,-1,axis=0)-tri;w=goal+v[:,:2]
    dx=w[:,None,:]-tri;edge=(delta[None,:,0]*dx[:,:,1]-delta[None,:,1]*dx[:,:,0])/np.linalg.norm(delta,axis=1)
    margin=edge.min(axis=1);i=np.argmax(margin);return float(margin[i]),v[i].tolist()
opts=[]
for offset in range(0,401,20):
    goal=mouth+np.array([offset,0]);depth,w=witness(goal);hits=gate.hits(waiting,goal)
    if depth>=25 and not hits:opts.append((offset,goal,depth,w))
assert opts;offset,goal,depth,w=opts[-1]
# Sweep closed and open controls at the stand-off. Wire/gear take-up is part of service allowance.
turn=1.2; ptime=route['driveSeconds']+turn+times([waiting,goal])[0]
access=a['openGateTurn'];closed=transition(waiting,125,a['angle'],0)
assert access['pass5mm'] and closed['pass5mm'] and not a['finalApproachBlocked']
for k,r in a['climb'].items():
    arrival=r['driveSeconds']+turn+a['finalDrive']
    r.update(arrival=arrival,confirmed=arrival+1.2+read(root/'chain-contact-02/front48.json')['confirmAt'],poweredOff=arrival+1.2+8,narrativeBudget=arrival+10)
o=dict(layout=a['layout'],park=dict(position=goal.tolist(),bumperLocalMm=w,depthInsideFieldTriangleMm=depth,time=ptime,gate='closed',riseS=turn,closedTurn=closed),branches=a['climb'],openTurn=access,ingress='Standing terminal branch uses the archived snapshot with five peers. The sixth-score location is a separate timing sensitivity.',scope='1.2 s gross reposition for 33.7 degrees. Gating before entry and drivetrain power must be available. Timed source trials for load transfer begin at chain engagement.')
(p/'branch.json').write_text(json.dumps(o,indent=2));print(depth,ptime,[(k,v['narrativeBudget']) for k,v in a['climb'].items()])


"""Two-pin service link between the translating block and fixed A-frame at the battery service index."""
from pathlib import Path
exec(compile((Path(__file__).parent/'lock-source.py').read_text('utf8'),'cold service geometry','exec'))
# The removable flat link approaches from the outboard (-Y) service side.
points=np.array([[178.,418.],[scheduled(-40)+10,463.]])
direction=(points[1]-points[0]);length=np.linalg.norm(direction);normal=np.array([-direction[1],direction[0]])/length
def plate(y,th,radius):
    corners=[points[0]+normal*radius,points[1]+normal*radius,points[1]-normal*radius,points[0]-normal*radius]
    a=prism([(x,y,z) for x,z in corners],(0,th,0))
    for x,z in points:a=fuse(a,ypeg(x,z,radius,y,th))
    return a
link=plate(-266,5,8)
link=fuse(link,ypeg(*points[0],8,-261,1))
tool={};updated={};edges={};envelopes={};control={}
for (x,z),n,desc,y0,th in zip(points,[rightFrame,rightBlock],['frame','truck'],[-260,-261],[12,14]):
    x=float(x);z=float(z);s=scene[n]
    # Shaft counterbore crosses joint; model major thread/pilot only within the tapped region.
    hole=ypeg(x,z,3.20,-267,7.0 if desc=='frame' else 6.0)
    link=cut(link,hole)
    smooth=ypeg(x,z,3.20,y0-.01,3.51);pilot=ypeg(x,z,2.1,y0+3.5,th-3.49)
    altered=cut(cut(s,smooth),pilot);expected=math.pi*(3.2**2*3.5+2.1**2*(th-3.5))
    assert abs(vol(s)-vol(altered)-expected)<.001,(n,vol(s)-vol(altered),expected)
    updated[n]=altered if n==rightFrame else place(altered,move(-delta))
    scene[n]=altered
    head=ypeg(x,z,5,-271,5);shoulder=ypeg(x,z,3,-266,y0+3-(-266))
    thread=ypeg(x,z,2.5,y0+3,7)
    screw=fuse(fuse(shoulder,head),thread);tool['Service slide shoulder screw · '+desc]=screw
    # Socket tool solid represents the full straight tool-access cylinder at engagement.
    envelopes['Hex driver · '+desc]=ypeg(x,z,4,-349,78)
    capZone=ypeg(x,z,2.5,y0+3.5,6.51)
    outside=common(cut(screw,capZone),altered)
    assert outside<.001
    edges[desc]=dict(component=n,centerXZ=[x,z],hostThicknessMm=th,clearShoulderDepthMm=3,underheadSeatMm2=math.pi*(5**2-3.2**2),tappedMajorOverlapMm=6.5,threadCommonMm3=common(screw,altered),outsideThreadMm3=outside,exitReserveMm=th-10)
    # The screw, entirely withdrawn toward the operator, passes through space occupied by its driver.
    envelopes['Screw pull · '+desc]=ypeg(x,z,5,-347,81)
tool['Service slide link']=link
bad=[]
for n,s in tool.items():
    for other,h in scene.items():
        if not nearby(bounds(s),bounds(h)):continue
        v=common(s,h)
        allowed=(n.endswith('· frame') and other==rightFrame) or (n.endswith('· truck') and other==rightBlock)
        if v>.002 and not allowed:bad.append([n,other,v])
for n,s in envelopes.items():
    for other,h in scene.items():
        if nearby(bounds(s),bounds(h)):
            v=common(s,h)
            if v>.002:bad.append([n,other,v])
# Translate the complete plate cross section through the full 75 mm straight service approach.
sweepLink=plate(-341,80,8);sweepBoss=ypeg(*points[0],8,-341,81)
for other,h in scene.items():
    for swept in [sweepLink,sweepBoss]:
        if nearby(bounds(swept),bounds(h)):
            v=common(swept,h)
            if v>.002:bad.append(['Link installation',other,v])
for n,s in tool.items():
    if n!='Service slide link':assert common(s,link)<.002
# Battery and its handling allowance are separate from the tool access.
pack=bounds(stock['Battery and terminal case'])
battery=box(*pack[:3],pack[3]-pack[0],pack[4]-pack[1],pack[5]-pack[2]+370)
batteryMargins={n:distance(s,battery) for n,s in tool.items()}
for dx in [-2,2]:
    # Translate block while holding its shoulder screw fixed. Check only its shear-bearing section.
    xx,zz=points[1];peg=ypeg(xx,zz,3,-261,3);control[str(dx)]=common(peg,place(scene[rightBlock],move(dx)))
assert min(control.values())>10
ratio=length/abs(direction[0]);horizontal=2*55.963*9.80665
physical=dict(test='Assumed isolated 2 g horizontal inertia bound using entire upper robot mass at the service pose; external impacts excluded.',horizontalN=horizontal,linkN=ratio*horizontal,linkNetStressMPa=ratio*horizontal/((16-6.4)*5),shoulderSingleShearMPa=ratio*horizontal/(math.pi*3**2),blockBearingMPa=ratio*horizontal/(6*3),worstEstimatedHorizontalBacklashMm=ratio*4*.20)
out=dict(parent=digest(source),wrongPoseControl=control,contacts=bad,seats=edges,pullMm=75,batteryToolGapMm=batteryMargins,statics=physical,toolMetalKg=sum(vol(s)*(2700 if n=='Service slide link' else 7850)*1e-9 for n,s in tool.items()),issues='Thread engagement, actual fastener geometry, lower-bound material strength, restraint install sequence, de-energized handling and proof load need shop verification.')
save(p/'truck-lock.json',out)
print('Findings:',bad,control,physical,flush=True)
if bad:raise ValueError('Assembly or access interference')
G.clear();G.update(stock);G.update(updated);write_step(p/'service-locked-robot.step')
G.clear();G.update(tool);write_step(p/'removable-truck-lock.step')
for path,expected in [('service-locked-robot.step',len(stock)),('removable-truck-lock.step',len(tool))]:
    imported={name(k):s for k,s in load_solids(p/path)[0].items()};assert len(imported)==expected
    for n,s in (updated if expected==len(stock) else tool).items():assert abs(vol(imported[n])-vol(s))<.001,n
out.update(assembly=digest(p/'service-locked-robot.step'),fixture=digest(p/'removable-truck-lock.step'),stockRemovalKg=sum((vol(stock[n])-vol(s))*2700e-9 for n,s in updated.items()))
save(p/'truck-lock.json',out);save(p/'truck-lock-mesh.json',{n:viewmesh(s) for n,s in (updated|tool).items()});print('Accepted',out['assembly'])


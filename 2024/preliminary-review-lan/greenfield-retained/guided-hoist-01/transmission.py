"""Machined shoe, positive slide stops, drive dog and explicit cable paths in the same preliminary assembly."""
from pathlib import Path
import math
p=Path(__file__).resolve().parent
spec=(p/'routed.py').read_text(encoding='utf8')
exec(compile(spec.split('fixedHits=[]')[0],'guided-rail layout','exec'))
oldsub=subtracted
def update(k,s,rho=None):
    assert BRepCheck_Analyzer(s).IsValid()
    rho=rows[k]['densityKgM3'] if rho is None else rho
    parts[k]=s;v=vol(s);rows[k].update(volumeMm3=v,kg=v*rho*1e-9,densityKgM3=rho)
def bar(a,b,rad):
    a=np.array(a);b=np.array(b);v=b-a;return cyl(a,v/np.linalg.norm(v),rad,np.linalg.norm(v))
def hring(y,l,ro,ri):return ann((0,y,0),Y,ro,ri,l)
def hexbar(y,l,af):
    rr=af/math.sqrt(3)
    return prism([(rr*math.cos(k*math.pi/3),y,rr*math.sin(k*math.pi/3)) for k in range(6)],(0,l,0))
def dshape(r,name):
    """D shaft retains a flat 6.8 mm from the center for positive drum drive."""
    return cut(r,box(6.8,-310,-25,22,620,50))
shaft=members['Pivot torque shaft']
for side,h in [(1,'left'),(-1,'right')]:
    def b(x,y,z,dx,dy,dz):return box(x,y if side==1 else -y-dy,z,dx,dy,dz)
    def ax(x,y,z,r,dy):return cyl((x,y if side==1 else -y-dy,z),Y,r,dy)
    def an(x,y,z,ro,ri,dy):return cut(ax(x,y,z,ro,dy),ax(x,y-.01,z,ri,dy+.02))
    def K(n):return n+' · '+h
    # Move faces inboard 4 mm, retaining room for two dead wraps plus the working stroke over a 3.2 mm winding pitch.
    drum=an(0,193,0,28,24.5,18)
    for y in (193,209):
        fl=cut(ax(0,y,0,32,2),dshape(ax(0,y-.01,0,9.1,2.02),h))
        drum=fuse(drum,fl)
    update(K('Pivot cable drum'),drum)
    shaft=cut(shaft,b(6.75,192,-10,4,20,20))
    # Pin holes and lined carrier seat. Service pin retracts clear of the channel wall until the shoe reaches its sensor.
    shoeName=K('Captive hook wear block')
    shoe=parts[shoeName]
    dogBore=ax(180,193,164,3.1,29)
    shoe=cut(shoe,dogBore)
    for yy in [196.2,215.0]:
        liner=b(154,yy,155,52,.8,17)
        update(shoeName,cut(parts[shoeName],liner),2700)
        shoe=cut(shoe,liner)
        liner=cut(liner,dogBore)
        add(K('Hook slider liner '+str(yy)),liner,'carriage',1420)
    update(shoeName,shoe,2700)
    trackName=K('Hook guide channel');track=cut(parts[trackName],ax(-50,211,164,3.2,10))
    update(trackName,track)
    lug=an(-50,218,164,8,3.15,8)
    add(K('Slide stop guide'),lug,'cassette',7850)
    add(K('Slide stop pin'),ax(-50,218.3,164,3,14),'stop',7850)
    add(K('Slide stop actuator'),an(-50,226,164,10,4,22),'cassette',2700)
    # Rope free lengths: from drum to redirect on their common outer tangent, then along the running lane.
    c1=np.array([0.,0.]);c2=np.array([-131.,151.]);rr,rg=29.5,14.5
    d=c2-c1;unit=d/np.linalg.norm(d);tan=(rr-rg)/np.linalg.norm(d)
    normal=unit*tan+np.array([-unit[1],unit[0]])*math.sqrt(1-tan*tan)
    st=c1+rr*normal;fin=c2+rg*normal
    def pt(pair):return [pair[0],side*202,pair[1]]
    add(K('Hoist rope drum to sheave'),bar(pt(st),pt(fin),1.5),'cassette',970)
    add(K('Hoist rope working span'),bar(pt(c2+[0,rg]),pt([153,165.5]),1.5),'cord',970)
    # A captive aluminum end ferrule inside the carrier transfers tension to the cross pin.
    ferrule=an(154,199,165.5,3.7,1.6,6)
    # A simplified tension anchor with cross pin and 3 mm rope port; independent of the stop bore.
    ferrule=cyl((151,side*202,165.5),X,3.8,13)
    ferrule=cut(ferrule,cyl((150.9,side*202,165.5),X,1.55,10))
    ferrule=cut(ferrule,ax(161,196,165.5,2,22))
    shoe=cut(parts[shoeName],cyl((150,side*202,165.5),X,3.9,14.2))
    bore=ax(161,195,165.5,2,22);update(shoeName,cut(shoe,bore))
    add(K('Hoist rope anchor'),ferrule,'carriage',2700)
    add(K('Hoist anchor cross pin'),ax(161,196.3,165.5,1.98,19.4),'carriage',7850)
# Socket on the right cheek and axially sliding, shaft-keyed coupling.
origShaftVol=vol(members['Pivot torque shaft']);subtracted+=origShaftVol*7850e-9
shaft=cut(shaft,box(6.75,-247,-10,4,19,20));add('Pivot torque shaft',shaft,'drum',7850)
cy=lambda x,y,z,r,l:cyl((x,y,z),Y,r,l)
# Clutch ring ports, three positive teeth at r17, fixed ring on the arm. Outward withdrawal is 4.2 mm.
cup=hring(-232,6,22,10);plate=hring(-238,6,21,9.1)
plate=fuse(plate,cut(cy(0,-238,0,12,6),dshape(cy(0,-238.1,0,9.1,6.2),'collar')))
for ang in [0,120,240]:
    a=math.radians(ang);x=17*math.cos(a);z=17*math.sin(a)
    cup=cut(cup,cy(x,-232.1,z,3.1,4.3));plate=fuse(plate,cy(x,-232,z,3,4))
add('Pivot drive socket',cup,'cassette',7850);add('Sliding pivot clutch',plate,'clutch',7850)
# Package 3 functional mounting holes in the disk; underlying cheek material is checked at each pilot.
boltSites=[(15*math.cos(math.radians(a)),15*math.sin(math.radians(a))) for a in [60,180,300]]
cheek='Cassette cheek · right';subtracted+=vol(members[cheek])*2700e-9
cs=members[cheek];measure=[]
for x,z in boltSites:
    cb=cy(x,-232.1,z,2.15,6.2);pilot=cy(x,-226,z,1.65,7.5)
    measure.append(dict(x=x,z=z,fullDepthMm=common(cs,pilot)/(math.pi*1.65**2)))
    cs=cut(cs,pilot);cup=cut(cup,cb)
update('Pivot drive socket',cup);add(cheek,cs,'cassette',2700)
def placedAt(k,stroke,pin,release):
    s=parts.get(k,members.get(k))
    if role.get(k)=='cord':
        side=1 if k.endswith('left') else -1
        return bar([-131,side*202,165.5],[153-stroke,side*202,165.5],1.5)
    if role.get(k)=='carriage' or k in moving:return place(s,offset(-stroke))
    if role.get(k)=='stop':return place(s,offset(0,(-pin if k.endswith('left') else pin)))
    if role.get(k)=='clutch':return place(s,offset(0,-release))
    return s
partsAll={**members,**parts};hits=[];near=[];pairs=0
states=[(0,0,0),(0,0,4.2),(55,0,4.2),(110,0,4.2),(165,0,4.2),(220,0,4.2),(230,0,4.2),(230,8,4.2)]
for stroke,dep,release in states:
    arr={k:placedAt(k,stroke,dep,release) for k in partsAll};bbs={k:np.array(bounds(s)) for k,s in arr.items()}
    for i,(a,sa) in enumerate(arr.items()):
        for j,(b,sb) in enumerate(arr.items()):
            if i>=j or (a not in parts and b not in parts) or not candidate(bbs[a],bbs[b]):continue
            pairs+=1;v=common(sa,sb)
            if v>.01:hits.append([stroke,dep,release,a,b,v])
    if not stroke and not release:neutral=arr
testPin=place(parts['Slide stop pin · left'],offset(0,-8))
control=common(testPin,placedAt('Captive hook wear block · left',226,8,4.2))
twist=common(parts['Pivot drive socket'],place(parts['Sliding pivot clutch'],mat(5,0,0,0)))
twistOut=common(parts['Pivot drive socket'],place(placedAt('Sliding pivot clutch',0,0,4.2),mat(5,0,0,0)))
shaftSigned={}
for k in ['Pivot cable drum · left','Pivot cable drum · right','Sliding pivot clutch']:
    shaftSigned[k]=[common(shaft,place(parts[k],mat(a,0,0,0))) for a in [0,-4,4]]
out=dict(parent=digest(source),netReplacedKg=subtracted,netStockKg=sum(x['kg'] for x in rows.values())-subtracted,rows=rows,states=states,intersections=hits,comparisons=pairs,cheekPilots=measure,wrongStopAt226Mm=control,wrongClockClutchMm3=[twist,twistOut],shaftFlatMm3=shaftSigned)
G.clear();G.update({k:place(s,park) for k,s in parts.items()});write_step(p/'cable-package.step');out['step']=digest(p/'cable-package.step');save(p/'transmission.json',out)
print(out['netStockKg'],pairs,len(hits),measure,control,twist,twistOut)


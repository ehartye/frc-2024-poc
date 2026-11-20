"""Verify actual deck bores, support contacts and service access on a reopened archive."""
import math
from adsk import core, fusion
from physical_bumper import dimensions, registered_solid
from physical_harness import BASE_HOLES, DECK_Z, POSTS, ROUTES, SHELF_Z, cylinder, xy


def inspect(design):
    mgr=fusion.TemporaryBRepManager.get()
    parts={}
    for occ in design.rootComponent.allOccurrences:
        for b in occ.bRepBodies:
            shape=registered_solid(occ,b)
            parts[b.name]=(shape,dimensions(shape))

    def overlap(a,b):
        c=mgr.copy(a)
        if not mgr.booleanOperation(c,b,fusion.BooleanTypes.IntersectionBooleanType):
            raise ValueError('Cable support intersection')
        return c.volume*1000 if c.isValid and c.lumps.count else 0.

    def volume(a,skip=()):
        bounds=dimensions(a)
        return {key:round(v,5) for key,(shape,b) in parts.items() if key not in skip
                and all(min(b[i+3],bounds[i+3])-max(b[i],bounds[i])>1e-5 for i in range(3))
                and (v:=overlap(a,shape))>.01}

    def contact(a,b,dz):
        s=mgr.copy(a)
        m=core.Matrix3D.create()
        m.translation=core.Vector3D.create(0,0,dz/10)
        mgr.transform(s,m)
        return (overlap(s,b)-overlap(a,b))/abs(dz)

    plate=parts['R7 162 controller-deck'][0]
    def bore(x,y,d):
        return overlap(cylinder(x,y,DECK_Z-3.1,DECK_Z+.1,d-.01),plate)

    holes=[dict(xyMm=[x,y],diameterMm=d,filledMm3=bore(x,y,d))
           for coords,d in [(BASE_HOLES,5.5),(sum(POSTS,()),4.5)] for x,y in coords]
    old=[dict(xyMm=[x,y],filledMm3=bore(x,y,4.8)) for x in (-306,-244) for y in (-180,-158)]
    new_posts=[]
    gauges=[]
    physical={}
    cables=[]
    controls=[]
    for index,(route,group) in enumerate(zip(ROUTES,POSTS),1):
        label=f'Lead {index} '
        cable=parts[label+'Southwire 104110604 local cable envelope'][0]
        center_length=53-26.416+50*math.pi/2+18
        volume_expected=math.pi*(7.75/2)**2*center_length
        stations=[]
        for deg in range(0,91,5):
            a=math.radians(deg)
            x,y=xy(route,53+50*math.sin(a),route[3]*50*(1-math.cos(a)))
            ball=mgr.createSphere(core.Point3D.create(x/10,y/10,26.2),.10)
            stations.append(overlap(ball,cable))
        if not cable.isSolid or cable.lumps.count!=1 or abs(cable.volume*1000-volume_expected)>10 or min(stations)<4.17:
            raise ValueError('Reopened cable continuity')
        cables.append(dict(lead=index,solid=cable.isSolid,lumps=cable.lumps.count,centerLengthMm=center_length,
                           straightBeforeBendMm=26.584,bendLengthMm=50*math.pi/2,straightAfterBendMm=18.,
                           installedVolumeMm3=cable.volume*1000,analyticVolumeMm3=volume_expected,
                           arcCenterTestSphereVolumesMm3=stations,
                           endCondition='Local jacket section from lug barrel mouth to open routing outlet. Stock cut length requires the downstream connection and supplier strip depth.'))
        saddle=parts[label+'one-piece raised clamp saddle'][0]
        for j,(x,y) in enumerate(group,1):
            post=parts[label+f'tapped stand-off {j}'][0]
            new_posts.append(dict(xyMm=[x,y],deckEdgeMm=min(x+320,-x,y+310,310-y)-2.25,
                                  postFootLandMm2=contact(post,plate,-.01),
                                  shelfLandMm2=contact(post,saddle,.01),clearToNeighbor=volume(post,[label+f'tapped stand-off {j}',label+f'upper M4 x 10 shelf screw {j}',label+f'lower M4 x 10 deck screw {j}'])))
            for text,z,sgn in [('upper',256,1),('lower',DECK_Z-7,-1)]:
                gauges.append(dict(location=[x,y,z],direction=sgn,diameterMm=6.35,lengthMm=32,
                                  hits=volume(cylinder(x,y,z,z+sgn*32,6.35))))
        for v in (-11.5,11.5):
            x,y=xy(route,45,v)
            gauges.append(dict(location=[x,y,276],direction=1,diameterMm=6.35,lengthMm=32,
                        hits=volume(cylinder(x,y,276,308,6.35))))
        for name,(solid,_) in parts.items():
            if name.startswith(label) and any(n in name for n in ['saddle','clamp cap','local cable','Panduit','Pacer']):
                hits=volume(solid,[name])
                physical[name]=hits
    for x,y in [(-312.,-122.5),(-312.,-88.5),(-222.,-155.),(-192.,-155.)]:
        controls.append(dict(xyMm=[x,y],upperDriverOtherBodies=volume(cylinder(x,y,256,288,6.35))))
    separation=min(math.hypot(x-X,y-Y) for x,y in BASE_HOLES for X in (-306,-244) for Y in (-180,-158))
    if max(r['filledMm3'] for r in holes)>.01 or min(min(r['postFootLandMm2'],r['shelfLandMm2']) for r in new_posts)<55:
        raise ValueError('Deck bores or stand-off bearing failed')
    return dict(cables=cables,predecessorPostLocations=controls,deckBores=holes,inheritedDeckBores=old,posts=new_posts,serviceGauges=gauges,physicalIntersectionsMm3=physical,
                closestOldToNewPedestalCenterMm=separation,oldToNewLigamentAtBoth5p5Mm=separation-5.5,
                serviceGaugeMeaning='Tool body diameter and straight reach from screw head; test manual working clearance on a bench.',
                limits=['Thread-major envelopes overlap their matching tap bores.',
                        'The public breaker body is an outer reservation; lug and boot seating use the private registered open-shell source separately.'])

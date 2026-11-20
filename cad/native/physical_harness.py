"""Install the revised breaker deck and two passive cable-support assemblies."""
from math import pi, radians
import sys
import adsk.core as core
import adsk.fusion as fusion
from physical_bumper import ROOT, install_part
from stage_climber import box, cut, pt, rod

sys.path.insert(0,str(ROOT/'cad/assembly-readiness'))
from breaker_layout import BASE_HOLES, DECK_Z, PLATFORMS, POSTS, ROUTES, SHELF_Z, SHIFT_Y, xy

GROUP = 'R9 local breaker harness'
MGR = fusion.TemporaryBRepManager.get()


def combined(a,b,op=fusion.BooleanTypes.UnionBooleanType):
    if not MGR.booleanOperation(a,b,op):
        raise ValueError('Cable support solid operation')
    return a


def place(solid,route):
    x,y,angle,_=route
    matrix=core.Matrix3D.create()
    matrix.setToRotation(radians(angle),core.Vector3D.create(0,0,1),core.Point3D.create())
    matrix.translation=core.Vector3D.create(x/10,y/10,0)
    if not MGR.transform(solid,matrix):
        raise ValueError('Cable support registration')
    return solid


def cylinder(x,y,z0,z1,d):
    return rod((x,y,z0),(x,y,z1),d)


def socket_screw(x,y,seat,sign,length):
    head=cylinder(x,y,seat,seat+sign*4.,7.)
    recess=cylinder(x,y,seat+sign*1.5,seat+sign*4.2,3.464101615)
    # Three strips leave the six 3 mm across-flats faces of a hex recess.
    for angle in (0,60,120):
        mask=box(-1.5,-4,min(seat,seat+sign*5)-1,1.5,4,max(seat,seat+sign*5)+1)
        combined(recess,place(mask,(x,y,angle,0)),fusion.BooleanTypes.IntersectionBooleanType)
    cut(head,recess)
    return combined(head,cylinder(x,y,seat,seat-sign*length,4.))


def redrill(comp):
    body=comp.bRepBodies.itemByName('R7 162 controller-deck')
    device=comp.bRepBodies.itemByName('R7 149 main-breaker')
    if not body or not device:
        raise ValueError('Breaker or controller deck missing')
    objects=core.ObjectCollection.create()
    objects.add(device)
    move=comp.features.moveFeatures
    req=move.createInput2(objects)
    value=core.ValueInput.createByReal
    req.defineAsTranslateXYZ(value(0),value(SHIFT_Y/10),value(0),False)
    move.add(req).name='Shift breaker 12 mm toward robot center'
    for title,holes,diam in [('Shifted pedestal mounting pattern',BASE_HOLES,5.5),
                             ('Cable stand-off drill pattern',sum(POSTS,()),4.5)]:
        sk=comp.sketches.add(comp.xYConstructionPlane)
        sk.name=title+' stations'
        for x,y in holes:
            sk.sketchCurves.sketchCircles.addByCenterRadius(pt(x,y,0),diam/20)
        profiles=core.ObjectCollection.create()
        for profile in sk.profiles:
            profiles.add(profile)
        if profiles.count!=len(holes):
            raise ValueError('Unexpected cable support hole profile count')
        cut_req=comp.features.extrudeFeatures.createInput(profiles,fusion.FeatureOperations.CutFeatureOperation)
        cut_req.setDistanceExtent(False,core.ValueInput.createByString('206 mm'))
        cut_req.participantBodies=[body]
        comp.features.extrudeFeatures.add(cut_req).name=title


def cable(design,route):
    sign=route[3]
    scratch=design.rootComponent.occurrences.addNewComponent(core.Matrix3D.create())
    try:
        comp=scratch.component
        sketch=comp.sketches.add(comp.yZConstructionPlane)
        def in_plane(y,z):
            return sketch.modelToSketchSpace(pt(0,y,z))
        sketch.sketchCurves.sketchCircles.addByCenterRadius(in_plane(0,262),.3875)
        axis=sketch.sketchCurves.sketchLines.addByTwoPoints(in_plane(50,240),in_plane(50,285))
        axis.isConstruction=True
        maker=comp.features.revolveFeatures
        req=maker.createInput(sketch.profiles.item(0),axis,fusion.FeatureOperations.NewBodyFeatureOperation)
        req.setAngleExtent(False,core.ValueInput.createByString('90 deg'))
        elbow=MGR.copy(maker.add(req).bodies.item(0))
    finally:
        scratch.deleteMe()
    m=core.Matrix3D.create()
    if sign<0:
        m.setToRotation(pi,core.Vector3D.create(1,0,0),pt(0,0,262))
        MGR.transform(elbow,m)
        m=core.Matrix3D.create()
    m.translation=core.Vector3D.create(5.3,0,0)
    MGR.transform(elbow,m)
    arc_volume=pi*(7.75/2)**2*pi*25/1000
    if not elbow.isSolid or abs(elbow.volume-arc_volume)>.008:
        raise ValueError('Quarter-round bend volume '+str(elbow.volume))
    from physical_bumper import dimensions
    before_bounds=dimensions(elbow)
    combined(elbow,rod((26.416,0,262),(53,0,262),7.75))
    combined(elbow,rod((103,sign*50,262),(103,sign*68,262),7.75))
    analytic=pi*(7.75/2)**2*(53-26.416+pi*25+18)/1000
    if not elbow.isSolid or elbow.lumps.count!=1 or abs(elbow.volume-analytic)/analytic>.002:
        raise ValueError('Incomplete tangent cable '+str((elbow.volume,elbow.lumps.count,analytic,before_bounds,dimensions(elbow))))
    return place(elbow,route)


def install(d,chassis,lib):
    redrill(chassis.component)
    color=d.appearances.itemByName('R7 structure')
    al=lib.itemByName('Aluminum 6061')
    steel=lib.itemByName('Steel, Alloy')
    copper=lib.itemByName('Copper') or lib.itemByName('Copper, Pure')
    if not copper:
        raise ValueError('Copper material is required for nominal lug geometry')
    def proxy(name):
        base=lib.itemByName('Nylon 6/6')
        if not base:raise ValueError('Missing reference polymer for named packaging proxy')
        return d.materials.addByCopy(base,'R9 '+name+' envelope only - receiving mass pending')
    boot_proxy=proxy('Pacer flexible boot')
    liner_proxy=proxy('EPDM installed liner')
    cable_proxy=proxy('Southwire composite cable')
    items=[]
    for index,(route,posts,plate) in enumerate(zip(ROUTES,POSTS,PLATFORMS),1):
        def add(label,s,material,local=True):
            items.append((f'Lead {index} {label}',place(s,route) if local else s,material,color))
        # The supplier catalog fixes the tongue stock and axial sizes. Barrel
        # and outer boot shapes are inspection envelopes for the received parts.
        seat=253.75148403491677+5.461
        tongue=combined(cylinder(0,0,seat,seat+2.032,12.192),box(0,-6.096,seat,16,6.096,seat+2.032))
        cut(tongue,cylinder(0,0,seat-.2,seat+2.3,6.86))
        barrel=rod((14.224,0,262),(26.416,0,262),10.2)
        cut(barrel,rod((14.1,0,262),(26.6,0,262),6.1))
        add('Panduit LCAX6-14-L lug envelope',combined(tongue,barrel),copper)
        # Flexible cover silhouette. Exterior acceptance is checked separately
        # by a surrounding continuous diameter-30 keepout.
        boot=cut(cylinder(0,0,seat,seat+26.67,25),cylinder(0,0,seat-.1,seat+23.5,20.574))
        neck=MGR.createCylinderOrCone(pt(5,0,262),.84,pt(27,0,262),.675)
        combined(boot,neck)
        cut(boot,rod((0,0,262),(17,0,262),14.30),
            MGR.createCylinderOrCone(pt(17,0,262),.715,pt(21,0,262),.535),
            rod((21,0,262),(28,0,262),10.7))
        add('Pacer TBCA2-R boot envelope',boot,boot_proxy)
        add('Southwire 104110604 local cable envelope',cable(d,route),cable_proxy,False)
        bore=rod((37,0,262),(53,0,262),11.)
        lower=cut(box(38,-17,252,52,17,261.85),bore)
        cap=cut(box(38,-17,262.15,52,17,272),bore)
        for side in (-11.5,11.5):
            cut(lower,cylinder(45,side,251,263,3.3))
            cut(cap,cylinder(45,side,261,273,4.5))
            shim=cut(box(38,side-4.5,261.85,52,side+4.5,262.15),cylinder(45,side,261.5,262.5,4.5))
            add(f'0.30 gap shim {side}',shim,lib.itemByName('Stainless Steel') or steel)
            add(f'M4 x 16 clamp screw {side}',socket_screw(45,side,272,1,16),steel)
        add('split clamp cap',cap,al)
        liner=cut(rod((38,0,262),(52,0,262),11),rod((37,0,262),(53,0,262),7.75))
        for label,z0,z1 in [('upper',262,269),('lower',255,262)]:
            half=combined(MGR.copy(liner),box(37,-6,z0,53,6,z1),fusion.BooleanTypes.IntersectionBooleanType)
            add(label+' EPDM liner envelope',half,liner_proxy)
        x0,y0,x1,y1=plate
        base=box(x0,y0,SHELF_Z,x1,y1,252.)
        for j,(x,y) in enumerate(posts,1):
            cut(base,cylinder(x,y,SHELF_Z-.2,252.2,4.5))
            post=cylinder(x,y,DECK_Z,SHELF_Z,10)
            cut(post,cylinder(x,y,DECK_Z-.1,DECK_Z+12,3.3),cylinder(x,y,SHELF_Z-12,SHELF_Z+.1,3.3))
            add(f'tapped stand-off {j}',post,al,False)
            add(f'upper M4 x 10 shelf screw {j}',socket_screw(x,y,252,1,10),steel,False)
            add(f'lower M4 x 10 deck screw {j}',socket_screw(x,y,DECK_Z-3,-1,10),steel,False)
        add('one-piece raised clamp saddle',combined(base,place(lower,route)),al,False)
    install_part(d,chassis,GROUP,items)

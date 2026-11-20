"""Rework the four mast-to-frame attachment stations and prove the assembly order."""
import math

import adsk.core as core
import adsk.fusion as fusion
from stage_climber import box, cut, pt, rod, side_rails
from physical_bumper import dimensions, registered_solid

FIXED='R8 fixed climb mast'


def hexagonal(x,y,z0,z1,af):
    mgr=fusion.TemporaryBRepManager.get()
    shape=rod((x,y,z0),(x,y,z1),af*1.16)
    for a in (0,60,120):
        slab=box(x-af/2,y-af,z0-1,x+af/2,y+af,z1+1)
        matrix=core.Matrix3D.create()
        matrix.setToRotation(math.radians(a),core.Vector3D.create(0,0,1),pt(x,y,z0))
        if not mgr.transform(slab,matrix) or not mgr.booleanOperation(shape,slab,fusion.BooleanTypes.IntersectionBooleanType):
            raise ValueError('Hex flats')
    return shape


def holes(component,bodies):
    sketch=component.sketches.add(component.xYConstructionPlane)
    sketch.name='Four 13.1 mm flush sleeve installation bores'
    for y in (-329,329):
        for x in (23,53):
            c=sketch.sketchCurves.sketchCircles.addByCenterRadius(pt(x,y,0),.655)
            c.isFixed=True
    profiles=core.ObjectCollection.create()
    for profile in sketch.profiles: profiles.add(profile)
    if profiles.count!=4: raise ValueError('Sleeve bore count')
    ext=component.features.extrudeFeatures
    request=ext.createInput(profiles,fusion.FeatureOperations.CutFeatureOperation)
    request.setDistanceExtent(False,core.ValueInput.createByString('161 mm'))
    request.participantBodies=bodies
    ext.add(request).name='Drill both tube walls for weld-in sleeves'


def install(d,chassis,lib):
    from physical_bumper import install_part
    mgr=fusion.TemporaryBRepManager.get()
    mast=next(o for o in d.rootComponent.occurrences if o.component.name==FIXED)
    comp=mast.component
    outdated=[b for b in comp.bRepBodies if b.name.startswith(('Rail mounting fastener envelope',
        'Rail crush sleeve','Crossmember crush sleeve','Captive nut envelope'))]
    if len(outdated)!=16:raise ValueError('Mast fastening inventory')
    for b in outdated:
        comp.features.removeFeatures.add(b).name='Replace '+b.name
    holes(chassis.component,side_rails(chassis.component))
    holes(comp,[comp.bRepBodies.itemByName('Crossmember with four rail mounting holes')])
    items=[]
    metal=lib.itemByName('Steel, Alloy')
    al=lib.itemByName('Aluminum 6061')
    color=d.appearances.itemByName('R7 structure')
    for y in (-329,329):
        for x in (23,53):
            tag=f'{x} {y}'
            for label,z0,z1 in [('Rail',69.85,120.65),('Crossmember',120.65,159.)]:
                sleeve=cut(rod((x,y,z0),(x,y,z1),13.),rod((x,y,z0-1),(x,y,z1+1),8.5))
                items.append((label+' flush weld sleeve '+tag,sleeve,al,color))
            bolt=rod((x,y,59.),(x,y,159.),8)
            head=rod((x,y,159.),(x,y,167.),13)
            if not mgr.booleanOperation(bolt,head,fusion.BooleanTypes.UnionBooleanType):
                raise ValueError('Socket head union')
            cut(bolt,hexagonal(x,y,163.,168.,6.))
            # A washer bridges the sleeve/wall joint and spreads nut seating load.
            washer=cut(rod((x,y,68.25),(x,y,69.85),16),rod((x,y,67.),(x,y,71.),8.4))
            nut=cut(hexagonal(x,y,61.75,68.25,13),rod((x,y,61.),(x,y,69.),6.8))
            items.extend([('MISUMI CB8-100 '+tag,bolt,metal,color),('M8 plain washer '+tag,washer,metal,color),
                          ('Accu HPN-M8-C8-Z '+tag,nut,metal,color)])
    install_part(d,chassis,'R9 mast attachment',items)


def inspect(d):
    mgr=fusion.TemporaryBRepManager.get()
    parts={}
    for o in d.rootComponent.allOccurrences:
        for b in o.bRepBodies:
            solid=registered_solid(o,b)
            parts[b.name]=(solid,dimensions(solid))
    def common(a,b):
        t=mgr.copy(a)
        if not mgr.booleanOperation(t,b,fusion.BooleanTypes.IntersectionBooleanType):
            raise ValueError('Intersection failed')
        return t.volume*1000 if t.isValid and t.lumps.count else 0.
    def seat(a,b,dz):
        m=core.Matrix3D.create()
        m.translation=core.Vector3D.create(0,0,dz/10)
        shifted=mgr.copy(a)
        if not mgr.transform(shifted,m):raise ValueError('Mast bearing shift')
        return (common(shifted,b)-common(a,b))/abs(dz)
    def collisions(shape, skip):
        bounds=dimensions(shape)
        return {name:round(v,4) for name,(part,bb) in parts.items() if name not in skip
                and all(min(bounds[i+3],bb[i+3])-max(bounds[i],bb[i])>1e-5 for i in range(3))
                and (v:=common(shape,part))>.01}
    beam=parts['Crossmember with four rail mounting holes'][0]
    results=[]
    for y in (-329,329):
        for x in (23,53):
            tag=f'{x} {y}'
            screw='MISUMI CB8-100 '+tag
            lower='Rail flush weld sleeve '+tag
            upper='Crossmember flush weld sleeve '+tag
            nut='Accu HPN-M8-C8-Z '+tag
            # Reintroduce the historic 0.2 mm head error against an intact
            # 8.5 mm clearance opening to prove sensitivity of the overlap query.
            historic=cut(box(x-10,y-10,156.,x+10,y+10,159.),
                         rod((x,y,155.),(x,y,160.),8.5))
            control=common(rod((x,y,158.8),(x,y,165.),14.),historic)
            repaired=common(parts[screw][0],beam)+common(parts[screw][0],parts[upper][0])
            if not 19.4<control<19.6 or repaired>.01:raise ValueError('Head seating control')
            # Staged, isolated loose tubes. Drop each sleeve through its
            # finished wall pair before flush welds, coating or final bolting.
            beam_drop=common(rod((x,y,120.65),(x,y,197.35),13.),beam)
            rail=next(val[0] for name,val in parts.items() if name.endswith('Tab-mounted rectangular rail')
                    and val[1][1]<y<val[1][4] and val[1][0]<x<val[1][3])
            rail_drop=common(rod((x,y,69.85),(x,y,171.45),13.),rail)
            washer=parts['M8 plain washer '+tag][0]
            cap_land=seat(parts[screw][0],parts[upper][0],-.01)
            nut_land=seat(parts[nut][0],washer,.01)
            washer_rail=seat(washer,rail,.01)
            washer_sleeve=seat(washer,parts[lower][0],.01)
            if min(cap_land,nut_land,washer_rail,washer_sleeve)<1:raise ValueError('Missing mast bearing face')
            results.append(dict(capToUpperSleeveMm2=cap_land,nutToWasherMm2=nut_land,
                washerToRailMm2=washer_rail,washerToLowerSleeveMm2=washer_sleeve,stationMm=[x,y],negativeControlMm3=control,headOverlapMm3=repaired,
                looseTubeSleeveInsertionMm3=[beam_drop,rail_drop],
                threadEngagementMm=6.5,tipPastNutMm=2.75,
                topHexToolHits=collisions(rod((x,y,167),(x,y,217),12),{screw}),
                bottomSocketHits=collisions(rod((x,y,19),(x,y,68.25),19),{nut,screw})))
    return results

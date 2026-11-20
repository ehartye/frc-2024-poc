"""Check the installed boards, screw access and full downward service stroke in R9."""
from physical_front import FRONT, REAR
from pathlib import Path
import runpy

import adsk.core as core
import adsk.fusion as fusion

from physical_bumper import CORE, COVER, dimensions, make_box, registered_solid


def inspect(design):
    spec = runpy.run_path(str(Path(__file__).with_name('swerve_bumper.py')))
    manager = fusion.TemporaryBRepManager.get()
    p = core.Point3D.create
    parts = {}
    for occ in design.rootComponent.allOccurrences:
        for body in occ.bRepBodies:
            solid = registered_solid(occ, body)
            parts[body.name] = (solid, dimensions(solid), occ.component.name)
    if len(parts) != sum(o.bRepBodies.count for o in design.rootComponent.allOccurrences):
        raise ValueError('Expected unique body names')
    def common(a, b):
        result = manager.copy(a)
        if not manager.booleanOperation(result, b, fusion.BooleanTypes.IntersectionBooleanType):
            raise ValueError('Intersection failed')
        return result.volume*1000 if result.isValid and result.lumps.count else 0.
    def contact(a, b, n):
        moved = manager.copy(a)
        matrix = core.Matrix3D.create()
        matrix.translation = core.Vector3D.create(n[0]*.001,n[1]*.001,0)
        if not manager.transform(moved,matrix):
            raise ValueError('Contact probe failed')
        return (common(moved,b)-common(a,b))/.01
    def hits(shape, skip):
        bounds = dimensions(shape)
        found = []
        for name,(solid,bb,_) in parts.items():
            if name in skip or any(bounds[i+3]<=bb[i]+1e-5 or bb[i+3]<=bounds[i]+1e-5 for i in range(3)):
                continue
            volume = common(shape,solid)
            if volume>.01:
                found.append(dict(body=name,intersectionMm3=round(volume,3)))
        return found
    indexed = {int(name.split()[1]):name for name in parts if name.startswith('R7 ')}
    remove = set()
    for i in range(593,612):
        if i not in (595,596,597):
            remove.add(indexed[i])
    for name,(_,_,comp) in parts.items():
        if comp in (CORE,COVER,'R9 bumper washers') or name in {indexed[i] for i in range(487,503)}:
            remove.add(name)
    seats = []
    intersections = []
    for side,rail_id,bolt_ids in [('Front',592,(593,598)),('Rear',595,(600,602)),
                                   ('Left',596,(604,606)),('Right',597,(608,610))]:
        n,radius,thick,_ = spec['FACES'][side]
        if side=='Front':
            radius=FRONT
        if side=='Rear':
            radius,thick=REAR,4.7625
        rail = parts[indexed[rail_id]][0]
        board = parts[side+' drilled bumper wood'][0]
        axis=0 if n[0] else 1
        for station,idx in zip(spec['STATIONS'][side],bolt_ids):
            def location(depth):
                return [n[0]*(radius+depth)-n[1]*station,n[1]*(radius+depth)+n[0]*station,145.]
            def cylinder(start,finish,radius):
                return manager.createCylinderOrCone(p(*(v/10 for v in start)),radius/10,
                    p(*(v/10 for v in finish)),radius/10)
            bolt,bolt_bb,_=parts[indexed[idx]]
            nut,nut_bb,_=parts[indexed[idx+1]]
            support=rail
            skip={indexed[idx]}
            washer_seat=None
            if side in ('Left','Right'):
                name=f'Side M6 washer {round(location(0)[0])} {n[1]}'
                support=parts[name][0]
                skip.add(name)
                washer_seat=contact(support,rail,n)
                if not 78<washer_seat<81: raise ValueError(f'Washer seat {washer_seat}')
            head_seat=contact(bolt,support,n)
            nut_seat=contact(nut,board,(-n[0],-n[1]))
            tip=max(bolt_bb[i]*n[axis] for i in (axis,axis+3))
            entry,exit=sorted(nut_bb[i]*n[axis] for i in (axis,axis+3))
            if not (43<head_seat<47 and 157<nut_seat<160 and entry+6<=tip<=exit-.5):
                raise ValueError(f'Invalid {side} {station} attachment')
            seat_depth=-thick-(1.6 if washer_seat else 0)
            gauge=cylinder(location(seat_depth-48),location(seat_depth),8)
            collisions=hits(gauge,skip)
            seats.append(dict(face=side,stationMm=station,headSeatMm2=head_seat,washerSeatMm2=washer_seat,nutSeatMm2=nut_seat,
                nominalEngagementMm=tip-entry,tipToOuterFaceMm=exit-tip,toolHits=collisions))
    # Continuous translational bounds conservatively enclose each straight stroke.
    # Partition the cover into four strips; a single bounding box fills the chassis.
    for name in sorted(remove):
        if name.startswith('Side M6 washer') or name not in parts or name in {indexed[i] for i in (593,598,600,602,604,606,608,610)}:
            continue
        geom,bb,comp=parts[name]
        boxes=[bb]
        if comp==COVER and 'skin' in name:
            x,y,z,X,Y,Z=bb
            boxes=[[x,y,z,-REAR,Y,Z],[FRONT,y,z,X,Y,Z],
                   [-REAR,y,z,FRONT,-355.6,Z],[-REAR,355.6,z,FRONT,Y,Z]]
        for box in boxes:
            lo=box[:3]
            lo[2]-=200
            intersections.extend(dict(moving=name,**hit) for hit in hits(make_box(lo,box[3:]),remove))
    return dict(checkedBodies=len(parts),seats=seats,toolDiameterMm=16,toolLengthMm=48,
        downwardStrokeMm=200,removalHits=intersections,minimumVolumeMm3=.01,
        method='Exact BRep seat perturbation and tool gauges; continuous conservative radial boxes for the detached assembly.',
        topOfDetachedCoverMm=parts['Sewn nylon skin, nominal envelope'][1][5]-200)

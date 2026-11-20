"""Move the outer bumper rail and add an inner crossmember and four weld spacers."""
import math

import adsk.core as core
import adsk.fusion as fusion

from stage_climber import box, cut, rod

SHIFT = 60.0
REAR_SHIFT = 5.8745
REAR = 359.537-REAR_SHIFT
FRONT = 394.462 + SHIFT
SIDE_LENGTH = 792.099 + SHIFT-REAR_SHIFT


def translate(shape, dx):
    matrix = core.Matrix3D.create()
    matrix.translation = core.Vector3D.create(dx/10,0,0)
    if not fusion.TemporaryBRepManager.get().transform(shape,matrix):
        raise ValueError('Front registration failed')
    return shape


def reshape_core(name, shape):
    if name=='Front drilled bumper wood' or name.startswith('fixed bumper end noodle 1 '):
        return translate(shape,SHIFT)
    if name=='Rear drilled bumper wood' or name.startswith('fixed bumper end noodle -1 '):
        return translate(shape,REAR_SHIFT)
    if name.startswith('fixed bumper side noodle'):
        bb = shape.preciseBoundingBox
        center=[(a+b)*5 for a,b in zip(bb.minPoint.asArray(),bb.maxPoint.asArray())]
        extension=rod((413.512,*center[1:]),(413.512+SHIFT,*center[1:]),63.5)
    elif name in ('Left drilled bumper wood','Right drilled bumper wood'):
        low,high=(355.6,374.65) if name.startswith('Left') else (-374.65,-355.6)
        extension=box(413.512,low,58.5,413.512+SHIFT,high,185.5)
    else:
        return shape
    if not fusion.TemporaryBRepManager.get().booleanOperation(shape,extension,fusion.BooleanTypes.UnionBooleanType):
        raise ValueError('Side extension '+name)
    cut(shape,box(-379.,-441.,55.,-378.587+REAR_SHIFT,441.,188.))
    return shape


def added_volume_cm3(name):
    if name.startswith('fixed bumper side noodle'):
        return (SHIFT-REAR_SHIFT)*math.pi*31.75**2/1000
    if name in ('Left drilled bumper wood','Right drilled bumper wood'):
        return (SHIFT-REAR_SHIFT)*19.05*127/1000
    return 0.


def install(d,chassis,lib):
    from physical_bumper import install_part
    comp=chassis.component
    for dx,dy,keys in [(REAR_SHIFT,0,[491,492,493,494,601,603]),(0,-1.6,[604,606]),(0,1.6,[608,610]),(SHIFT,0,[487,488,489,490,592,593,594,598,599]),
                    ((SHIFT+REAR_SHIFT)/2,0,list(range(495,503)))]:
        selected=core.ObjectCollection.create()
        for b in comp.bRepBodies:
            if b.name.startswith('R7 ') and int(b.name.split()[1]) in keys:
                selected.add(b)
        if selected.count!=len(keys):raise ValueError('Front hardware/markings inventory')
        move=comp.features.moveFeatures
        request=move.createInput2(selected)
        val=core.ValueInput.createByReal
        if not request.defineAsTranslateXYZ(val(dx/10),val(dy/10),val(0),False):
            raise ValueError('Displacement definition failed')
        move.add(request).name='Relocate nose hardware and cover graphics'
    def trim(ids, xmin, xmax, title):
        sketch=comp.sketches.add(comp.xYConstructionPlane)
        sketch.name=title+' profile'
        sketch.sketchCurves.sketchLines.addTwoPointRectangle(core.Point3D.create(xmin/10,-36,0),
                                                           core.Point3D.create(xmax/10,36,0))
        req=comp.features.extrudeFeatures.createInput(sketch.profiles.item(0),fusion.FeatureOperations.CutFeatureOperation)
        req.setDistanceExtent(False,core.ValueInput.createByString('166 mm'))
        req.participantBodies=[b for b in comp.bRepBodies if b.name.startswith('R7 ') and int(b.name.split()[1]) in ids]
        comp.features.extrudeFeatures.add(req).name=title
    trim([595],-365.,-REAR,'Rear backing plate to 4.7625 mm')
    trim([600,602],-375.,-368.9,'Rear M6 shoulder-to-tip length 20 mm')
    stock=lib.itemByName('Aluminum 6061')
    color=d.appearances.itemByName('R7 structure')
    # Transverse member spans the original side rail welds. Rectangular struts
    # bridge the two crossmember faces; both outer struts back the side panels.
    outer=box(356.362,-355.6,74.6,394.462,355.6,125.4)
    anchor=cut(outer,box(359.537,-356,77.775,391.287,356,122.225))
    items=[('Inner crossmember 711.2 mm',anchor,stock,color)]
    for y in (-342.9,-160.,160.,342.9):
        support=cut(box(394.462,y-12.7,74.6,FRONT-38.1,y+12.7,125.4),
                    box(394.0,y-9.525,77.775,FRONT-37.5,y+9.525,122.225))
        items.append((f'Nose spacer at {y} mm',support,stock,color))
    install_part(d,chassis,'R9 front frame',items)
    washers=[]
    for normal in (-1,1):
        for x in (-175,326):
            a,b=(normal*t for t in (347.3,348.9))
            shape=cut(rod((x,a,145),(x,b,145),12),rod((x,a-normal*.2,145),(x,b+normal*.2,145),6.4))
            washers.append((f'Side M6 washer {x} {normal}',shape,lib.itemByName('Steel, Alloy'),color))
    install_part(d,chassis,'R9 bumper washers',washers)

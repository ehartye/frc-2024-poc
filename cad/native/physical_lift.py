"""Install the R9 central lift drive and its upper shaft supports in robot coordinates."""
import math
import adsk.core as core
import adsk.fusion as fusion

from stage_climber import box, cut, rod
from physical_bumper import install_part

ASSEMBLY = 'R9 upper lift drive'
X, Z = 60., 898.5
ANGLE = -math.pi/4
MANAGER = fusion.TemporaryBRepManager.get()


def rotate(shape, angle):
    transform = core.Matrix3D.create()
    transform.setToRotation(angle,core.Vector3D.create(0,1,0),core.Point3D.create())
    if not MANAGER.transform(shape,transform):
        raise ValueError('Lift orientation')
    return shape


def position(shape):
    rotate(shape, ANGLE)
    transform = core.Matrix3D.create()
    transform.translation = core.Vector3D.create(X/10,0,Z/10)
    if not MANAGER.transform(shape,transform):
        raise ValueError('Lift shaft position')
    return shape


def hex_bar(y0,y1,across,diameter):
    shape = rod((0,y0,0),(0,y1,0),diameter)
    for angle in (0,math.pi/3,-math.pi/3):
        slab = box(-across/2,y0-1,-diameter,across/2,y1+1,diameter)
        if not MANAGER.booleanOperation(shape,rotate(slab,angle),fusion.BooleanTypes.IntersectionBooleanType):
            raise ValueError('Rounded shaft flat')
    return shape


def along_shaft(a,b,d):
    return rod((X,a,Z),(X,b,Z),d)


def install(d, chassis, lib):
    comp = chassis.component
    retired = {}
    for body in comp.bRepBodies:
        if body.name.startswith('R7 '):
            key = int(body.name.split()[1])
            if key in (253,266,276,277,278): retired[key] = body
    if len(retired)!=5: raise ValueError('Unexpected lift predecessor')
    for key,body in retired.items():
        comp.features.removeFeatures.add(body).name=f'Replace R7 lift article {key}'

    structure=d.appearances.itemByName('R7 structure')
    drive=d.appearances.itemByName('R7 drive') or structure
    items=[]
    def add(name,shape,material,appearance=structure):
        items.append((name,shape,lib.itemByName(material),appearance))
    def thru(y0,y1):
        return position(hex_bar(y0,y1,12.80,15))
    def union(a,b):
        if not MANAGER.booleanOperation(a,b,fusion.BooleanTypes.UnionBooleanType):
            raise ValueError('Lift union')
        return a
    # Local +X follows the input axis, +Y is the output axis. The square
    # case is a conservative external envelope with supplier face-mount bores.
    case = box(-31.75,-25.4,-31.75,31.75,25.4,31.75)
    for x,z in ((-25.4,0),(25.4,0),(0,-25.4),(0,25.4)):
        cut(case,rod((x,-26,z),(x,26,z),4.826))
    cut(case,hex_bar(-26,26,12.70,15))
    add('REV-21-2120 through-bore gearbox clearance model',position(case),'Aluminum 6061',drive)
    add('Lift MAXPlanetary stack reservation',position(box(-95.25,-25.4,-30.95,-31.75,25.4,30.95)),'Steel',drive)
    add('Lift NEO motor reservation',position(rod((-153.6,0,0),(-95.25,0,0),60)),'Steel',drive)
    shaft=position(hex_bar(-297.75,297.75,12.7,13.75))
    cut(shaft,along_shaft(-299,299,4.0))
    add('REV-21-1135 cut 595.5 mm rounded hex shaft',shaft,'Aluminum 7075',drive)
    for side in (-1,1):
        # The two plates locate the gearbox on the original transverse portal.
        y0,y1=sorted([25.4*side,31.75*side])
        plate=box(13.5,y0,854.,88.9,y1,954.)
        cut(plate,along_shaft(y0-1,y1+1,32))
        for i,(dx,dz) in enumerate(((-25.4,0),(25.4,0),(0,-25.4),(0,25.4))):
            px=X+math.cos(ANGLE)*dx+math.sin(ANGLE)*dz
            pz=Z-math.sin(ANGLE)*dx+math.cos(ANGLE)*dz
            cut(plate,rod((px,y0-1,pz),(px,y1+1,pz),5.1))
            a,b,c=(v*side for v in (36.576,31.75,20.6375))
            shank=rod((px,b,pz),(px,c,pz),4.826)
            head=rod((px,a,pz),(px,b,pz),7.925)
            add(f'Gearcase mount 10-32 x 7-16 screw {side} {i+1}',union(head,shank),'Steel, Alloy',drive)
        add(f'Central gearbox weld tab {side}',plate,'Aluminum 6061')

        # The journal spans a 1/4-in plate. Flange faces the outer sprocket.
        inner,outer,lip=(v*side for v in (274.15,280.5,282.0875))
        a,b=sorted([inner,outer])
        seat=cut(box(39,a,877.5,88.9,b,954.),along_shaft(a-1,b+1,28.605))
        add(f'Outer bearing weld tab {side}',seat,'Aluminum 6061')
        bearing=union(along_shaft(inner,outer,28.575),along_shaft(outer,lip,31.115))
        cut(bearing,along_shaft(273*side,283*side,13.75))
        add(f'REV-21-1916 flanged bearing reservation {side}',bearing,'Steel',drive)

        # Keep the original 16-tooth pitch plane at +/-290 mm. The teeth and
        # internal bearing races remain supplier-controlled.
        h0,h1=(v*side for v in (285.25,294.75))
        sprocket=union(along_shaft(h0,h1,19.1),along_shaft(288.35*side,291.65*side,35.9))
        cut(sprocket,thru(min(h0,h1)-1,max(h0,h1)+1))
        add(f'REV-21-2016 upper lift sprocket reservation {side}',sprocket,'Steel',drive)
        for tag,start,end in [('inner',282.2875,285.25),('outer',294.75,297.75)]:
            spacer=cut(along_shaft(side*start,side*end,16),along_shaft(side*(start-.1),side*(end+.1),13.85))
            add(f'Upper shaft turned {tag} spacer {side}',spacer,'Aluminum 6061')
        washer=cut(along_shaft(297.75*side,298.75*side,18),along_shaft(297*side,300*side,5.2))
        add(f'Upper shaft end washer {side}',washer,'Steel, Alloy')
        screw=union(along_shaft(298.75*side,289.225*side,4.826),along_shaft(298.75*side,303.576*side,7.925))
        add(f'Upper shaft tapped end screw {side}',screw,'Steel, Alloy')
    install_part(d,chassis,ASSEMBLY,items)

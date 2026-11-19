"""Install the project-defined 10-32 socket-head and locknut envelopes in Fusion."""
import math

import adsk.core
import adsk.fusion


def install(root, frame):
    point, placement = frame["point"], frame["placement"]
    extrude, named_circle = frame["extrude"], frame["named_circle"]
    kinds = adsk.fusion.FeatureOperations
    def hexagon(sketch, flats):
        radius = flats / math.sqrt(3)
        vertices = [point(radius*math.cos(math.pi*i/3),radius*math.sin(math.pi*i/3),0) for i in range(6)]
        for i in range(6):
            sketch.sketchCurves.sketchLines.addByTwoPoints(vertices[i],vertices[(i+1)%6])

    def screw(component):
        sketch = component.sketches.add(component.xYConstructionPlane)
        sketch.name = "10-32 nominal shank"
        named_circle(sketch,point(0,0,0),4.826,"4.826 mm")
        extrude(component,sketch.profiles.item(0),"-69.85 mm",kinds.NewBodyFeatureOperation,"2.75 inch under-head length")
        head = component.sketches.add(component.xYConstructionPlane)
        named_circle(head,point(0,0,0),7.9248,"7.9248 mm")
        extrude(component,head.profiles.item(0),"4.826 mm",kinds.JoinFeatureOperation,"Socket head")
        plane_input = component.constructionPlanes.createInput()
        plane_input.setByOffset(component.xYConstructionPlane,adsk.core.ValueInput.createByString("4.826 mm"))
        socket = component.sketches.add(component.constructionPlanes.add(plane_input))
        hexagon(socket,3.96875)
        extrude(component,socket.profiles.item(0),"-2.326 mm",kinds.CutFeatureOperation,"Hex key socket")
        component.bRepBodies.item(0).name = "10-32 bolt envelope"

    def nut(component):
        sketch = component.sketches.add(component.xYConstructionPlane)
        hexagon(sketch,9.525)
        named_circle(sketch,point(0,0,0),4.826,"4.826 mm")
        annulus = next(p for p in sketch.profiles if p.profileLoops.count==2)
        extrude(component,annulus,"-6.3246 mm",kinds.NewBodyFeatureOperation,"Locknut envelope")
        component.bRepBodies.item(0).name = "10-32 locknut envelope"

    masters = {}
    for x,y,rear in ((0,329.85,False),(0,-329.85,False),(-329.85,0,True)):
        for end in (-228.25,228.25):
            for offset in (-12.7,12.7):
                cx,cy = (x-offset,end) if rear else (end,y+offset)
                for name,z,build in (("10-32 x 2.75 socket head",124.3125,screw),("10-32 locknut",65.5637,nut)):
                    transform = placement(cx,cy,z)
                    if name in masters:
                        root.occurrences.addExistingComponent(masters[name],transform)
                    else:
                        component = root.occurrences.addNewComponent(transform).component
                        component.name = name
                        build(component)
                        masters[name] = component

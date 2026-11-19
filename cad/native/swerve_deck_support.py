"""Author the welded lower members supporting the two retained electronics beams."""
import adsk.core
import adsk.fusion


def install(design, frame, root=None):
    root = root or design.rootComponent
    point, placement = frame["point"], frame["placement"]
    rectangle, extrude = frame["section_rectangle"], frame["extrude"]
    ops = adsk.fusion.FeatureOperations
    params = {"deckCrossWidth":25.4,"deckCrossLength":621.6,"deckRunnerLength":158.4,
              "deckCapThickness":4.0,"deckSocketBore":17.0,"deckRiserHeight":56.065}
    for name, value in params.items():
        design.userParameters.add(name,adsk.core.ValueInput.createByString(f"{value} mm"),"mm","Aft deck frame support")
    def tube(component, length):
        sketch = component.sketches.add(component.xYConstructionPlane)
        sketch.name = "Rectangular support tube"
        rectangle(sketch,25.4,50.8,"deckCrossWidth","frameRailHeight")
        rectangle(sketch,19.05,44.45,"deckCrossWidth - 2 * frameWall","frameRailHeight - 2 * frameWall")
        ring = next(p for p in sketch.profiles if p.profileLoops.count==2)
        extrude(component,ring,length,ops.NewBodyFeatureOperation,"Saw tube to finished length",symmetric=True)
    # Locally Z is the tube length. Runners butt against the crossmember
    # and rear rail. The crossmember butts against both side rails.
    cross = root.occurrences.addNewComponent(placement(-139.7,0,95.25,rail="rear")).component
    cross.name = "Deck support - transverse member"
    tube(cross,"deckCrossLength")
    runner = None
    for y in (-80,80):
        loc = placement(-231.6,y,95.25,rail="side")
        if runner is None:
            runner = root.occurrences.addNewComponent(loc).component
            runner.name = "Deck support - rear stringer"
            tube(runner,"deckRunnerLength")
        else:
            root.occurrences.addExistingComponent(runner,loc)
    # The deck's original through-bolts and locknuts remain accessible
    # between the two webs. The cap bears against the retained beam.
    tower = None
    for x in (-266.7,-139.7):
        for y in (-80,80):
            loc = placement(x,y,120.65)
            if tower is not None:
                root.occurrences.addExistingComponent(tower,loc)
                continue
            tower = root.occurrences.addNewComponent(loc).component
            tower.name = "Deck support - open socket pedestal"
            webs = tower.sketches.add(tower.xYConstructionPlane)
            webs.name = "Two pedestal webs with side access"
            for sign in (-1,1):
                y0,y1 = sorted([sign*12.7,sign*(12.7-3.175)])
                webs.sketchCurves.sketchLines.addTwoPointRectangle(point(-12.7,y0,0),point(12.7,y1,0))
            pair = adsk.core.ObjectCollection.create()
            for profile in webs.profiles:
                pair.add(profile)
            extrude(tower,pair,"deckRiserHeight - deckCapThickness",ops.NewBodyFeatureOperation,"Two upright webs")
            planes = tower.constructionPlanes
            params = planes.createInput()
            params.setByOffset(tower.xYConstructionPlane,adsk.core.ValueInput.createByString("deckRiserHeight - deckCapThickness"))
            cap = tower.sketches.add(planes.add(params))
            cap.name = "Beam seat with socket passage"
            cap.sketchCurves.sketchLines.addTwoPointRectangle(point(-12.7,-12.7,0),point(12.7,12.7,0))
            frame["named_circle"](cap,point(0,0,0),17.0,"deckSocketBore")
            plate = next(p for p in cap.profiles if p.profileLoops.count==2)
            extrude(tower,plate,"deckCapThickness",ops.JoinFeatureOperation,"Welded beam seat")
            assert tower.bRepBodies.count==1

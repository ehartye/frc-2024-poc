"""Check installed bumper geometry in robot coordinates. Interface dimensions are mm."""
import math
from pathlib import Path
import runpy

import adsk.core
import adsk.fusion


def inspect(design):
    spec = runpy.run_path(str(Path(__file__).with_name("swerve_bumper.py")))
    root = design.rootComponent
    parent = next(o for o in root.occurrences if o.component.name==spec["ASSEMBLY"])
    revision = parent.component.attributes.itemByName("bumper","revision")
    boxed = revision is not None
    if boxed and revision.value != "Boxed front support":
        raise ValueError("Unknown bumper revision")
    removed = {spec["ORIGINAL"],*spec["WOOD_NAMES"].values()}
    if any(o.component.name in removed for o in root.occurrences):
        raise ValueError("Superseded bumper material remains")
    temp = adsk.fusion.TemporaryBRepManager.get()
    point,vector = adsk.core.Point3D.create,adsk.core.Vector3D.create

    def common(a,b):
        result = temp.copy(a)
        if not temp.booleanOperation(result,b,adsk.fusion.BooleanTypes.IntersectionBooleanType):
            raise ValueError("Material intersection failed")
        return 1000*result.volume if result.isValid and result.lumps.count else 0.

    def physical(o,b):
        result = temp.copy(b.nativeObject or b)
        if not temp.transform(result,o.transform2):
            raise ValueError("Solid registration failed")
        return result

    def bounds(bb):
        return [v*10 for p in (bb.minPoint,bb.maxPoint) for v in p.asArray()]

    def near(a,b):
        return all(a[i]<=b[i+3]+.02 and b[i]<=a[i+3]+.02 for i in range(3))

    def cylinder(a,b,r):
        return temp.createCylinderOrCone(point(*(v/10 for v in a)),r/10,
                                         point(*(v/10 for v in b)),r/10)

    def area(a,b,n,step=.01):
        moved = temp.copy(a)
        matrix = adsk.core.Matrix3D.create()
        matrix.translation = vector(n[0]*step/10,n[1]*step/10,0)
        if not temp.transform(moved,matrix):
            raise ValueError("Bearing perturbation failed")
        return (common(moved,b)-common(a,b))/step

    control = cylinder([0,0,0],[0,0,2],1)
    if abs(common(control,cylinder([0,0,1],[0,0,3],1))-math.pi)>.001:
        raise ValueError("Intersection control failed")
    parts,rails,boards,hardware = {},{},{},{}
    records = []
    for occ in root.allOccurrences:
        if not occ.fullPathName.startswith(parent.name+"+"):
            continue
        if occ.bRepBodies.count!=1:
            raise ValueError("Expected one solid per bumper article")
        solid = physical(occ,occ.bRepBodies.item(0))
        name = occ.component.name
        bb = bounds(solid.preciseBoundingBox)
        path = occ.fullPathName
        parts[path] = (solid,bb)
        records.append(dict(name=name,boundsMm=bb,volumeMm3=solid.volume*1000))
        if name.endswith(" frame rail"):
            rails[name.split()[0]] = solid
        elif name.endswith(" bumper wood"):
            boards[name.split()[0]] = solid
        else:
            origin = occ.transform2.translation
            hardware[name,tuple(round(10*v,3) for v in origin.asArray())] = (solid,path)
    if (len(parts),len(rails),len(boards),len(hardware))!=(24,4,4,16):
        raise ValueError("Wrong bumper inventory")
    # A whole-body bounding box would fill the relieved central NOTE path.
    # These non-overlapping prisms must account for the entire reopened solid.
    front_regions = [[356.362,-355.6,58.5,359.537,355.6,185.5],
                     [350.012,-355.6,58.5,356.362,-210,185.5],
                     [350.012,210,58.5,356.362,355.6,185.5],
                     [350.012,-210,160.1,356.362,210,185.5]]
    if boxed:
        # Four non-overlapping tube walls and the two drilled screw landings.
        front_regions = [[356.362,-355.6,74.6,394.462,355.6,77.775],
                         [356.362,-355.6,122.225,394.462,355.6,125.4],
                         [356.362,-355.6,77.775,359.537,355.6,122.225],
                         [391.287,-355.6,77.775,394.462,355.6,122.225],
                         *[[384.937,y-15,125.4,394.462,y+15,160.] for y in (-235,235)]]
        tube_volume = (38.1*50.8-31.75*44.45)*711.2
        lands_volume = 2*(30*34.6-math.pi*3.3**2)*9.525
        if abs(rails["Front"].volume*1000-tube_volume-lands_volume)>.05:
            raise ValueError("Front tube or drilled landing material differs from the build sheet")
    covered = 0.
    for bb in front_regions:
        lo,hi = bb[:3],bb[3:]
        region = temp.createBox(adsk.core.OrientedBoundingBox3D.create(
            point(*[(a+b)/20 for a,b in zip(lo,hi)]),vector(1,0,0),vector(0,1,0),
            *[(b-a)/10 for a,b in zip(lo,hi)]))
        covered += common(region,rails["Front"])
    if abs(covered-rails["Front"].volume*1000)>.01:
        measured = bounds(rails["Front"].preciseBoundingBox)
        raise ValueError(f"Front rail envelopes miss material: boxed={boxed}, bounds={measured}, "
                         f"covered={covered:.3f}, total={rails['Front'].volume*1000:.3f} mm³")
    tool_gauges = []
    seats = []
    for face,(n,radius,thick,half) in spec["FACES"].items():
        if boxed and face=="Front":
            radius = 394.462
        for u in spec["STATIONS"][face]:
            def pos(depth):
                return [n[0]*(radius+depth)-n[1]*u,n[1]*(radius+depth)+n[0]*u,145.]
            bolt,path = hardware["M6 x 25 bumper socket screw",tuple(round(v,3) for v in pos(-thick))]
            nut,_ = hardware["M6 captive square flange barrel nut",tuple(round(v,3) for v in pos(19.05))]
            head_area = area(bolt,rails[face],n)
            flange_area = area(nut,boards[face],[-n[0],-n[1]])
            if not (44<head_area<45 and 157<flange_area<160):
                raise ValueError(f"{face} bolt or nut failed to seat: {head_area}, {flange_area}")
            # Measure actual shaft and nut extents along the registered outward normal.
            axis = 0 if n[0] else 1
            def radial(body):
                b = bounds(body.preciseBoundingBox)
                return sorted(v*n[axis] for v in (b[axis],b[axis+3]))
            # The screw tip must terminate inside the threaded barrel.
            tip = radial(bolt)[1]
            entry,exit = radial(nut)
            if not entry+6<=tip<=exit-.5:
                raise ValueError("Insufficient nominal engagement or tip projection")
            gauge = cylinder(pos(-thick-48),pos(-thick),8)
            tool_gauges.append((path,gauge,bounds(gauge.boundingBox)))
            seats.append(dict(face=face,stationMm=u,headSeatMm2=head_area,
                              nutSeatMm2=flange_area,nominalEngagementMm=tip-entry,
                              tipToOuterFaceMm=exit-tip))
    hits,access_hits,frame_seats = [],[],[]
    removal = []
    # The upper shooter overhangs the perimeter. For service, support the
    # bumper and raise the chassis 200 mm. This is a downward bumper sweep in
    # robot coordinates. The captive nuts stay inside the boards.
    for occ in root.allOccurrences:
        name = occ.component.name
        if not (name.endswith(" drilled bumper wood") or name.startswith(("fixed bumper end noodle","fixed bumper side noodle"))):
            continue
        body = physical(occ,occ.bRepBodies.item(0))
        bb = bounds(body.preciseBoundingBox)
        bb[2] -= 200
        lo,hi = bb[:3],bb[3:]
        sweep = temp.createBox(adsk.core.OrientedBoundingBox3D.create(
            point(*[(a+b)/20 for a,b in zip(lo,hi)]),vector(1,0,0),vector(0,1,0),
            *[(b-a)/10 for a,b in zip(lo,hi)]))
        removal.append((name,sweep,bb))
    removal_hits = []
    parked_bottom = math.inf
    foam_records = []
    module_count = foam_count = 0
    for o in root.allOccurrences:
        name,path = o.component.name,o.fullPathName
        module = name.startswith(("Fixed housing - ","MAXSwerve Front","MAXSwerve Rear"))
        foam = name.startswith(("fixed bumper end noodle","fixed bumper side noodle"))
        if not (o.isVisible or module or foam or path in parts):
            continue
        top = root.occurrences.itemByName(path.split("+")[0])
        # Presentation cover and team numerals duplicate the structural bumper
        # volume. Physical wood, foam and nut pockets are included explicitly.
        if top and ("display" in top.component.name.lower() or "team marking" in name.lower()):
            continue
        role = top.component.attributes.itemByName("frcDisplay","role") if top else None
        if role and role.value=="reserved-space":
            continue
        for b in o.bRepBodies:
            if not b.isSolid or not (b.isVisible or module or foam or path in parts):
                continue
            module_count += int(module)
            foam_count += int(foam)
            other = bounds(b.boundingBox)
            raw = None
            if foam:
                raw = physical(o,b)
                foam_records.append(dict(name=name,boundsMm=bounds(raw.preciseBoundingBox),
                                         volumeMm3=1000*raw.volume))
            moves_with_bumper = foam or (path in parts and not name.endswith(" frame rail"))
            if not moves_with_bumper:
                parked_bottom = min(parked_bottom,other[2])
                for panel,sweep,bb in removal:
                    if not near(bb,other):
                        continue
                    raw = raw or physical(o,b)
                    overlap = common(sweep,raw)
                    if overlap>.01:
                        removal_hits.append([panel,b.name,round(overlap,4)])
            for key,a,bb in [(k,*pair) for k,pair in parts.items()]+tool_gauges:
                if path==key or not near(bb,other):
                    continue
                raw = raw or physical(o,b)
                overlap = common(a,raw)
                if overlap>.01:
                    target = hits if a is parts[key][0] else access_hits
                    target.append([key,b.name,round(overlap,4)])
            if name=="Swerve frame - interchangeable drilled rail":
                for face in ("Left","Right","Rear"):
                    if near(bounds(rails[face].boundingBox),other):
                        raw = raw or physical(o,b)
                        n=spec["FACES"][face][0]
                        seat = area(rails[face],raw,[-n[0],-n[1]])
                        if seat>1:
                            frame_seats.append([face,seat])
    corners = [area(rails[side],rails[end],spec["FACES"][end][0])
               for side in ("Left","Right") for end in ("Front","Rear")]
    top_after_drop = max(bb[5] for _,_,bb in removal)-200
    if module_count!=1092 or foam_count!=8 or len(removal)!=12 or top_after_drop>=parked_bottom or len(frame_seats)!=3 or min(corners)<240:
        raise ValueError(f"Incomplete frame or corner interfaces: {module_count}, {foam_count}, {frame_seats}, {corners}")
    result = dict(articles=records,seats=seats,railToChassisMm2=frame_seats,cornerSeatsMm2=corners,
                moduleSolids=module_count,physicalFoamSolids=foam_count,toolChecks=len(tool_gauges),
                frontRegionsMm=front_regions,intersections=hits,toolIntersections=access_hits,
                chassisServiceRiseMm=200,endServiceVerticalGapMm=parked_bottom-top_after_drop,
                removalIntersections=removal_hits,
                foam=foam_records)
    if boxed:
        panels = {r["name"]:r["boundsMm"] for r in [*records,*foam_records]}
        tests = {"Front drilled bumper wood":(394.462,413.512),
                 "Left drilled bumper wood":(-378.587,413.512),
                 "Right drilled bumper wood":(-378.587,413.512),
                 **{f"fixed bumper end noodle 1 {i}":(413.512,477.012) for i in (0,1)},
                 **{f"fixed bumper side noodle {s} {i}":(-378.587,413.512)
                    for s in (-1,1) for i in (0,1)}}
        for name,(lo,hi) in tests.items():
            actual = panels[name]
            if abs(lo-actual[0])>.02 or abs(hi-actual[3])>.02:
                raise ValueError(f"Incorrect front or rear extent on {name}")
        cover = root.occurrences.itemByName("Continuous bumper native display v2:1")
        cover_body = physical(cover,cover.bRepBodies.item(0))
        bb = bounds(cover_body.preciseBoundingBox)
        expected = [-443.587,-439.65,57.,478.512,439.65,187.]
        cover_volume = ((478.512+443.587)*879.3-(394.462+359.537)*711.2)*130
        if (not cover.isVisible or max(abs(a-b) for a,b in zip(bb,expected))>.02 or
                abs(cover_body.volume*1000-cover_volume)>.1):
            raise ValueError("Continuous bumper cover does not match the revised panels")
        for face,axis,plane in (("x1",0,478.512),("x-1",0,-443.587),
                                ("y1",1,439.65),("y-1",1,-439.65)):
            label = root.occurrences.itemByName(f"1780 WHITE team marking {face} - visual only:1")
            if not label.isVisible or label.bRepBodies.count!=4:
                raise ValueError(f"Missing team numerals on {face}")
            for body in label.bRepBodies:
                extent = bounds(physical(label,body).preciseBoundingBox)
                if min(abs(extent[axis]-plane),abs(extent[axis+3]-plane))>.15:
                    raise ValueError(f"Team numeral no longer follows the {face} cover")
        result.update(frontTubeVolumeMm3=tube_volume,drilledLandingVolumeMm3=lands_volume,
                      coverBoundsMm=bb,registeredNumerals=16)
    return result


def require_fit(result):
    for name in ("intersections","toolIntersections","removalIntersections"):
        if result[name]:
            raise ValueError(f"{name}: {result[name]}")
    if len(result["seats"])!=8 or result["toolChecks"]!=8:
        raise ValueError("Incomplete bolt seating or service check")


def run(context):
    import json
    app = adsk.core.Application.get()
    dialog = app.userInterface.createFileDialog()
    dialog.title = "Select the private robot with removable bumpers"
    dialog.filter = "Fusion archive (*.f3d)"
    if dialog.showOpen()!=adsk.core.DialogResults.DialogOK:
        return
    previous = app.activeDocument
    options = app.importManager.createFusionArchiveImportOptions(dialog.filename)
    doc = app.importManager.importToNewDocument(options)
    try:
        doc.name = "Bumper Fit - Temporary Verification"
        design = adsk.fusion.Design.cast(doc.products.itemByProductType("DesignProductType"))
        if not design.computeAll():
            raise ValueError("Robot failed to recompute")
        result = inspect(design)
        app.log(json.dumps(result,indent=2))
        require_fit(result)
    finally:
        doc.close(False)
        if previous and previous.isValid:
            previous.activate()

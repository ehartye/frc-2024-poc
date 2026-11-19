"""Check frame material, module hole registration and deck seating in the reopened robot."""
import math

import adsk.core
import adsk.fusion


FRAME = "Team 1780 - swerve frame and electronics supports"
TAB_DEPTH_MM = {"lower": (65.56375, 69.85), "upper": (120.65, 124.3125)}


def covers(spans, low, high):
    """Test the union of the measured coaxial wall faces along the bolt."""
    end = low
    for start, stop in sorted(spans):
        if start < low - .005 or start > end + .005 or stop > high + .005:
            return False
        end = max(end, stop)
    return abs(end - high) <= .005


def inspect(design):
    root = design.rootComponent
    temp = adsk.fusion.TemporaryBRepManager.get()
    def box(body):
        b = body.boundingBox
        return [*b.minPoint.asArray(),*b.maxPoint.asArray()]
    def overlap(a,b):
        return all(a[i]<b[i+3]-.000001 and b[i]<a[i+3]-.000001 for i in range(3))
    def physical(b,o):
        raw = temp.copy(b.nativeObject or b)
        if not temp.transform(raw,o.transform2):
            raise ValueError("Could not register a solid")
        return raw
    def common(a,b):
        cut = temp.copy(a)
        if not temp.booleanOperation(cut,b,adsk.fusion.BooleanTypes.IntersectionBooleanType):
            raise ValueError("Material intersection operation failed")
        return cut.volume*1000 if cut.isValid and cut.lumps.count else 0.
    def raised(body,mm):
        r = temp.copy(body)
        mat = adsk.core.Matrix3D.create()
        mat.translation = adsk.core.Vector3D.create(0.,0.,mm/10)
        if not temp.transform(r,mat):
            raise ValueError("Contact perturbation failed")
        return r
    assembly = next(o for o in root.occurrences if o.component.name==FRAME)
    articles = []
    for o in root.allOccurrences:
        if o.fullPathName.startswith(assembly.name+"+") and o.bRepBodies.count:
            b = physical(o.bRepBodies.item(0),o)
            articles.append((o.name,b,box(b)))
    if len(articles)!=46:
        raise ValueError(f"Expected 46 installed frame articles, received {len(articles)}")
    bores = {}
    for name,body,_ in articles:
        if name.startswith("10-32 x 2.75"):
            b = body.preciseBoundingBox
            key = tuple(round((getattr(b.minPoint,i)+getattr(b.maxPoint,i))*5,3) for i in ("x","y"))
            bores[key] = {"lower":[], "upper":[]}
    if len(bores)!=12:
        raise ValueError("Missing frame bolt")
    # Check the temporary intersection implementation on known intersecting
    # and disjoint material before treating zero output as a clearance result.
    p = adsk.core.Point3D.create
    cyl = temp.createCylinderOrCone(p(0,0,0),.1,p(0,0,1),.1)
    if not math.isclose(common(cyl,raised(cyl,5)),5*math.pi,abs_tol=.001) or common(cyl,raised(cyl,20))>.00001:
        raise ValueError("Intersection controls failed")
    hits = []
    gauges = {}
    relieved = set()
    retired_hits = []
    frame_pairs = 0
    for i,(an,a,ab) in enumerate(articles):
        for bn,b,bb in articles[i+1:]:
            if overlap(ab,bb):
                frame_pairs += 1
                v = common(a,b)
                if v>.01:
                    hits.append([an,bn,round(v,4)])
    material_pairs = 0
    deck_areas = []
    module_counts = {}
    for o in root.allOccurrences:
        path = o.fullPathName
        if path.startswith(assembly.name+"+") or o.component.name==FRAME:
            continue
        name = o.component.name
        module = name.startswith(("Fixed housing - ","MAXSwerve Front","MAXSwerve Rear"))
        if not module and not o.isVisible:
            continue
        top = root.occurrences.itemByName(path.split("+")[0])
        role = top.component.attributes.itemByName("frcDisplay","role") if top else None
        if role and role.value=="reserved-space":
            continue
        for b in o.bRepBodies:
            if not b.isSolid or (not module and not b.isVisible):
                continue
            raw = physical(b,o) if module else None
            rb = box(raw if raw else b)
            if module:
                module_counts[name] = module_counts.get(name,0)+1
            # Imported fixed housing base features can reopen with generic body names.
            # Locate the complete plate by its measured installed Z envelope.
            plate = None
            if name.startswith("Fixed housing - "):
                pb = raw.preciseBoundingBox
                z_range = (10*pb.minPoint.z,10*pb.maxPoint.z)
                for label,limits in (("lower",(65.5637,78.2637)),("upper",(111.6125,124.3125))):
                    if max(abs(z-target) for z,target in zip(z_range,limits))<.05:
                        plate = label
            if plate:
                plate_axes = set()
                for face in raw.faces:
                    cylinder = adsk.core.Cylinder.cast(face.geometry)
                    if cylinder and abs(cylinder.axis.z)>.999999 and min(abs(cylinder.radius-r) for r in (.25,.255))<.000001:
                        key = tuple(round(v*10,3) for v in (cylinder.origin.x,cylinder.origin.y))
                        if key in bores:
                            bb = face.boundingBox
                            ends = (10*bb.minPoint.z,10*bb.maxPoint.z)
                            bores[key][plate].append(ends)
                            if abs(cylinder.radius-.25)<.000001:
                                plate_axes.add(key)
                            else:
                                relieved.add((key,plate))
                low, high = TAB_DEPTH_MM[plate]
                for key in plate_axes:
                    if (key,plate) in gauges:
                        raise ValueError("Two housing bodies claim the same mounting tab")
                    x, y = (v/10 for v in key)
                    def witness(radius_mm, start, stop):
                        gauge = temp.createCylinderOrCone(p(x,y,start/10), radius_mm/10,
                                                          p(x,y,stop/10), radius_mm/10)
                        return common(gauge,raw)
                    outer, inner = ((low,high-.02) if plate=="lower" else (high-.02,low))
                    gauges[key,plate] = (witness(2.49,low,high), witness(2.6,low,high),
                                         witness(3.9624,outer,outer+.02)/.02,
                                         witness(4.5,inner,inner+.02)/.02)
            is_beam = b.name.startswith("aft-electrical-beam-")
            for an,a,ab in articles:
                if is_beam and an.startswith("Deck support - open socket pedestal"):
                    raw = raw or physical(b,o)
                    # Positive volume under 0.01 mm intrusion witnesses the
                    # bearing face on the retained beam at this exact station.
                    area = common(raised(a,.01),raw)/.01
                    if area>1.:
                        deck_areas.append(area)
                if not overlap(ab,rb):
                    continue
                raw = raw or physical(b,o)
                material_pairs += 1
                volume = common(a,raw)
                if volume>.01:
                    retired = b.name.startswith(("am-","frame fastening","aft-adapter","aft-bolt","aft-crush"))
                    (retired_hits if retired else hits).append([an,b.name,name,round(volume,4)])
    if len(module_counts)!=12 or sum(module_counts.values())!=4*273:
        raise ValueError(f"Module solid inventory changed: {module_counts}")
    paired = sum(all((key,plate) in gauges and covers(walls[plate],*depth)
                     for plate,depth in TAB_DEPTH_MM.items()) for key,walls in bores.items())
    if len(gauges)!=24:
        raise ValueError(f"Expected 24 independently located mounting bores, found {len(gauges)}")
    return dict(frameArticles=len(articles),supplierSolids=sum(module_counts.values()),
                broadPhaseFramePairs=frame_pairs,broadPhaseRobotPairs=material_pairs,
                twoPlateBoltAxes=paired,
                fullDepthGaugeChecks=len(gauges), relievedPlateBores=len(relieved),
                maximumClearanceGaugeIntersectionMm3=max(row[0] for row in gauges.values()),
                minimumOversizeProbeIntersectionMm3=min(row[1] for row in gauges.values()),
                minimumFastenerSeatAreaMm2=min(row[2] for row in gauges.values()),
                minimumRailSeatAreaMm2=min(row[3] for row in gauges.values()),
                deckBearingAreaMm2=deck_areas,hits=hits,retiredHardwareHits=retired_hits)

def require_fit(fit):
    if fit["hits"] or fit["retiredHardwareHits"]:
        raise ValueError("Installed frame has material intersections")
    if fit["twoPlateBoltAxes"]!=12:
        raise ValueError("Expected twelve bolt stacks registered through both supplier plates")
    if (fit["fullDepthGaugeChecks"]!=24 or fit["maximumClearanceGaugeIntersectionMm3"]>.00001
            or fit["minimumOversizeProbeIntersectionMm3"]<=0
            or min(fit["minimumFastenerSeatAreaMm2"],fit["minimumRailSeatAreaMm2"])<=0):
        raise ValueError("Full-depth clearance, oversize or seat witness failed")
    areas = fit["deckBearingAreaMm2"]
    if len(areas)!=4 or min(areas)<=350:
        raise ValueError("Expected four pedestal seats bearing on the retained transverse beams")


def run(context):
    import json
    app = adsk.core.Application.get()
    dialog = app.userInterface.createFileDialog()
    dialog.title = "Inspect the supported 2024 MAXSwerve robot"
    dialog.filter = "Fusion archive (*.f3d)"
    if dialog.showOpen()!=adsk.core.DialogResults.DialogOK:
        return
    previous = app.activeDocument
    opened = app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(dialog.filename))
    try:
        opened.name = "Team 1780 - Mounted Frame Inspection"
        design = adsk.fusion.Design.cast(opened.products.itemByProductType("DesignProductType"))
        if not design.computeAll():
            raise ValueError("Could not recompute the robot")
        result = inspect(design)
        app.log(json.dumps(result,indent=2))
        require_fit(result)
    finally:
        opened.close(False)
        if previous and previous.isValid:
            previous.activate()

"""Screen the native starting pose against the 2024 frame-perimeter projection."""
import adsk.core
import adsk.fusion


def inspect(design):
    root = design.rootComponent
    manager = adsk.fusion.TemporaryBRepManager.get()
    # A box around the entire slab intersection also encloses the fixed
    # nonarticulated hull. Including moving solids can only enlarge this bound.
    zone = manager.createBox(adsk.core.OrientedBoundingBox3D.create(
        adsk.core.Point3D.create(0,0,9.525),adsk.core.Vector3D.create(1,0,0),
        adsk.core.Vector3D.create(0,1,0),300,300,19.05))
    extrema = [float("inf"),float("inf"),-float("inf"),-float("inf")]
    rows = []
    zone_extents = []
    zone_count = 0
    for occ in root.allOccurrences:
        path = occ.fullPathName
        top = root.occurrences.itemByName(path.split("+")[0]).component
        name = occ.component.name
        role = top.attributes.itemByName("frcDisplay","role")
        if ("display" in top.name.lower() or "team marking" in top.name.lower()
                or top.name=="SUPERSEDED cover retained hidden - visual only"
                or (role and role.value=="reserved-space") or "withdrawal reserve" in name.lower()
                or name.startswith(("fixed bumper","M6 captive square flange barrel nut","M6 x 25 bumper socket screw"))
                or name.endswith("drilled bumper wood")):
            continue
        for body in occ.bRepBodies:
            if not body.isSolid:
                continue
            raw = manager.copy(body.nativeObject or body)
            if not manager.transform(raw,occ.transform2):
                raise ValueError("Starting pose registration failed")
            bb = raw.preciseBoundingBox
            lo,hi = bb.minPoint,bb.maxPoint
            if max(abs(v) for v in (lo.x,lo.y,hi.x,hi.y))>=150:
                raise ValueError("Zone clipping box too small for this robot")
            limits = [10*lo.x,10*lo.y,10*hi.x,10*hi.y]
            rows.append((name,body.name,limits,10*hi.z,body.isVisible))
            if lo.z>19.05 or hi.z<0:
                continue
            if lo.z<0 or hi.z>19.05:
                if not manager.booleanOperation(raw,zone,adsk.fusion.BooleanTypes.IntersectionBooleanType):
                    raise ValueError("Failed to clip material to the bumper zone")
                if not raw.isValid or not raw.lumps.count or raw.volume<=0:
                    continue
                bb = raw.preciseBoundingBox
                limits = [10*bb.minPoint.x,10*bb.minPoint.y,10*bb.maxPoint.x,10*bb.maxPoint.y]
            extrema = [min(extrema[0],limits[0]),min(extrema[1],limits[1]),
                       max(extrema[2],limits[2]),max(extrema[3],limits[3])]
            zone_extents.append((name,limits))
            zone_count += 1
    if not zone_count:
        raise ValueError("Missing bumper-zone geometry")
    outside = []
    for name,body,limits,height,visible in rows:
        beyond = max(extrema[0]-limits[0],extrema[1]-limits[1],limits[2]-extrema[2],limits[3]-extrema[3])
        if beyond>6.35 and visible:
            outside.append(dict(component=name,body=body,overhangMm=beyond,boundsXYMm=limits))
    witnesses = [sorted({name for name,limits in zone_extents if abs(limits[i]-bound)<.001})
                 for i,bound in enumerate(extrema)]
    return dict(inventoriedSolids=len(rows),slabSolids=zone_count,zoneEnclosingRectMm=extrema,
                extremaComponents=witnesses,
                rectangularPerimeterUpperMm=2*(extrema[2]-extrema[0]+extrema[3]-extrema[1]),
                maxSurveyedHeightMm=max(row[3] for row in rows),
                visibleOutsideMinorAllowance=sorted(outside,key=lambda hit:-hit["overhangMm"]))

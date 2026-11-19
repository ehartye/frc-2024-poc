"""Exercise the eight MAXSwerve joints in an opened private Fusion study copy."""
import math

import adsk.core
import adsk.fusion

CORNERS = ("Front left", "Front right", "Rear left", "Rear right")
ANGLE = math.radians(30)
TOLERANCE_CM = .001


def station(corner):
    return (26 if corner.startswith("Front") else -26,
            26 if corner.endswith("left") else -26, 3.81)


def rotate(point, origin, axis, angle):
    x, y, z = [p - o for p, o in zip(point, origin)]
    u, v, w = axis
    cross = (v*z-w*y, w*x-u*z, u*y-v*x)
    dot = u*x+v*y+w*z
    c, s = math.cos(angle), math.sin(angle)
    return [o+c*p+s*k+(1-c)*dot*a for o,p,k,a in zip(origin,(x,y,z),cross,axis)]


def world(occurrence, vertices):
    points = []
    for vertex in vertices:
        point = adsk.core.Point3D.create(*vertex)
        point.transformBy(occurrence.transform2)
        points.append(point.asArray())
    return points


def inspect(design):
    root = design.rootComponent
    joints = {joint.name: joint for joint in root.asBuiltJoints}
    components = {o.component.name: o for o in root.occurrences}
    witnesses = {}
    for corner in CORNERS:
        for group, leaf in (("carrier", 11), ("wheel", 32)):
            occ = components[f"MAXSwerve {corner} {group}"]
            body = next(b for b in occ.component.bRepBodies if b.name.startswith(f"REV leaf {leaf} solid "))
            points = [v.geometry.asArray() for v in body.vertices]
            # Distinct CAD vertices make an unintended rotation observable even
            # when the center of a round part stays exactly on the axis.
            vertices = [max(points,key=lambda p:p[i]) for i in range(3)]
            witnesses[corner,group] = occ, vertices
        housing = next(o for o in root.allOccurrences if o.component.name=="Fixed housing - "+corner)
        native = housing.component.bRepBodies
        housing_body = max(native, key=lambda b:b.volume if b.isSolid else 0.)
        pts = [v.geometry.asArray() for v in housing_body.vertices]
        witnesses[corner,"housing"] = housing,[max(pts,key=lambda p:p[i]) for i in range(3)]
        groups = [native, *(components[f"MAXSwerve {corner} {g}"].component.bRepBodies for g in ("carrier","wheel"))]
        if sum(bodies.count for bodies in groups)!=274 or sum(b.isSolid for bodies in groups for b in bodies)!=273:
            raise ValueError(f"{corner} supplier body roster differs from the original")
        axis_center = station(corner)
        for xyz in world(*witnesses[corner,"housing"]):
            if math.dist(xyz,axis_center)>19:
                raise ValueError(f"{corner} housing is outside its registered station")
        for axis in ("steer", "roll"):
            motion = joints[corner+" "+axis].jointMotion
            if motion.objectType != adsk.fusion.RevoluteJointMotion.classType() or abs(motion.rotationValue)>1e-6:
                raise ValueError(f"{corner} {axis} must start at its registered zero")
    def snapshot():
        return {k:world(*pair) for k,pair in witnesses.items()}
    start = snapshot()
    def pose_error(expect):
        current = snapshot()
        return max(math.dist(a,b) for key in expect for a,b in zip(current[key],expect[key]))
    worst = 0.
    signed_axes = {}
    for corner in CORNERS:
        center = station(corner)
        for axis in ("steer","roll"):
            motion = joints[corner+" "+axis].jointMotion
            basis = (0,0,1) if axis=="steer" else ((0,1,0) if corner in ("Front left","Rear right") else (1,0,0))
            if abs(sum(a*b for a,b in zip(basis,motion.rotationAxisVector.asArray())))<.99999:
                raise ValueError(f"Unexpected {corner} {axis} axis")
            expected_positive = None
            for angle in (ANGLE,-ANGLE):
                motion.rotationValue = angle
                adsk.doEvents()
                if abs(motion.rotationValue-angle)>1e-5:
                    raise ValueError(f"{corner} {axis} command missed")
                choices = {}
                for sign in (1,-1):
                    expected = dict(start)
                    for group in (("carrier","wheel") if axis=="steer" else ("wheel",)):
                        key = corner,group
                        expected[key] = [rotate(p,center,basis,angle*sign) for p in start[key]]
                    choices[sign] = pose_error(expected)
                sign = min(choices,key=choices.get)
                if expected_positive is None:
                    expected_positive = sign
                if sign!=expected_positive or choices[sign]>TOLERANCE_CM:
                    raise ValueError(f"{corner} {axis} witness error {choices}")
                worst = max(worst,choices[sign])
                key = corner,("carrier" if axis=="steer" else "wheel")
                travel = max(math.dist(a,b) for a,b in zip(start[key],snapshot()[key]))
                if travel < .2:
                    raise ValueError(f"Insufficient {corner} {axis} witness motion")
            motion.rotationValue=0.
            adsk.doEvents()
            if pose_error(start)>TOLERANCE_CM:
                raise ValueError(f"{corner} {axis} failed to return to zero")
            signed_axes[corner,axis] = tuple(expected_positive*v for v in basis)

        # Roll the real wheel with its carrier steered, in a single assembly pose.
        steer = joints[corner+" steer"].jointMotion
        roll = joints[corner+" roll"].jointMotion
        steer.rotationValue = ANGLE
        roll.rotationValue = -ANGLE
        adsk.doEvents()
        expected = dict(start)
        for group in ("carrier","wheel"):
            key = corner,group
            points = start[key]
            if group=="wheel":
                points = [rotate(p,center,signed_axes[corner,"roll"],-ANGLE) for p in points]
            expected[key] = [rotate(p,center,signed_axes[corner,"steer"],ANGLE) for p in points]
        error = pose_error(expected)
        if error>TOLERANCE_CM:
            raise ValueError(f"{corner} combined steer and roll error {error} cm")
        worst = max(worst,error)
        roll.rotationValue = 0.
        steer.rotationValue = 0.
        adsk.doEvents()
        if pose_error(start)>TOLERANCE_CM:
            raise ValueError("Combined pose failed to return to zero")
    return {"independentSignedPoses":16,"combinedSteerAndRollPoses":4,
            "witnessToleranceMm":TOLERANCE_CM*10,"maximumWitnessErrorMm":worst*10,
            "positiveAxes":{c:{a:list(signed_axes[c,a]) for a in ("steer","roll")} for c in CORNERS}}


def inspect_archive(app, archive):
    """Run on a disposable import so failed poses cannot change the saved draft."""
    previous = app.activeDocument
    options = app.importManager.createFusionArchiveImportOptions(str(archive))
    opened = app.importManager.importToNewDocument(options)
    try:
        design = adsk.fusion.Design.cast(opened.products.itemByProductType("DesignProductType"))
        return inspect(design)
    finally:
        opened.close(False)
        if previous and previous.isValid:
            previous.activate()


def run(context):
    import json
    app = adsk.core.Application.get()
    dialog = app.userInterface.createFileDialog()
    dialog.title = "Choose the 2024 articulated MAXSwerve robot archive"
    dialog.filter = "Fusion archive (*.f3d)"
    if dialog.showOpen() == adsk.core.DialogResults.DialogOK:
        app.log(json.dumps(inspect_archive(app, dialog.filename), indent=2))

"""Build and inspect only the removable structural tie on the R13 teaching frame."""
import json
import sys
from pathlib import Path
from adsk import core
from ground_rail_service import ROOT, STATIONS, retired, new_component
from physical_bumper import CORE, COVER, native, design, sha, dimensions, registered_solid
from passive_service_fit import common, moved, overlap
from stage_climber import box, rod
sys.path.insert(0,str(ROOT / "cad/service-readiness/stiffness"))
from selection import TIE_NAMES, TUBE_NAME

R13 = json.loads((ROOT / "cad/service-readiness/ground-rails/native-fit.json").read_bytes())


def build(parent, fixture, output):
    if sha(parent) != R13["parentSha256"] or sha(fixture) != R13["archiveSha256"]:
        raise ValueError("Expected registered R9 and R13 archives")
    source, target = native(parent), None
    try:
        target = native(fixture)
        d = design(target)
        child = new_component(d,"R14 removable passive frame tie")
        base = child.features.baseFeatures.add()
        base.name = "Registered tube, sleeves and removable fasteners"
        parts = [(o,b) for o in design(source).rootComponent.allOccurrences for b in o.bRepBodies
                 if b.name in TIE_NAMES]
        if {b.name for _,b in parts} != TIE_NAMES or len(parts) != 17:
            raise ValueError("Incomplete tie inventory")
        base.startEdit()
        try:
            for occ, body in parts:
                child.bRepBodies.add(registered_solid(occ,body),base)
        finally:
            base.finishEdit()
        for copy, (_,original) in zip(child.bRepBodies,parts,strict=True):
            copy.name = original.name
            m,c = original.material,original.appearance
            copy.material = d.materials.itemByName(m.name) or d.materials.addByCopy(m,m.name)
            copy.appearance = d.appearances.itemByName(c.name) or d.appearances.addByCopy(c,c.name)
            copy.isLightBulbOn = True
        if not d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(str(output))):
            raise RuntimeError("Frame tie export failed")
    finally:
        if target and target.isValid: target.close(False)
        source.close(False)


def removable_bumper(name, component):
    if component in (CORE,COVER,"R9 bumper washers"):
        return True
    if not name.startswith("R7 "):
        return False
    i = int(name.split()[1])
    return 487 <= i <= 502 or 593 <= i < 612 and i not in (595,596,597)


def inspect(fixture, whole):
    articles = {}
    for o in whole.rootComponent.allOccurrences:
        for b in o.bRepBodies:
            if not retired(o.component.name,b.name) or b.name in TIE_NAMES:
                solid = registered_solid(o,b)
                articles[b.name] = (solid,dimensions(solid),o.component.name,b.material.name)
    if len(articles) != 909:
        raise ValueError("Expected the ground chassis and 17 removable tie articles")
    frame = {}
    for o in fixture.rootComponent.allOccurrences:
        for b in o.bRepBodies:
            if not o.isVisible or not b.isVisible:
                raise ValueError("Hidden frame article " + b.name)
            shape = registered_solid(o,b)
            frame[b.name] = shape
            if b.name in articles:
                src,bounds,_,material = articles[b.name]
                if dimensions(shape) != bounds or abs(src.volume-shape.volume) > .0001 or b.material.name != material:
                    raise ValueError("Source placement or material changed: "+b.name)
    gauge_name = "Removable 8 mm stepped inspection gauge"
    if set(frame) != TIE_NAMES | set(R13["verifiedParkedFrameBodies"]) | {gauge_name}:
        raise ValueError("Expected 34 visible fixture bodies")
    def hits(swept, skip=()):
        bounds=dimensions(swept)
        return {name:v for name,(s,b,_,_) in articles.items()
                if name not in skip and overlap(bounds,b) and (v:=common(swept,s))>.01}
    stations=[]
    for x,y in STATIONS:
        tag = f" {x} {y}"
        bolt,nut,washer = [p+tag for p in ("MISUMI CB8-100","Accu HPN-M8-C8-Z","M8 plain washer")]
        local=lambda a,b,d:rod((x,y,a),(x,y,b),d)
        checks = {
            "driver":hits(local(167,217,12),{bolt}),
            "socket":hits(local(19,68.25,19),{nut,bolt}),
            "nutAndWasherDown":hits(local(19,69.85,16),{bolt,nut,washer}),
            "shaftUp":hits(local(59,267,8),{bolt,nut,washer}),
            "headUp":hits(local(159,275,13),{bolt,nut,washer}),
            "inspectionStem":hits(local(66.65,180.65,8),TIE_NAMES),
            "inspectionGrip":hits(local(120.65,194.65,12),TIE_NAMES)}
        retained = common(local(66.65,180.65,8),frame[bolt])
        if any(checks.values()) or retained<4900:
            raise ValueError(str((x,y,checks,retained)))
        stations.append(dict(centerMm=[x,y],sweeps=checks,retainedBoltMm3=retained))
    # Bounding prisms enclose the entire tube and four welded upper sleeves.
    # All four complete fastener sets are removed, then the tube is lifted 5 mm.
    rise = box(13,-348.4,120.65,63,348.4,164.)
    slide = box(13,-348.4,125.65,63,1148.4,164.)
    bumper = {n for n,(_,_,comp,_) in articles.items() if removable_bumper(n,comp)}
    lift_hits = hits(rise,TIE_NAMES | bumper)
    slide_hits = hits(slide,TIE_NAMES | bumper)
    blocked = hits(slide,TIE_NAMES)
    if lift_hits or slide_hits or "Left drilled bumper wood" not in blocked:
        raise ValueError(str((lift_hits,slide_hits,blocked)))
    return dict(fixtureBodies=len(frame),parkedBodies=len(articles),visibleBodies=len(frame),
                stationResults=stations, liftMm=5, lateralRemovalMm=800,
                riseHits=lift_hits,lateralHits=slide_hits,bumperInstalledControlMm3=blocked,
                tubeVolumeCm3=frame[TUBE_NAME].volume)

def capture_views(archive, parent, frame_png, whole_png):
    """Render the frame and selected parked context using disposable imported documents."""
    import time
    import adsk
    app = core.Application.get()
    prior = app.activeDocument
    def settle(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            adsk.doEvents()
            time.sleep(.035)
    def render(doc, whole, destination):
        doc.activate()
        settle(.7)
        d = design(doc)
        d.activateRootComponent()
        for comp in d.allComponents:
            comp.isBodiesFolderLightBulbOn = True
        count = 0
        for occ in d.rootComponent.allOccurrences:
            occ.isLightBulbOn = True
            for body in occ.bRepBodies:
                present = not whole or not retired(occ.component.name,body.name) or body.name in TIE_NAMES
                body.isLightBulbOn = present
                count += int(present)
        if count != (909 if whole else 34) or not d.computeAll():
            raise ValueError("Unexpected visible selection")
        settle(.7)
        view = app.activeViewport
        cam = view.camera
        cam.isSmoothTransition = False
        cam.target = core.Point3D.create(0,0,37 if whole else 10)
        cam.eye = core.Point3D.create(*( (115,-140,110) if whole else (85,-120,100) ))
        cam.upVector = core.Vector3D.create(0,0,1)
        cam.isFitView = True
        view.camera = cam
        view.fit()
        view.refresh()
        settle(1.5)
        view.refresh()
        settle(.4)
        if not view.saveAsImageFile(str(destination),1400,900):
            raise ValueError("Viewport capture failed")
    try:
        for path,whole,destination in ((archive,False,frame_png),(parent,True,whole_png)):
            doc = native(path)
            try:
                render(doc, whole, destination)
            finally:
                doc.close(False)
    finally:
        if prior and prior.isValid:
            prior.activate()

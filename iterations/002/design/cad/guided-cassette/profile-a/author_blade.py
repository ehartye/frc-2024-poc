"""Create the removable POC blade in the deployed cassette coordinate system, in mm."""
import json
import math
from pathlib import Path
import traceback

from adsk import core, fusion


def run(context):
    app = core.Application.get()
    assert app.documents.count == 0 and not app.activeDocument, "Expected Fusion Home"
    data = json.loads((Path(CONFIG["jobInputs"]) / "profile.json").read_bytes())
    out = Path(CONFIG["output"])
    out.mkdir(exist_ok=False)
    state = {"sourceProfile": data["referenceSha256"], "complete": False}
    doc = None
    mm = lambda x: core.ValueInput.createByString(f"{x} mm")
    point = lambda x, y, z: core.Point3D.create(x / 10, y / 10, z / 10)
    try:
        doc = app.documents.add(core.DocumentTypes.FusionDesignDocumentType)
        design = fusion.Design.cast(doc.products.itemByProductType("DesignProductType"))
        design.designType = fusion.DesignTypes.ParametricDesignType
        blade = design.rootComponent
        for name, value in (("BladeWidth", 434), ("ClearanceBore", 3.4), ("SeatDiameter", 6.4), ("SeatPlaneOffset", .5)):
            design.userParameters.add(name, mm(value), "mm", "Profile A interface")
        section = blade.sketches.add(blade.xZConstructionPlane)
        section.name = "Deployed pickup contour - X forward, Z up"
        pts = [section.modelToSketchSpace(point(x, 0, z)) for x, z in data["sectionXZmm"]]
        for a, b in zip(pts, pts[1:] + pts[:1]):
            section.sketchCurves.sketchLines.addByTwoPoints(a, b)
        assert section.profiles.count == 1
        extrudes = blade.features.extrudeFeatures
        ex = extrudes.createInput(section.profiles.item(0), fusion.FeatureOperations.NewBodyFeatureOperation)
        ex.setSymmetricExtent(core.ValueInput.createByString("BladeWidth"), True)
        stock = extrudes.add(ex)
        stock.name = "Extrude replaceable UHMW blank symmetrically"
        body = stock.bodies.item(0)
        body.name = "Profile A wear blade - UHMW-PE stock specification"
        for lib in app.materialLibraries:
            material = next((m for m in lib.materials if "uhmw" in m.name.lower() or "ultra high molecular" in m.name.lower()), None)
            if material:
                body.material = material
                break
        nx, nz = 7 / math.hypot(7, 6.2), 6.2 / math.hypot(7, 6.2)
        face = next(f for f in body.faces if abs(f.pointOnFace.x * 10 - 596.9) < 3.1
                    and f.geometry.surfaceType == core.SurfaceTypes.PlaneSurfaceType
                    and abs(f.pointOnFace.z * 10 - (24 - (f.pointOnFace.x * 10 - 593.8) * 7 / 6.2)) < .0001
                    and f.area > 20)
        # The inclined seat meets a second facet inside each head footprint.
        # Begin above both facets so the 90-degree cutter reaches the entire rim.
        plane_input = blade.constructionPlanes.createInput()
        plane_input.setByOffset(face, core.ValueInput.createByString("SeatPlaneOffset"))
        plane = blade.constructionPlanes.add(plane_input)
        plane.name = "Countersink tool start above both wear faces"
        sketch = blade.sketches.add(plane)
        sketch.name = "Four common fastener axes on the inclined seat"
        targets = core.ObjectCollection.create()
        for y in data["stationsYmm"]:
            h = 3.5 * nz + .5
            loc = point(596.9 + nx * h, y, 17 + nz * h)
            targets.add(sketch.sketchPoints.add(sketch.modelToSketchSpace(loc)))
        holes = blade.features.holeFeatures
        cs = holes.createCountersinkInput(core.ValueInput.createByString("ClearanceBore"),
                                         core.ValueInput.createByString("SeatDiameter + 2 * SeatPlaneOffset"),
                                         core.ValueInput.createByString("90 deg"))
        cs.setPositionBySketchPoints(targets)
        cs.setDistanceExtent(mm(8))
        cut = holes.add(cs)
        cut.name = "Four flush M3 countersinks on the common mount"
        assert design.computeAll()
        archive = out / "profile-a.f3d"
        export = design.exportManager
        assert export.execute(export.createFusionArchiveExportOptions(str(archive), blade))
        doc.close(False)
        doc = None
        im = app.importManager
        doc = im.importToNewDocument(im.createFusionArchiveImportOptions(str(archive)))
        d = fusion.Design.cast(doc.products.itemByProductType("DesignProductType"))
        b = d.rootComponent.bRepBodies.item(0)
        assert d.rootComponent.bRepBodies.count == 1
        state["features"] = [d.timeline.item(i).entity.name for i in range(d.timeline.count)]
        assert all(d.timeline.item(i).healthState == fusion.FeatureHealthStates.HealthyFeatureHealthState for i in range(d.timeline.count))
        state["volumeMm3"] = b.volume * 1000
        state["parametersMm"] = {par.name: par.value * 10 for par in d.userParameters}
        state["materialInFusion"] = b.material.name
        state["faceCount"] = b.faces.count
        state["countersinks"] = sum(f.geometry.surfaceType == core.SurfaceTypes.ConeSurfaceType for f in b.faces)
        assert state["countersinks"] == 4
        state["boundsMm"] = [[round(10*x, 7) for x in pt.asArray()] for pt in (b.boundingBox.minPoint,b.boundingBox.maxPoint)]
        export = d.exportManager
        assert export.execute(export.createSTEPExportOptions(str(out / "profile-a.step"), d.rootComponent))
        state["complete"] = True
    except Exception:
        state["error"] = traceback.format_exc()
    finally:
        if doc and doc.isValid:
            doc.close(False)
        state["restored"] = app.documents.count == 0 and not app.activeDocument
        (out / "readback.json").write_text(json.dumps(state, indent=2))
        print(json.dumps(state))

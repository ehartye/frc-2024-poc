"""Verify the moving display meshes against the scored plant and measure the four tire envelopes."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def registration(folder, comparison):
    record = json.loads((ROOT/"cad/drivetrain/maxswerve-front-crossmember.json").read_bytes())
    layout = json.loads((ROOT/"cad/drivetrain/maxswerve-layout.json").read_bytes())
    model = json.loads((folder/"manifest.json").read_bytes())
    proof = json.loads(comparison.read_bytes())
    if (model["sourceSha256"] != record["archiveSha256"] or proof["displayNative"] != model["sourceSha256"] or
            proof["plantNative"] != layout["privateFusionArchiveSha256"] or proof["registeredBodyCount"] != 188 or
            not 0 <= proof["maximumSymmetricDifferenceMm3"] <= proof["volumeToleranceMm3"] <= .001 or
            model["excludedSupplierSolids"] != 1092):
        raise ValueError("Expected the registered native solid comparison for the scored assembly")
    measured = json.loads((folder/"wheel-bounds.json").read_bytes())
    diameter = layout["module"]["wheelDiameterMm"]
    wheels = {}
    for station,entry in layout["stations"].items():
        axis = layout["articulation"]["nativeJointTest"]["positiveAxes"][station]["roll"]
        axle = np.flatnonzero(axis).item()
        radial = [i for i in range(3) if i != axle]
        center = np.asarray(entry["wheelCenterMm"])
        bounds = np.array(measured[station]).reshape(-1,2,3)
        spans = bounds[:,1]-bounds[:,0]
        tire = np.flatnonzero(np.all(np.abs(spans[:,radial]-diameter)<.01,axis=1)).item()
        if not np.allclose(bounds[tire].mean(axis=0),center,atol=.01,rtol=0):
            raise ValueError("Tire center differs from the native station")
        wheels[station] = dict(centerMm=center.tolist(),nativeRollAxis=axis,diameterMm=diameter,
                              widthMm=round(spans[tire,axle],4))
    return dict(displayNative=model["sourceSha256"],plantNative=layout["privateFusionArchiveSha256"],
                displayMesh=hashlib.sha256((folder/"public-floor.glb").read_bytes()).hexdigest(),
                registeredBodyCount=proof["registeredBodyCount"],maximumSymmetricDifferenceMm3=proof["maximumSymmetricDifferenceMm3"],
                volumeToleranceMm3=proof["volumeToleranceMm3"],wheels=wheels)


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ("folder","comparison","output"):
        parser.add_argument(name,type=Path)
    args=parser.parse_args()
    args.output.write_text(json.dumps(registration(args.folder,args.comparison),indent=2)+"\n")

"""Replay the worst observed unloaded NOTE contacts on each solver clock against the exact STEP."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import sys

import numpy as np


def matrix(values):
    return np.vstack((np.reshape(values, (3, 4)), [0, 0, 0, 1]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plugin", type=Path)
    parser.add_argument("layout_step", type=Path)
    args = parser.parse_args()
    sys.path.insert(0, str(args.plugin.resolve() / "scripts"))
    from mechanical_motion import distance, load_solids, transform
    from payload_compression import interfaces, sweep
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCP.gp import gp_Trsf

    here = Path(__file__).resolve().parent
    catalog = here.parents[1] / "docs/preliminary-design"
    release = catalog / "releases" / json.loads((catalog / "releases.json").read_bytes())["current"]
    commissioning = json.loads((here / "compression-limits-002.json").read_bytes())
    original = json.loads((here / "contact-depth-002.json").read_bytes())
    data = (release / "scene.json.gz").read_bytes()
    step_hash = hashlib.sha256(args.layout_step.read_bytes()).hexdigest()
    assert hashlib.sha256(data).hexdigest() == original["sceneSha256"]
    assert json.loads((release / "release.json").read_bytes())["geometry"] == step_hash
    cases = {c["key"]: c for c in json.loads(gzip.decompress(data))["concepts"]["guided"]["cases"]}
    shapes = {n.replace("\u00c2\u00b7", "\u00b7"): s for n, s in load_solids(args.layout_step)[0].items()}
    output = []
    for clock in original["clocks"]:
        key = "continuous-cycle-" + clock["source"].split("-")[-1].split(".")[0]
        case = cases[key]
        for row in clock["maxima"]:
            name, frame = row["part"], row["frame"]
            poses = case["series"]
            relative = np.linalg.solve(matrix(poses[case["rig"][name]][frame]), matrix(poses["piece"][frame]))
            pose = gp_Trsf()
            pose.SetValues(*relative[:3].flatten())
            payload = "Chambered NOTE"
            scene = {payload: BRepBuilderAPI_Transform(shapes[payload], pose, False).Shape(), name: shapes[name]}
            def check(limit):
                contact = dict(payload=payload, contact=name, maxDepthMm=limit, basis=commissioning["basis"])
                pair = tuple(sorted(scene))
                entry = interfaces({"payloadCompression": [contact]}, scene, {})[pair]
                return sweep(entry, scene, {}, {}, 0, transform, distance)
            nominal = check(commissioning["limitsMm"][name])
            result = dict(case=key, frame=frame, timeS=row["t"], **nominal)
            if name == "Center intake belt":
                result["sevenMmControl"] = check(7)["status"]
            output.append(result)
    print(json.dumps(dict(stepSha256=step_hash, sceneSha256=original["sceneSha256"],
          scope="Analytic section depth at the reported sampled mesh extrema on each clock. Interval coverage is separate.",
          measurements=output), indent=2, ensure_ascii=True))


if __name__ == "__main__":
    main()


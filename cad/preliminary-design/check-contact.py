"""Measure rigid game-piece contact on the exact STEP and the published replay at recorded witness frames."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import shutil

import numpy as np
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.gp import gp_Trsf


def sha(file):
    return hashlib.sha256(file.read_bytes()).hexdigest()


def posed(shape, matrix):
    tr = gp_Trsf()
    tr.SetValues(*matrix[:3].flatten())
    return BRepBuilderAPI_Transform(shape, tr, True).Shape()


def matrix(values):
    return np.vstack((np.reshape(values, (3, 4)), [0, 0, 0, 1]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plugin", type=Path)
    parser.add_argument("layout_step", type=Path)
    parser.add_argument("--motion-output", type=Path, help="Run the registered motion checker on the two runner-contact intervals")
    args = parser.parse_args()
    sys.path.insert(0, str(args.plugin.resolve() / "scripts"))
    from showcase_contacts import load_solids
    from mechanical_motion import common, distance

    catalog = Path(__file__).resolve().parents[2] / "docs/preliminary-design"
    release = catalog / "releases" / json.loads((catalog / "releases.json").read_bytes())["current"]
    record = json.loads((release / "release.json").read_bytes())
    scene_file = release / "scene.json.gz"
    assert sha(scene_file) == record["sceneSha256"] and sha(args.layout_step) == record["geometry"]
    scene = json.loads(gzip.decompress(scene_file.read_bytes()))
    raw, _ = load_solids(args.layout_step)
    solids = {n.replace("\u00c2\u00b7", "\u00b7"): shape for n, shape in raw.items()}
    assert len(solids) == len(raw)
    cases = {c["key"]: c for c in scene["concepts"]["guided"]["cases"]}
    rows = []
    # At the largest mesh penetrations on each clock, measure the original curved BReps.
    for resolution, frames in [(48, [1784, 604, 453]), (96, [485, 605, 39])]:
        key = f"continuous-cycle-{resolution}"
        case = cases[key]
        for group, i in zip([["NOTE runner · left", "NOTE runner · right"], ["Windowed keeper panel"], ["Replaceable pickup lip"]], frames):
            for name in group:
                # A common robot/world transform cancels when both bodies are expressed in the stock frame.
                piece = matrix(case["series"]["piece"][i])
                stock = matrix(case["series"][case["rig"][name]][i])
                at = np.linalg.solve(stock, piece)
                moving = posed(solids["Chambered NOTE"], at)
                v = common(moving, solids[name])
                at[2, 3] += 2000
                clear = posed(solids["Chambered NOTE"], at)
                gap = distance(clear, solids[name])
                assert gap > 1000 and common(clear, solids[name]) == 0
                rows.append(dict(case=key, frame=i, seconds=case["time"][i], part=name,
                                 rigidCommonMm3=v, separatedControlGapMm=gap))
    if args.motion_output:
        from OCP.gp import gp_Quaternion
        from mechanical_motion import evaluate
        def operations(transform):
            tr=gp_Trsf()
            tr.SetValues(*transform[:3].flatten())
            rotation=tr.GetRotation()
            rot=np.array([rotation.X(),rotation.Y(),rotation.Z()])
            length=np.linalg.norm(rot)
            angle=2*np.arctan2(length,rotation.W())
            ops = []
            if length > 1e-8:
                ops.append(dict(kind="rotation", axis=(rot/length).tolist(), originMm=[0,0,0], travelDeg=float(np.degrees(angle))))
            shift = transform[:3,3]
            if np.linalg.norm(shift) > 1e-7:
                ops.append(dict(kind="translation",travelMm=shift.tolist()))
            return ops
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp)
            source=folder/"robot.step"
            shutil.copyfile(args.layout_step,source)
            study=dict(schema="agent-frc/mechanical-motion@1",step=dict(file=source.name,sha256=sha(source)),cases=[])
            for resolution,i in [(48,1784),(96,485)]:
                case=cases[f"continuous-cycle-{resolution}"]
                initial,moves={},{}
                for original in raw:
                    name=original.replace("\u00c2\u00b7","\u00b7")
                    group=case["rig"].get(name)
                    if group is None:
                        raise ValueError("Commission the motion group for "+name)
                    def pose(j):
                        return np.linalg.solve(matrix(case["series"]["frame"][j]),matrix(case["series"][group][j]))
                    start,finish=pose(i),pose(i+1)
                    if ops:=operations(start):
                        initial[original]=ops
                    if ops:=operations(finish@np.linalg.inv(start)):
                        moves[original]=ops
                study["cases"].append(dict(id=case["key"],stage="operating",basis=f"Published frames {i} to {i+1} in the robot frame",
                                           initialPose=initial,moves=moves,minGapMm=0,maxDepth=2))
            result=evaluate(folder,study)
            args.motion_output.mkdir(parents=True,exist_ok=True)
            for filename,data in [("motion.json",study),("motion-result.json",result)]:
                (args.motion_output/filename).write_text(json.dumps(data,indent=2),encoding="utf-8")
    print(json.dumps(dict(sceneSha256=sha(scene_file), stepSha256=sha(args.layout_step),
                          scope="Rigid nominal NOTE and stock at recorded frames. Foam compression is unqualified.",
                          results=rows), indent=2, ensure_ascii=True))


if __name__ == "__main__":
    main()

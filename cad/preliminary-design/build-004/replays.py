"""Bind the four season replays to their trial files and write the binding manifest for release 004.

The replays are the grouped-transform cases of the published release 003 scene: two six-NOTE scoring
clocks and two score-to-climb runs. This script re-derives the recorded motion of each replay from its
raw trial files, compares it with the published transforms frame by frame, and writes
`replay-bindings.json`: every trial file by SHA-256, the time base, the rigid groups and the measured
agreement. A single-viewer replay entry can be generated from this manifest without trusting the scene.

Usage: python cad/preliminary-design/build-004/replays.py
"""
import hashlib
import json

import numpy as np

import kin
import roster
from timeline import EVIDENCE, REPO, pin, read, sha

CAMPAIGN = EVIDENCE / "counted-reset-power-02"
OUT = REPO / "cad/preliminary-design/releases/004/replay-bindings.json"
CLIMB_START_LABEL = "Capture and lock"


def matrix(row):
    return np.vstack((np.array(row).reshape(3, 4), [0, 0, 0, 1]))


def relative(case, group, i):
    return np.linalg.solve(matrix(case["series"]["frame"][i]), matrix(case["series"][group][i]))


def error_mm(a, b, reach=180.0):
    e = a - b
    return float(np.linalg.norm(e[:3, 3]) + reach * np.abs(e[:3, :3]).max())


def scoring(case, n):
    run, plan = read(CAMPAIGN / f"run-dev-{n}.json"), read(CAMPAIGN / "plan.json")
    times = np.array(case["time"])
    trials, worst, compared = [], 0.0, 0
    for i, (leg, part) in enumerate(zip(plan["plan"], run["parts"])):
        files = dict(carry=CAMPAIGN / f"dev-{part['id']}-{n}.json",
                     shot=CAMPAIGN / f"split-tail_.25_0-dev-{part['id']}-{n}-6.88.json",
                     emptyReturn=CAMPAIGN / f"empty-dev-{part['id']}-{n}.json")
        assert sha(files["carry"]) == part["carry"] and sha(files["shot"]) == part["shot"]
        assert sha(files["emptyReturn"]) == part["emptyMotion"]
        carry, shot = read(files["carry"]), read(files["shot"])
        carry_start = part["depart"] + sum(leg["outT"]) + (1.15 if i == 0 else 0.0)
        station = np.array([*leg["loaded"][-1], 0.0])
        for row in carry["trajectory"]:
            k = int(np.argmin(abs(times - (carry_start + row[0]))))
            if abs(times[k] - (carry_start + row[0])) < 1e-6:
                worst = max(worst, error_mm(relative(case, "piece", k), kin.piece(1000 * np.array(row[1:4]), row[4:8])))
                compared += 1
        limit = part["both"] if i == 5 else run["parts"][i + 1]["depart"] - part["shotStart"]
        for v in shot["trace"]:
            k = int(np.argmin(abs(times - (part["shotStart"] + v["t"]))))
            if v["t"] <= limit and abs(times[k] - (part["shotStart"] + v["t"])) < 1e-6:
                q = np.array(v["q"])
                worst = max(worst, error_mm(relative(case, "piece", k), kin.piece(1000 * (q[:3] - station), q[3:])))
                compared += 1
        trials += [dict(role=role, note=part["id"], **pin(path)) for role, path in files.items()]
    index = [dict(role="run", **pin(CAMPAIGN / f"run-dev-{n}.json")), dict(role="plan", **pin(CAMPAIGN / "plan.json")),
             dict(role="freeze", **pin(CAMPAIGN / "freeze.json"))]
    return dict(trials=index + trials, compared=compared, worst=worst,
                recorded=["NOTE pose through pickup, carry, feed and flight", "cover leaf angle"],
                commanded=["chassis route and heading", "cassette angle (4.8 s smoothstep)", "pivot truck position (slide law)"],
                result=dict(scored=run["count"], elapsedS=run["elapsedS"]))


def climb(case, side):
    path = CAMPAIGN / f"climb-gross-{side}.json"
    trial = read(path)
    start = next(i for i, m in enumerate(case["measure"]) if m.startswith(CLIMB_START_LABEL))
    rows = trial["trace"]
    assert len(case["time"]) - start == len(rows)
    worst = 0.0
    for k, r in enumerate(rows):
        chassis = np.eye(4)
        chassis[:3, :3] = kin.quat_matrix(*r["q"])
        chassis[:3, 3] = 1000 * np.array(r["p"])
        worst = max(worst, error_mm(matrix(case["series"]["frame"][start + k]), chassis, reach=600.0))
        arm = relative(case, "arm", start + k)
        worst = max(worst, error_mm(arm, kin.arm(r["angle"], kin.PARK_PIVOT_X_MM), reach=720.0))
        chain = 1000 * np.array(r["chain"])[::2]
        worst = max(worst, float(np.abs(chain - np.array(case["chain"][start + k])).max()))
    return dict(trials=[dict(role="climb", **pin(path)), dict(role="travel", **pin(CAMPAIGN / "endgame-transfer.json"))],
                compared=len(rows), worst=worst, solvedFromFrame=start,
                recorded=["chassis pose", "cassette angle", "carriage draw", "keeper gate", "drum coil",
                          "chain centreline, 25 of 49 recorded sites"],
                commanded=["drive from the speaker to the stage", "cassette approach from 125 to the capture angle",
                           "catch dog, clutch and stop-pin movements (display allocations)", "rope shortening (display scale)"],
                result=dict(captured=trial["captured"], latched=trial["latched"],
                            disabledMinimumClearanceMm=1000 * trial["disabledMinTireOrBumperM"],
                            modeledRobotMassKg=trial["modeledRobotMass"], powerCutS=trial["cutS"]))


def main():
    scene = roster.scene()
    cases = {c["key"]: c for c in scene["concepts"]["guided"]["cases"]}
    out = []
    for key, bind in [("continuous-cycle-96", lambda c: scoring(c, 96)), ("continuous-cycle-48", lambda c: scoring(c, 48)),
                      ("score-to-climb-front", lambda c: climb(c, "front")), ("score-to-climb-rear", lambda c: climb(c, "rear"))]:
        case = cases[key]
        b = bind(case)
        assert b["worst"] < 0.01, (key, b["worst"])
        groups = {}
        for body, group in case["rig"].items():
            groups[group] = groups.get(group, 0) + 1
        rig = json.dumps(case["rig"], sort_keys=True, ensure_ascii=False).encode("utf-8")
        out.append(dict(key=key, title=case["id"], frames=len(case["time"]),
                        timeBase=dict(unit="s", start=case["time"][0], end=case["time"][-1], perFrameClock=True,
                                      perFrameReadout=True),
                        rigidGroups=groups, rigSha256=hashlib.sha256(rig).hexdigest(),
                        fieldFrame=dict(positionsMm=True, headingInFrameTransform=True),
                        deformingChain=key.startswith("score-to-climb"),
                        trials=b["trials"], recorded=b["recorded"], commanded=b["commanded"], result=b["result"],
                        verification=dict(framesCompared=b["compared"], largestDifferenceMm=round(b["worst"], 6),
                                          method="Recorded poses rebuilt from the trial files and compared with the "
                                                 "published transforms at every frame that falls on a trace sample."),
                        **({"solvedFromFrame": b["solvedFromFrame"]} if "solvedFromFrame" in b else {})))
    manifest = dict(schema="frc-2024-poc/replay-bindings@1",
                    scene=dict(file="docs/preliminary-design/releases/003/scene.json.gz", sha256=roster.SCENE_SHA256,
                               note="Grouped transforms, clocks and readouts are the published scene's cases."),
                    replays=out)
    OUT.write_bytes((json.dumps(manifest, indent=1, ensure_ascii=False) + "\n").encode("utf-8"))
    for r in out:
        print(r["key"], r["frames"], "frames;", len(r["trials"]), "trial files;", r["verification"])


if __name__ == "__main__":
    main()

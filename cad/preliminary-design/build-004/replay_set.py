"""Write the replay set of release 004: the four recorded season runs as entries of the review's one motion list.

The runs are the grouped-transform cases of the published release 003 season scene: two six-NOTE scoring
clocks (48 and 96 ring segments) and two score-to-climb runs (centre of gravity forward and rearward).
`replays.py` binds each of them to its raw trial files by SHA-256 and checks the published transforms against
those files frame by frame; this script pins that binding manifest as the evidence of every replay.

A replay is playback. The layout check neither runs nor credits it. The speaker, stage, anchors and chains are
the project's simplified field drawings from the same scene, not FIRST's field CAD. The runs were recorded with
the release 003 solids and the trial firing station (y = 5.871 m); the review shows the release 004 solids
moving on those recorded transforms.

Usage: python cad/preliminary-design/build-004/replay_set.py   (after replays.py)
"""
import base64
import gzip
import hashlib
import json
import struct

import roster
from timeline import REPO, sha

OUT = REPO / "cad/preliminary-design/releases/004"
BINDINGS = "replay-bindings.json"
NAME = "replays.json.gz"
SCORING = "Feed and deliver the NOTE through the speaker opening"
CLIMB = "Solo ONSTAGE"
FIELD = dict(
    speakerPanel=("Speaker opening panel", "#6c8fb0", "Front panel and hood of the blue speaker around the opening"),
    speakerRamp=("Speaker counter ramp", "#7da8c4", "Surface inside the speaker that carries a NOTE to the counters"),
    sensors=("Speaker counter beams", "#ff8c1f", "Housings of the two beam pairs that count a NOTE"),
    stageSteel=("Stage truss", "#7091b3", "Stage legs and truss nearest the working chain"),
    anchors=("Chain anchors", "#d4c478", "Anchors of the three stage chains"),
    otherChain=("Other stage chains", "#c2cfd9", "The two chains this climb does not use"),
    workingChain=("Working chain", "#ffc23d", "The chain the robot captures; it follows its recorded centreline"),
)
SHOT, STAGE = ("speakerPanel", "speakerRamp", "sensors"), ("stageSteel", "anchors", "otherChain")


def mesh(packed):
    v = struct.unpack(f"<{len(base64.b64decode(packed['v'])) // 4}f", base64.b64decode(packed["v"]))
    f = struct.unpack(f"<{len(base64.b64decode(packed['f'])) // 4}I", base64.b64decode(packed["f"]))
    return dict(v=[[round(x, 3) for x in v[i:i + 3]] for i in range(0, len(v), 3)], f=[list(f[i:i + 3]) for i in range(0, len(f), 3)])


def events(case):
    """Frames where the readout changes its leading phrase: the stages of the run."""
    out, last = [], None
    for i, text in enumerate(case["measure"]):
        head = text.split(" · ")[0]
        if head != last:
            out.append(dict(frame=i, label=head))
            last = head
    # A long run names many legs; keep the review's event row short.
    step = max(1, len(out) // 12)
    return out[::step]


def main():
    scene = roster.scene()
    bindings_file = OUT / BINDINGS
    bindings = {r["key"]: r for r in json.loads(bindings_file.read_bytes())["replays"]}
    cases = {c["key"]: c for c in scene["concepts"]["guided"]["cases"]}
    service_only = set(scene["concepts"]["guided"]["fixtureOnlyNames"])     # service parts outside the 232 layout solids
    field = {}
    for key, (name, colour, purpose) in FIELD.items():
        field[key] = dict(name=name, color=colour, category="reference", subsystem="Field", mesh=mesh(scene["fixtureMeshes"][key]),
                          purpose=purpose, rationale="Simplified project drawing of the field part at its field position, "
                                                     "shown for context. It carries no contact claim.")
    evidence = [dict(file=BINDINGS, sha256=sha(bindings_file),
                     role="Binding manifest: every raw trial file of this run by SHA-256, and the frame-by-frame check")]
    replays = []
    for key in ("continuous-cycle-96", "continuous-cycle-48", "score-to-climb-front", "score-to-climb-rear"):
        case, bound = cases[key], bindings[key]
        climb = key.startswith("score-to-climb")
        frames = len(case["time"])
        assert all(len(series) == frames for series in case["series"].values()) and len(case["positions"]) == frames
        groups = {}
        for group, series in case["series"].items():
            rows = []
            for m, (x, y) in zip(series, case["positions"]):
                rows.append([m[0], m[1], m[2], m[3] + x, m[4], m[5], m[6], m[7] + y, m[8], m[9], m[10], m[11]])
            groups[group] = rows
        bodies = {name: group for name, group in case["rig"].items()
                  if name not in case.get("removed", []) and name not in service_only}
        bodies.update({k: None for k in (STAGE if climb else SHOT)})
        item = dict(
            id="replay-" + key, label=case["id"] + " (recorded)",
            basis=case["basis"] + " Recorded with the release 003 solids from the trial firing station; "
                  f"{bound['verification']['framesCompared']} frames agree with the raw trial files within "
                  f"{bound['verification']['largestDifferenceMm']:g} mm.",
            objective=CLIMB if climb else SCORING, evidence=evidence,
            recorded=bound["recorded"], display=bound["commanded"],
            time=case["time"], measure=case["measure"], events=events(case), follow="frame", bodies=bodies, groups=groups)
        if climb:
            # The trial records the chain centreline about the chain's own middle. Place it on the drawn chain.
            drawn = field["workingChain"]["mesh"]["v"]
            rest = case["chain"][0]
            shift = [sum(min(p[k] for p in drawn) + max(p[k] for p in drawn) for _ in (0,)) / 2
                     - (min(p[k] for p in rest) + max(p[k] for p in rest)) / 2 for k in range(3)]
            item["tubes"] = [dict(name="Working chain", body="workingChain",
                                  points=[[[round(p[k] + shift[k], 3) for k in range(3)] for p in row] for row in case["chain"]])]
        replays.append(item)
    data = json.dumps(dict(schema="agent-frc/review-replays@1", fieldBodies=field, replays=replays),
                      separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    (OUT / NAME).write_bytes(gzip.compress(data, 9, mtime=0))
    print(NAME, len(data), "bytes raw;", hashlib.sha256((OUT / NAME).read_bytes()).hexdigest())
    for r in replays:
        print(r["id"], len(r["time"]), "frames,", len(r["bodies"]), "bodies,", len(r["events"]), "events")


if __name__ == "__main__":
    main()

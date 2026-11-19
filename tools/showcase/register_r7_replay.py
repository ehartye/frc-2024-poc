"""Bind the nominal R7 animation to the D1 pickup, release and Speaker detector."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
from scipy.spatial.transform import Rotation

from build_replay import CLIP, TO_GLTF
from wing_field_frame import WingFrame
from register_first_film import LAB, MEDIA, ROOT, read, ref, save

from r7_dynamics import verify_binding
from swerve_shot_observation import measure


def register(route, sensors, checker):
    pickup, carry, shot, field = (route/("middle-" + phase) for phase in ("pickup", "carry", "shot", "field"))
    start = json.loads((pickup/"result.json").read_bytes())
    observation = measure(pickup, carry, shot, field, sensors)
    source = Path("sim/pressure/evidence/r7-first-note-D1.json")
    record = read(source)
    if (record["observation"] != observation or verify_binding(start) != record["massBinding"] or
        start["trace"] != record["pickupTrace"] or start["end"] != record["seatedS"] or
        record["piece"] != start["pieceId"] or start["routePolicy"] != record["routePolicy"]):
        raise ValueError("Animation receipt differs from the measured route")
    event = observation["firstSimultaneousFullShadowS"]
    with (field/"trace.jsonl").open() as stream:
        final = next(row for row in map(json.loads, stream) if abs(row["t"] - event) < 1e-9)
    with (shot/"trace.jsonl").open() as stream:
        for line in stream:
            last = line
    chassis = json.loads(last)["q"]
    frame = WingFrame(start["firstMark"], start["fieldYawDeg"])
    def position(q, point):
        relative = point - np.array([*q[:2], 0])
        return TO_GLTF.apply(Rotation.from_euler("z", -q[2]).apply(relative)).tolist()
    initial = start["initialQ"]
    identity = dict(revision=start["nativeSha256"], attempt=start["attempt"], piece=start["pieceId"], clock="match")
    states = [dict(**identity, frame="glTF scene", unit="m", timeS=t, position=xyz)
              for t, xyz in ((start["start"], position(initial, np.array(initial[-7:-4]))),
                             (event, position(chassis, frame.to_local(final["q"][:3]))))]
    trace = save(LAB/"r7-first-states.json", dict(rows=states, pickupTrace=start["trace"],
                 releaseTrace=observation["sourceTrace"], fieldTrace=observation["fieldTrace"]))
    events = [dict(**identity, type=kind, target=label, criterion=criterion, timeS=t, kind="target", qualified=True)
              for kind, label, criterion, t in (
                  ("seat", "Pickup", "The middle-wing NOTE seats between both belts.", start["end"]),
                  ("release", "Release", "Wheel and belt contact ends at the attained free release.", observation["releaseS"]),
                  ("opening", "Opening", "The whole NOTE clears the registered opening.", observation["opening"]["fullyThroughByS"]),
                  ("both-pairs", "Counter", "The same NOTE fully shadows both modeled sensor pairs.", event))]
    observations = save(LAB/"r7-first-events.json", dict(source=ref(source), events=events))
    chapter = dict(id="r7-middle", label="R7 middle-wing pickup and paired detection", animation=CLIP, node="NOTE",
                   start="/rows/0", end="/rows/1", sync=[dict(runS=row["timeS"], clipS=row["timeS"]-start["start"]) for row in states],
                   events=[dict(id=e["type"], type=e["type"], timeS=e["timeS"], outcome="reached", pointer=f"/events/{i}")
                           for i,e in enumerate(events)])
    config = LAB/"r7-first-chapters.json"
    media = ref(MEDIA/"r7-nominal-d1.glb")
    save(config, dict(schema="agent-frc/replay-chapters@1", revision="R7 nominal D1", media=media,
                      trace=trace, observations=observations, chapters=[chapter]))
    geometry = subprocess.check_output([sys.executable, str(checker.with_name("replay_glb.py")),
                                        str(ROOT/media["file"]), media["sha256"]], input=json.dumps([chapter]).encode())
    keys, = json.loads(geometry)
    for row, key in zip(states, ("start","end")):
        if np.linalg.norm(np.array(keys[key]) - row["position"]) > 1e-6:
            raise ValueError("Actual NOTE animation endpoint differs from the source pose")
    index = subprocess.check_output(["node", str(checker), str(ROOT/config), str(ROOT)])
    save(MEDIA/"r7-first-index.json", json.loads(index))


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for name in ("route", "sensors", "checker"):
        p.add_argument(name, type=Path)
    args = p.parse_args()
    register(args.route, args.sensors, args.checker.resolve(strict=True))

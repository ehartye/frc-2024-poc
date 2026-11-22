"""Build the release 003-1 inputs that put the four match replays in the review's one motion list.

Release 003-1 is release 003's design, presented in one view. This script:

1. copies the trial files each replay plays back into `releases/003-1/trials/`, checking every SHA-256
   against `replay-bindings.json` (the frame-by-frame binding of the published replays to their trials);
2. writes `releases/003-1/replays.json.gz`, the pinned replay set (`agent-frc/review-replays@1`): the
   grouped transforms, clocks and readouts of the published release 003 replay scene, placed in the field
   frame, with the official speaker and stage as field bodies and the working chain carried on its
   recorded centreline;
3. copies release 003's check inputs and writes `releases/003-1/layout.json` from its layout: the same
   groups, mounts, stages, limits and budgets, with the presentation fields this release adds. Release
   003 was checked in its own folder. This release is checked from the repository root, so every pin is
   rewritten from there and the digest of each record that holds a pin is refreshed. Nothing else in a
   record changes. `robot.step` is the layout STEP release 003 pinned; it is committed here unchanged.

Usage: python cad/preliminary-design/build-003-1/replays.py FIELD_MESHES.json [GIT_REF_WITH_TRIALS]

FIELD_MESHES.json comes from `field.py`. The trial files are read from GIT_REF_WITH_TRIALS (default
`origin/private/2024-guided-cassette`) when they are not already in `trials/`.
"""
import gzip
import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
SCENE = REPO / "docs/preliminary-design/releases/003/scene.json.gz"
SCENE_SHA256 = "5a1a5861cc9df6e3fc43ec7038701a64322de008b01f0e557416839b7943e0b5"
SOURCE = REPO / "cad/preliminary-design/releases/003"
OUT = REPO / "cad/preliminary-design/releases/003-1"
PREFIX = OUT.relative_to(REPO).as_posix() + "/"
OBJECTIVE = "Recover six floor pieces, score, then climb"
CHAIN_START_MM, CHAIN_PITCH_MM = 2893.751, 100.88075

FIELD_BODIES = {
    # ID: (mesh group, name, colour, purpose)
    "field-speaker-structure": ("speakerPanel", "Blue speaker structure", "#668cb0", "Official speaker structure and panels (GE-24300)."),
    "field-speaker-ramp": ("speakerRamp", "Blue speaker ramp", "#7da8c4", "Official speaker ramp and hood surfaces the NOTE enters over."),
    "field-speaker-sensors": ("sensors", "Blue speaker sensor housings", "#ff8c1f", "Official sensor housings that count a scored NOTE."),
    "field-stage-steel": ("stageSteel", "Blue stage truss and core", "#7091b3", "Official stage structure (GE-24200)."),
    "field-stage-anchors": ("anchors", "Blue stage chain anchors", "#d4c478", "Official chain clamps and hooks."),
    "field-stage-chains": ("otherChain", "Blue stage chains, unloaded", "#c2cfd9", "The two stage chains this climb does not load."),
    "field-working-chain": ("workingChain", "Blue stage chain, loaded", "#ffc23d", "The chain the robot hooks; its links follow the recorded centreline."),
}
RATIONALE = ("Tessellated from FIRST's FE-2024.STEP in its registered field coordinates, for display. "
             "The trials use their own contact surfaces; no clearance is checked against this mesh.")
LABELS = {"continuous-cycle-96": "Six NOTES, continuous state, 96 segments (match replay)",
          "continuous-cycle-48": "Six NOTES, continuous state, 48 segments (match replay)",
          "score-to-climb-front": "Sixth shot to climb, forward CG (match replay)",
          "score-to-climb-rear": "Sixth shot to climb, rearward CG (match replay)"}


def sha(data):
    return hashlib.sha256(data).hexdigest()


def layout_id(name):
    """The layout's member IDs carry the STEP reader's double-encoded middle dot; the replay scene's names do not."""
    return name.encode("utf-8").decode("cp1252")


def trials(bindings, ref):
    """Copy each bound trial file into the release inputs and return {repository path: release-relative pin}."""
    pins = {}
    (OUT / "trials").mkdir(parents=True, exist_ok=True)
    for replay in bindings["replays"]:
        for trial in replay["trials"]:
            name = PREFIX + "trials/" + Path(trial["file"]).name
            target = REPO / name
            if not target.exists():
                target.write_bytes(subprocess.check_output(["git", "-C", str(REPO), "cat-file", "blob", ref + ":" + trial["file"]]))
            assert sha(target.read_bytes()) == trial["sha256"], "Trial file differs from its binding: " + trial["file"]
            assert pins.setdefault(trial["file"], dict(file=name, sha256=trial["sha256"]))["file"] == name
    return pins


def repath(text, done):
    """A record's text with each {file, sha256} pin written from the repository root. Returns the text."""
    def pins(node):
        if isinstance(node, dict):
            if isinstance(node.get("file"), str) and isinstance(node.get("sha256"), str):
                yield node
            for value in node.values():
                yield from pins(value)
        elif isinstance(node, list):
            for value in node:
                yield from pins(value)
    for pin in list(pins(json.loads(text))):
        name, old = pin["file"], pin["sha256"]
        if name.startswith(PREFIX):
            continue
        new = carry(name, done)
        assert old == done[name][0], "Release 003 pins a different " + name
        text, count = re.subn(r'("file"\s*:\s*")' + re.escape(name) + '"', lambda m: m.group(1) + PREFIX + name + '"', text)
        assert count, name
        text = text.replace(old, new)
    return text


def carry(name, done):
    """Copy one release 003 input into this release, repathing its pins. Returns its new SHA-256."""
    if name not in done:
        source = OUT / name if name == "robot.step" else SOURCE / name
        raw = source.read_bytes()
        out = raw
        if name.endswith(".json"):
            done[name] = (sha(raw), None)  # a record never pins itself
            out = repath(raw.decode("utf-8"), done).encode("utf-8")
        (OUT / name).write_bytes(out)
        done[name] = (sha(raw), sha(out))
    return done[name][1]


def in_field(matrix, position):
    """A robot-relative transform placed at the robot's field position."""
    m = [round(v, 6) for v in matrix]
    m[3], m[7], m[11] = round(matrix[3] + position[0], 3), round(matrix[7] + position[1], 3), round(matrix[11], 3)
    return m


def events(case):
    """Named moments from the readout: each scored NOTE, or each phase of the climb."""
    out, measure = [], case["measure"]
    if case["fixture"] == "shot":
        count = lambda text: int(re.search(r"(\d)/6", text).group(1))
        for i in range(1, len(measure)):
            if count(measure[i]) != count(measure[i - 1]):
                out.append(dict(frame=i, label="NOTE " + str(count(measure[i])) + " of 6 counted"))
    else:
        phase = lambda text: text.split(" · ")[0]
        for i in range(1, len(measure)):
            if phase(measure[i]) != phase(measure[i - 1]):
                out.append(dict(frame=i, label=phase(measure[i])))
    return out


def chain_rest(mesh):
    """The loaded chain's centreline as drawn: the centroid of its link vertices in each of the 25 recorded stations."""
    sums = [[0.0, 0.0, 0.0, 0] for _ in range(25)]
    for x, y, z in mesh["v"]:
        j = min(24, max(0, round((y - CHAIN_START_MM) / CHAIN_PITCH_MM)))
        s = sums[j]
        s[0] += x; s[1] += y; s[2] += z; s[3] += 1
    assert all(s[3] for s in sums), "The loaded chain no longer spans its 25 recorded stations"
    return [[s[0] / s[3], CHAIN_START_MM + CHAIN_PITCH_MM * j, s[2] / s[3]] for j, s in enumerate(sums)]


def main(field_file, ref="origin/private/2024-guided-cassette"):
    raw = SCENE.read_bytes()
    assert sha(raw) == SCENE_SHA256, "Unexpected release 003 replay scene"
    scene = json.loads(gzip.decompress(raw))
    field = json.loads(Path(field_file).read_text(encoding="utf-8"))
    bindings = json.loads((HERE / "replay-bindings.json").read_text(encoding="utf-8"))
    assert bindings["scene"]["sha256"] == SCENE_SHA256
    bound = {r["key"]: r for r in bindings["replays"]}
    pins = trials(bindings, ref)
    done = {}
    for name in ("missions.json", "replay-source.json"):
        carry(name, done)
    layout = json.loads(repath((SOURCE / "layout.json").read_text(encoding="utf-8"), done))
    layout["view"] = PREFIX + layout["view"]
    roster = {m for g in layout["groups"].values() for m in g["members"]}

    field_bodies = {key: dict(name=name, category="field element", color=colour, mesh=field["meshes"][group], purpose=purpose, rationale=RATIONALE,
                              material="Official field assembly") for key, (group, name, colour, purpose) in FIELD_BODIES.items()}
    rest = chain_rest(field["meshes"]["workingChain"])
    replays, skipped = [], set()
    for case in scene["concepts"]["guided"]["cases"]:
        key, bind = case["key"], bound[case["key"]]
        frames = len(case["time"])
        assert frames == bind["frames"]
        bodies = {}
        for name, group in case["rig"].items():
            member = layout_id(name)
            if member not in roster:
                skipped.add(name)  # service-only parts of the 003 replay scene that are not in the 232-solid layout
            elif name not in case["removed"]:
                bodies[member] = group
        shot = case["fixture"] == "shot"
        for key_ in ("field-speaker-structure", "field-speaker-ramp", "field-speaker-sensors") if shot else ("field-stage-steel", "field-stage-anchors", "field-stage-chains"):
            bodies[key_] = None
        used = sorted({g for g in bodies.values() if g})
        replay = dict(
            id=key, label=LABELS[key], objective=OBJECTIVE, basis=case["basis"],
            evidence=[dict(pins[t["file"]], role=t["role"] + (" · " + t["note"] if t.get("note") else "")) for t in bind["trials"]],
            recorded=bind["recorded"] + ["Compared with the trial files at " + str(bind["verification"]["framesCompared"]) + " frames; largest difference "
                                         + format(bind["verification"]["largestDifferenceMm"], ".4f") + " mm"],
            display=bind["commanded"] + ([] if shot else ["Loaded chain links follow 25 of the 49 recorded centreline sites; no link contact or load is computed"]),
            time=[round(t, 4) for t in case["time"]], measure=case["measure"], events=events(case), follow="frame", bodies=bodies,
            groups={g: [in_field(m, case["positions"][i]) for i, m in enumerate(case["series"][g])] for g in used})
        if not shot:
            replay["tubes"] = [dict(name="Blue stage chain, loaded", body="field-working-chain",
                                    points=[[[round(rest[j][k] + now[j][k] - case["chain"][0][j][k], 2) for k in range(3)] for j in range(25)] for now in case["chain"]])]
        replays.append(replay)
    assert skipped <= set(scene["concepts"]["guided"]["fixtureOnlyNames"]), skipped
    content = json.dumps(dict(schema="agent-frc/review-replays@1", fieldBodies=field_bodies, replays=replays),
                         ensure_ascii=False, separators=(",", ":"), sort_keys=True).encode("utf-8")
    packed = gzip.compress(content, compresslevel=9, mtime=0)
    (OUT / "replays.json.gz").write_bytes(packed)

    presentation = layout["presentation"]
    presentation.update(
        title="Guided-cassette preliminary design", context="2024",
        focus="Play the match replays in the motion list for the whole cycle: six floor NOTES scored, then the climb. The mechanism cases above them "
              "are the checked discharge, the service-pin withdrawal and the displaced control.",
        notes=["Release 003-1 is the release 003 design in one view. The layout, limits, budgets, checked cases and trial data are unchanged; "
               "the four match replays now play in this page's motion list.",
               "The layout check covers one NOTE discharge (0.3 s of the 96-segment trial) and the service-pin withdrawal. The match replays are "
               "recorded playback of the pinned trial files. They are not clearance cases and the decision does not depend on them.",
               *scene["notes"], *scene["concepts"]["guided"]["notes"], scene["concepts"]["guided"]["focus"],
               "Speaker and stage are tessellated from FIRST's FE-2024.STEP (SHA-256 " + field["source"]["sha256"][:12] + "…) for display. "
               "The 003 replay scene's three service-only slide parts are not in the 232-solid layout and are not shown."],
        cutaway=dict(label="Speaker cutaway", hides=["field-speaker-structure"], outline=scene["corners"]),
        links=[dict(label="2024 robot study", href="../../../")],
        replays=[dict(file=PREFIX + "replays.json.gz", sha256=sha(packed))])
    (OUT / "layout.json").write_bytes((json.dumps(layout, ensure_ascii=False, indent=1) + "\n").encode("utf-8"))
    print("carried", sorted(k for k, v in done.items() if v[0] != v[1]), "repathed;", sorted(k for k, v in done.items() if v[0] == v[1]), "unchanged")
    print("replays.json.gz", len(packed), "bytes,", len(content), "raw;", len(pins), "trial files;", [(r["id"], len(r["time"]), len(r["events"])) for r in replays])


if __name__ == "__main__":
    main(*sys.argv[1:])

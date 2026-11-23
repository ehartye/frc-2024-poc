"""Roster of the 232 layout solids: rigid class, functional group, review text and mount datums.

Rigid classes come from the `rig` maps of the draft layout's replay scene, which name the rigid
body each solid rides on in the scoring replays and in the climb replays. A plan group here is one
function on one rigid body, so every member of a moving group shares the same motion.
"""
import gzip
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
SCENE = HERE / "inputs/replay-scene.json.gz"
SCENE_SHA256 = "5a1a5861cc9df6e3fc43ec7038701a64322de008b01f0e557416839b7943e0b5"
CYCLE_KEY = "continuous-cycle-96"
CLIMB_KEY = "score-to-climb-front"

# (rigid body in the scoring replays, rigid body in the climb replays) -> rigid class
CLASSES = {
    ("frame", "frame"): "frame", ("slide", "frame"): "truck", ("slide", "pin"): "catch-dog",
    ("arm", "arm"): "cassette", ("flap", "flap"): "cover-leaf", ("piece", "piece"): "note",
    ("arm", "carriage"): "carriage", ("arm", "keeper"): "keeper", ("arm", "drum"): "drum",
    ("arm", "clutch"): "clutch", ("arm", "cord"): "rope", ("arm", "stopLeft"): "stop-left",
    ("arm", "stopRight"): "stop-right",
}
CLASS_LABEL = {
    "frame": "frame side", "truck": "pivot truck", "catch-dog": "catch dog", "cassette": "cassette side",
    "cover-leaf": "leaf", "carriage": "hook carriage", "keeper": "keeper", "drum": "pivot shaft and drums",
    "clutch": "clutch", "rope": "rope", "stop-left": "left stop pin", "stop-right": "right stop pin",
}
NOTE = "Chambered NOTE"
BATTERY = "Battery and terminal case"
SERVICE_PIN = "Battery service pin"
BUMPER = "Blue bumper set"


def scene():
    data = SCENE.read_bytes()
    assert hashlib.sha256(data).hexdigest() == SCENE_SHA256, "Unexpected replay scene"
    return json.loads(gzip.decompress(data))


def build(names):
    """Return (rigid class by solid, plan groups, component review text) for the STEP solid names."""
    sc = scene()
    guided = sc["concepts"]["guided"]
    cases = {c["key"]: c for c in guided["cases"]}
    cycle, climb = cases[CYCLE_KEY]["rig"], cases[CLIMB_KEY]["rig"]
    bodies = {n: b for n, b in sc["bodies"].items() if n not in sc["concepts"]["field"]["remove"]}
    bodies.update(sc["concepts"]["field"]["add"])
    bodies.update(guided["add"])
    assert set(names) == set(climb) and set(names) <= set(cycle) and set(names) <= set(bodies)
    rigid = {n: CLASSES[cycle[n], climb[n]] for n in names}

    def function(n):
        b = bodies[n]
        if n == NOTE:
            return "NOTE", "payload"
        if n == SERVICE_PIN:
            return "Removable service pin", "mechanism"
        if n == BUMPER:
            return "Bumpers", "bumper"
        if n == BATTERY:
            return "Battery", "battery"
        if "swerve" in b["group"].lower():
            return b["group"], "drive"
        if b["group"] == "Chassis":
            return "Chassis", "structure"
        return b["group"], "mechanism"

    spans = {}
    for n in names:
        spans.setdefault(function(n)[0], set()).add(rigid[n])
    groups, member_group, components = {}, {}, {}
    for n in names:
        fn, role = function(n)
        key = fn if len(spans[fn]) == 1 else fn + " · " + CLASS_LABEL[rigid[n]]
        groups.setdefault(key, dict(role=role, members=[]))["members"].append(n)
        member_group[n] = key
        b = bodies[n]
        components[n] = dict(name=n, category=b["category"], material=b["material"], subsystem=b["group"],
                             purpose=b["purpose"], rationale=b["rationale"],
                             volumeType="envelope" if b["category"] == "reserved-space" else "material")
    return rigid, groups, member_group, components, sc


def contacts(shapes, bounds, cache, step_sha256, reach_mm=0.5):
    """Closest approach of every pair of parked solids nearer than reach_mm: {(a, b): (gap, point on b)}.

    `bounds` returns (xmin, ymin, zmin, xmax, ymax, zmax). The result is cached beside the builder, keyed by the
    STEP digest.
    """
    from OCP.BRepExtrema import BRepExtrema_DistShapeShape
    if cache.is_file():
        saved = json.loads(cache.read_bytes())
        if saved["stepSha256"] == step_sha256 and saved["reachMm"] == reach_mm:
            return {tuple(r["pair"]): r for r in saved["pairs"]}
    boxes = {n: bounds(s) for n, s in shapes.items()}
    names = sorted(shapes)
    rows = []
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            A, B = boxes[a], boxes[b]
            if any(A[k] - B[k + 3] > reach_mm or B[k] - A[k + 3] > reach_mm for k in range(3)):
                continue
            q = BRepExtrema_DistShapeShape(shapes[a], shapes[b])
            if not q.IsDone() or q.Value() > reach_mm:
                continue
            pa, pb = q.PointOnShape1(1), q.PointOnShape2(1)
            rows.append(dict(pair=[a, b], gapMm=q.Value(), onFirstMm=[pa.X(), pa.Y(), pa.Z()],
                             onSecondMm=[pb.X(), pb.Y(), pb.Z()]))
    cache.parent.mkdir(parents=True, exist_ok=True)
    cache.write_text(json.dumps(dict(stepSha256=step_sha256, reachMm=reach_mm, pairs=rows), indent=1) + "\n",
                     encoding="utf-8", newline="\n")
    return {tuple(r["pair"]): r for r in rows}


# The rigid body each class rides on; a mount prefers a host on that body.
PARENT = {"truck": "frame", "catch-dog": "truck", "cassette": "truck", "cover-leaf": "cassette", "carriage": "cassette",
          "keeper": "carriage", "drum": "truck", "clutch": "drum", "rope": "carriage", "stop-left": "cassette",
          "stop-right": "cassette", "frame": "frame"}


def mounts(groups, member_group, rigid, near, touch_mm=1e-5):
    """One mount per mechanism member that bears on a solid of another group; at least one per mechanism group."""
    by_member = {}
    for (a, b), row in near.items():
        by_member.setdefault(a, []).append((row["gapMm"], b, row["onSecondMm"]))
        by_member.setdefault(b, []).append((row["gapMm"], a, row["onFirstMm"]))
    out, floating = [], []
    for key, g in groups.items():
        if g["role"] != "mechanism":
            continue
        for member in g["members"]:
            options = [o for o in by_member.get(member, []) if groups[member_group[o[1]]]["role"] != "payload"]
            if not options:
                floating.append(member)
                continue
            # Prefer a touching host: on another group and the parent rigid body, then another group, then its own
            # group. A member that touches nothing takes its nearest neighbour and says so.
            wanted = PARENT[rigid[member]]
            options.sort(key=lambda o: (o[0] > touch_mm, member_group[o[1]] == key,
                                        rigid[o[1]] not in (wanted, rigid[member]), o[0], o[1]))
            gap, host, point = options[0]
            if gap <= touch_mm:
                basis = (f"Parked STEP surface datum on {host} where {member} bears on it. "
                         f"Measured separation {gap:.3f} mm.")
            else:
                basis = (f"Parked STEP surface datum on {host} nearest {member}. Measured clearance {gap:.3f} mm; "
                         "the pair does not touch in the parked layout, so the connection is not yet drawn.")
                floating.append(member)
            out.append(dict(group=key, member=member, on=host, pointMm=point, basis=basis))
        assert any(m["group"] == key for m in out), "No neighbouring solid for " + key
    return out, floating

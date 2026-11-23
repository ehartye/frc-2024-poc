"""Build the release 004 check inputs from the pinned traces and ledgers.

Usage: python cad/preliminary-design/build-004/build.py PLUGIN [--only PHASE,...] [--tolerance MM]

PLUGIN is an Agent FRC checkout. The script writes every input of the layout check into
cad/preliminary-design/releases/004/. It is deterministic: the same evidence and plugin give the same bytes.
"""
import argparse
import hashlib
import json
import math
import os
import shutil
import sys
from pathlib import Path

import numpy as np

import evidence
import kin
import ledgers
import risks
import roster
import segments
import timeline
from timeline import EVIDENCE, REPO, pin, read, sha

HERE = Path(__file__).resolve().parent
OUT = Path(os.environ["RELEASE_004_OUT"]).resolve() if os.environ.get("RELEASE_004_OUT") else REPO / "cad/preliminary-design/releases/004"
ISSUE = "2024-004-guided-cassette"
SOURCE_STEP = EVIDENCE / "guided-hoist-01/service-locked-robot.step"
LIMITS = REPO / "cad/preliminary-design/compression-limits-002.json"
NARRATIVE = REPO / "kickoff/strategic-narrative.md"
ISSUED_DESIGN = REPO / "cad/preliminary-design/releases/003/design.json"
NOTE_GROUP = "NOTE"
SERVICE_INDEX_DEG = -40.0
BATTERY_LIFT_MM = 370.0
PIN_PULL_MM = 225.0

# Pairs that touch by design and move against each other: journals, the hinge and the lead screw.
JOINTS = [
    ("Pivot bearing · left", "Pivot torque shaft", "Pivot shaft journal in the left bearing"),
    ("Pivot bearing · right", "Pivot torque shaft", "Pivot shaft journal in the right bearing"),
    ("Pivot reducer", "Pivot torque shaft", "Pivot shaft in the reducer output bore"),
    ("Flap axle", "Mouth retention flap", "Cover leaf hinge on its axle"),
    ("Slide screw", "Travel nut and clevis", "Lead screw running in the travel nut"),
    # Endgame: the pivot shaft is declutched from the cassette and reels the hoist lines.
    ("Pivot drive socket", "Sliding pivot clutch", "Pivot clutch sliding on its drive socket"),
    ("Cassette cheek · left", "Pivot torque shaft", "Declutched pivot shaft turning in the left cassette cheek"),
    ("Cassette cheek · right", "Pivot torque shaft", "Declutched pivot shaft turning in the right cassette cheek"),
    ("Pivot torque shaft", "Service index disk", "Declutched pivot shaft turning in the service index disk"),
    ("Common keeper yoke and sliding rack", "Keeper return pinion", "Keeper rack meshing with its return pinion"),
    ("Hoist rope drum to sheave · left", "Pivot cable drum · left", "Hoist line leaving the left cable drum"),
    ("Hoist rope drum to sheave · right", "Pivot cable drum · right", "Hoist line leaving the right cable drum"),
    ("Hoist rear redirect sheave · left", "Hoist rope working span · left", "Hoist line running over the left redirect sheave"),
    ("Hoist rear redirect sheave · right", "Hoist rope working span · right", "Hoist line running over the right redirect sheave"),
]
# The working span of each 3 mm hoist line is drawn at its parked length and rides on the cassette. As the hook
# carriage draws in, the line is taken up on the drum, so the drawn span runs on into the splice by the length
# taken up. That overlap is line that has left the span, not interference between two parts.
LINES = [
    ("Hoist rope working span · left", "Hoist splice allowance · left", "Hoist line taken up at the left hook carriage splice"),
    ("Hoist rope working span · right", "Hoist splice allowance · right", "Hoist line taken up at the right hook carriage splice"),
    ("Hoist rope working span · left", "Spliced hoist eye · left", "Hoist line running into its own spliced eye at the left hook carriage"),
    ("Hoist rope working span · right", "Spliced hoist eye · right", "Hoist line running into its own spliced eye at the right hook carriage"),
]
LINE_SECTION_MM2 = math.pi * 3.0 ** 2 / 4

PHASES = dict(
    reset=dict(purpose="operating", duty="reset", carried=False, stage="reset",
               text="Empty cassette returns from the launch angle to the floor pickup pose"),
    acquire=dict(purpose="operating", duty="acquire", carried=False, stage="acquire",
                 text="Floor NOTE rides over the lip and is drawn to its seat"),
    lift=dict(purpose="operating", duty="lift", carried=True, stage="retain",
              text="Seated NOTE is lifted to the launch angle while the truck follows its slide law"),
    feed=dict(purpose="operating", duty="feed", carried=False, stage="transfer",
              text="Metering belts feed the NOTE into the launch belts"),
    flight=dict(purpose="operating", duty="feed", carried=False, stage="deliver",
                text="Released NOTE flies free through the speaker opening"),
    stow=dict(purpose="park", duty="pivot-empty", carried=False, stage="park",
              text="Empty cassette moves from the launch angle to the parked angle"),
    approach=dict(purpose="operating", duty="pivot-empty", carried=False, stage="endgame",
                  text="Empty cassette lowers to the chain capture angle and the keeper opens (commanded approach)"),
    capture=dict(purpose="operating", duty="climb", carried=False, stage="endgame",
                 text="Keeper closes on the chain; catch dog and clutch movements are drawn as allocations"),
    hoist=dict(purpose="operating", duty="climb", carried=False, stage="endgame",
               text="Pivot motor reels both lines and the hook carriage draws along the cassette"),
    hold=dict(purpose="operating", duty="hold", carried=False, stage="endgame",
              text="Carriage seats on the slide stops and holds after motor power is cut"),
)
SCORING = ("reset", "acquire", "lift", "feed", "flight", "stow")
CLIMB_TRACE = ledgers.CLIMB["front"]
# Display allocations of the hoist publisher (guided-hoist-01/publish-review.py): not solved by the climb trial.
DOG_MM, DOG_S = -10.8, 0.13
CLUTCH_MM, CLUTCH_S = -4.2, (0.15, 0.35)
STOP_MM = 8.0
REEL_FROM_S = 0.45


# Every pin is written from the repository root, which is the project the layout check runs in.
PREFIX = OUT.relative_to(REPO).as_posix() + "/" if OUT.is_relative_to(REPO) else ""


def save(name, value):
    path = OUT / name
    path.write_bytes((json.dumps(value, indent=1, allow_nan=False, ensure_ascii=False) + "\n").encode("utf-8"))
    return dict(file=PREFIX + name, sha256=sha(path))


def copy(source, name):
    shutil.copyfile(source, OUT / name)
    return dict(file=PREFIX + name, sha256=sha(OUT / name))


def scoring_states(c):
    """Sampled states of each scoring phase on one clock that starts when the empty return starts."""
    def snap(st):
        if abs(st["px"] - kin.PARK_PIVOT_X_MM) < 1e-6:
            st["px"] = kin.PARK_PIVOT_X_MM
        return st
    note_floor = c.carry_state(0)["note"]
    out = {}
    ts = np.r_[np.arange(c.trigger, c.reset_end, 0.025), c.reset_end]
    out["reset"] = [snap(dict(c.reset_state(float(t)), note=note_floor, clock=float(t - c.trigger))) for t in ts]
    t0 = out["reset"][-1]["clock"]
    rows = [snap(dict(c.carry_state(i))) for i in range(len(c.rows))]
    for st in rows:
        st["clock"] = t0 + st.pop("t")
    k0 = max(i for i, r in enumerate(c.rows) if r[0] <= c.start_lift)
    out["acquire"], out["lift"] = rows[:k0 + 1], rows[k0:]
    t1 = rows[-1]["clock"]
    shots = [snap(dict(c.shot_state(j))) for j in range(len(c.shot_t))]
    for st in shots:
        st["clock"] = t1 + st.pop("t")
    j_release = max(j for j, t in enumerate(c.shot_t) if t <= c.release + 1e-9)
    j_through = min(j for j, t in enumerate(c.shot_t) if t >= c.through - 1e-9)
    out["feed"], out["flight"] = shots[:j_release + 1], shots[j_release:j_through + 1]
    last = shots[j_through]
    out["stow"] = [dict(last), dict(last, angle=kin.PARK_ANGLE_DEG, clock=last["clock"] + 1.0)]
    out.update(climb_states(read(CLIMB_TRACE), dict(out["stow"][-1])))
    # Each phase starts exactly where the previous one ended.
    order = list(PHASES)
    for prev, nxt in zip(order, order[1:]):
        end = out[prev][-1]
        out[nxt][0] = dict(end, clock=end["clock"])
    return {k: [segments.complete(s) for s in v] for k, v in out.items()}


def climb_states(trial, parked):
    """States of the endgame from the parked cassette through capture, hoist and the disabled hold.

    The approach is commanded: the cassette lowers to the capture angle of the trial while the keeper opens to
    the gate the trial starts with, and the cover leaf follows its cam. From capture on, cassette angle,
    carriage draw, keeper gate and drum coil are the recorded trial values.
    """
    rows = trial["trace"]
    first = rows[0]
    base = dict(parked, draw=0.0, gate=0.0, coil=0.0, dog=0.0, clutch=0.0, stop=0.0)
    t0 = base["clock"]

    def cam(angle):
        return min(base["flap"], 90.0 * min(1.0, max(0.0, (angle - 102.0) / 18.0)))
    span = kin.PARK_ANGLE_DEG - first["angle"]
    steps = int(math.ceil(span))
    approach = []
    for k in range(steps + 1):
        u = k / steps
        angle = kin.PARK_ANGLE_DEG - span * u
        approach.append(dict(base, angle=angle, flap=cam(angle), gate=1000 * first["gate"] * u, clock=t0 + 2.0 * u))
    t1 = approach[-1]["clock"]
    solved = []
    for r in rows:
        t = r["t"]
        solved.append(dict(base, angle=r["angle"], flap=cam(r["angle"]), draw=1000 * r["draw"],
                           gate=1000 * max(0.0, r["gate"]), coil=r["coil"],
                           dog=DOG_MM * min(t / DOG_S, 1.0),
                           clutch=CLUTCH_MM * min(max((t - CLUTCH_S[0]) / (CLUTCH_S[1] - CLUTCH_S[0]), 0.0), 1.0),
                           stop=STOP_MM if r["latchedNow"] else 0.0, clock=t1 + t))
    i_reel = max(i for i, r in enumerate(rows) if r["t"] <= REEL_FROM_S + 1e-9)
    i_latch = min(i for i, r in enumerate(rows) if r["latchedNow"])
    return dict(approach=approach, capture=solved[:i_reel + 1], hoist=solved[i_reel:i_latch],
                hold=solved[i_latch - 1:])


class Geometry:
    """The plugin's own placement, distance and common-volume functions on the release solids."""

    def __init__(self, plugin, step):
        sys.path.insert(0, str(Path(plugin).resolve() / "scripts"))
        import mechanical_motion
        self.mm = mechanical_motion
        self.shapes = mechanical_motion.load_solids(step)[0]

    def joint_record(self, case, pair, basis, step_sha, fractions=9, take_up=None):
        """Sample one designed joint through a case: the pair must never share material.

        With take_up=(start, end) carriage draw in mm the pair is a hoist line and its splice: the drawn span may
        run into the splice by no more than the line section times the line taken up."""
        a, b = pair
        start = {n: self.mm.transform(self.shapes[n], case.get("initialPose", {}).get(n, []), 1) for n in pair}
        rows = []
        for k in range(fractions):
            t = k / (fractions - 1)
            sa = self.mm.transform(start[a], case["moves"].get(a, []), t)
            sb = self.mm.transform(start[b], case["moves"].get(b, []), t)
            gap = self.mm.distance(sa, sb)
            row = dict(t=t, gapMm=gap, volumeMm3=self.mm.common(sa, sb) if gap <= 1e-7 else 0.0)
            if take_up is not None:
                row["takeUpMm"] = take_up[0] + (take_up[1] - take_up[0]) * t
                row["allowedMm3"] = LINE_SECTION_MM2 * row["takeUpMm"] + 0.5
            rows.append(row)
        worst = max(r["volumeMm3"] for r in rows)
        if take_up is None:
            ok = worst <= 0.01
            method = (f"Exact common volume of the two placed solids at {fractions} evenly spaced path fractions. "
                      "A designed sliding or turning fit passes when no sample shares more than 0.01 mm3.")
        else:
            ok = all(r["volumeMm3"] <= r["allowedMm3"] for r in rows)
            method = (f"Exact common volume of the drawn line span and its splice at {fractions} evenly spaced path "
                      f"fractions. The 3 mm line is flexible and is taken up as the hook carriage draws in; the drawn "
                      f"span keeps its parked length. The pair passes when the shared volume never exceeds the line "
                      f"section ({LINE_SECTION_MM2:.3f} mm2) times the line taken up, plus 0.5 mm3.")
        return dict(schema="frc-2024-poc/joint-qualification@1", stepSha256=step_sha, case=case["id"], pair=[a, b],
                    status="pass" if ok else "fail", basis=basis, method=method, largestCommonMm3=worst, samples=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plugin", type=Path)
    parser.add_argument("--only", default="", help="Comma-separated phases to build, for a partial trial run")
    parser.add_argument("--tolerance", type=float, default=1.0, help="Largest departure of a case path from its trace, mm")
    parser.add_argument("--refine", default="", help="Finer tolerance for named phases, as PHASE=MM,PHASE=MM; use it only "
                        "where a coarse case leaves a clearance thinner than the coarse tolerance")
    parser.add_argument("--battery", action="store_true", help="Include the battery lift at the service index")
    parser.add_argument("--count", action="store_true", help="Print the case count of each phase and write nothing")
    opt = parser.parse_args()
    refine = {k: float(v) for k, v in (item.split("=") for item in opt.refine.split(",") if item)}
    assert set(refine) <= set(PHASES), f"Unknown phase in --refine: {sorted(set(refine) - set(PHASES))}"
    tolerance_of = lambda phase: refine.get(phase, opt.tolerance)
    OUT.mkdir(parents=True, exist_ok=True)
    if not opt.count:
        for stale in [*OUT.glob("trial-*.json"), *OUT.glob("joint-*.json")]:
            stale.unlink()
    step_file, revision_file = OUT / "robot.step", OUT / "step-revision.json"
    import revise_step
    done = read(revision_file) if revision_file.is_file() else {}
    if not (step_file.is_file() and done.get("sha256") == sha(step_file) and done.get("sourceSha256") == sha(SOURCE_STEP)
            and done.get("raiseMm") == revise_step.RAISE_MM):
        save("step-revision.json", revise_step.build(SOURCE_STEP, step_file))
    step = dict(file=PREFIX + "robot.step", sha256=sha(step_file))
    geo = Geometry(opt.plugin, step_file)
    names = sorted(geo.shapes)
    rigid, groups, member_group, components, scene = roster.build(names)
    near = roster.contacts(geo.shapes, geo.mm.bounds, HERE / "derived/contacts.json", step["sha256"])
    mounts, floating = roster.mounts(groups, member_group, rigid, near)
    classes = sorted(set(rigid.values()))
    by_class = {c: [n for n in names if rigid[n] == c] for c in classes}
    note = by_class["note"][0]
    corners = {}
    for c, members in by_class.items():
        boxes = [geo.mm.bounds(geo.shapes[n]) for n in members]
        corners[c] = segments.box_corners([min(b[i] for b in boxes) for i in range(3)] + [max(b[i] for b in boxes) for i in (3, 4, 5)])

    cyc = timeline.Cycle()
    states = scoring_states(cyc)
    wanted = [p for p in opt.only.split(",") if p] or list(PHASES)
    if opt.count:
        for phase in wanted:
            cuts, _ = segments.cut(states[phase], classes, PHASES[phase]["carried"], tolerance_of(phase), corners)
            print(phase, len(states[phase]), "samples ->", len(cuts), "cases, largest deviation",
                  round(max(c[2] for c in cuts), 3), "mm")
        return
    limits = read(LIMITS)
    rules = [dict(payload=note, contact=n, maxDepthMm=v, basis=limits["basis"]) for n, v in limits["limitsMm"].items()]
    assert all(r["contact"] in names for r in rules)
    quals = evidence.qualifications(cyc)
    operating = quals.pop("operating")
    save("operating-point.json", operating)
    register = risks.risks(operating)

    design_form = json.loads(ISSUED_DESIGN.read_bytes())
    design_form.update(revision=3, parentRevision=2)
    design_form["provenance"]["basis"] = ("Release 004 of the guided cassette. The editable envelope is secondary to the "
                                           "registered solid model and the pinned trial traces.")
    design = save("design.json", design_form)
    datum = dict(name="Parked robot frame, mm", robotToStepMm=np.eye(4).flatten().tolist())

    cases, stages, cycle_ids, trials, timelines, case_phase, deviations = [], {}, [], {}, {}, {}, {}
    for phase in PHASES:
        if phase not in wanted:
            continue
        spec, sts = PHASES[phase], states[phase]
        cuts, s = segments.cut(sts, classes, spec["carried"], tolerance_of(phase), corners)
        for index, (i, j, deviation) in enumerate(cuts, 1):
            a, b = sts[i], sts[j]
            cid = phase if len(cuts) == 1 else f"{phase}-{index:02d}"
            ops, _, _ = segments.segment_ops(a, b, classes, spec["carried"])
            init = segments.initial_ops(a, classes)
            moves = {n: ops[rigid[n]] for n in names if ops[rigid[n]]}
            if not moves:
                continue
            case = dict(id=cid, stage=spec["purpose"],
                        basis=f"{spec['text']}. Cycle clock {a['clock']:.3f} to {b['clock']:.3f} s; the path departs from "
                              f"its trace samples by at most {deviation:.2f} mm.",
                        initialPose={n: init[rigid[n]] for n in names if init.get(rigid[n])},
                        moves=moves, minGapMm=0, maxDepth=12, payloadCompression=rules)
            joints = [(x, y, why, None) for x, y, why in JOINTS if moves.get(x, []) != moves.get(y, [])]
            joints += [(x, y, why, (a.get("draw", 0.0), b.get("draw", 0.0))) for x, y, why in LINES
                       if moves.get(x, []) != moves.get(y, []) or init.get(rigid[x]) != init.get(rigid[y])]
            moving_groups = sorted({member_group[n] for n in moves})
            stage = dict(purpose=spec["purpose"], movingGroups=moving_groups,
                         allowanceBasis="Zero clearance between unrelated solids. The NOTE may indent each registered "
                                        "contact surface to its declared limit. Designed joints are qualified pair by pair.")
            if note in moves and phase in ("acquire", "lift", "feed", "flight"):
                q = quals[phase]
                record = dict(schema="frc-2024-poc/trial-record@1", case=cid, phase=phase, interaction=q["interaction"],
                              cycleClockS=[a["clock"], b["clock"]], pathDeviationMm=deviation,
                              sources=q["sources"], qualification=q["qualification"], limits=q["limits"])
                path = dict(case=cid, payload=NOTE_GROUP, moves={note: moves[note]},
                            initialPose={note: case["initialPose"].get(note, [])})
                if phase == "flight":
                    record["freePath"] = path
                    trial = save(f"trial-{cid}.json", record)
                    stage["freePath"] = {NOTE_GROUP: dict(evidence=trial, basis=q["basis"])}
                else:
                    carriers = sorted(q["mechanisms"])
                    record["transport"] = dict(path, mechanisms=carriers)
                    record["transport"] = {k: record["transport"][k] for k in ("case", "payload", "mechanisms", "moves", "initialPose")}
                    trial = save(f"trial-{cid}.json", record)
                    stage["transportedBy"] = {NOTE_GROUP: dict(
                        mechanisms=carriers, basis=q["basis"], evidence=trial,
                        drive={g: q["mechanisms"][g] for g in carriers if g not in moving_groups})}
                trials[cid] = trial
            case["_joints"] = joints
            cases.append(case)
            stages[cid] = stage
            cycle_ids.append(cid)
            case_phase[cid] = phase
            deviations[cid] = deviation
            span = s[j] - s[i]
            fr = [k / 24 for k in range(25)]
            sk = [(s[k] - s[i]) / span for k in range(i, j + 1)]
            clock = np.interp(fr, sk, [sts[k]["clock"] for k in range(i, j + 1)])
            timelines[cid] = dict(time=[round(float(t), 4) for t in clock],
                                  measure=[f"{spec['text']} · cassette {np.interp(f, [0, 1], [a['angle'], b['angle']]):.1f}°"
                                           for f in fr])

    # The climb is not a payload transport, so its trial is pinned by the mission case, not by a stage.
    hold_ids = [c for c in cycle_ids if case_phase[c] == "hold"]
    if hold_ids:
        q = evidence.climb()
        cid = hold_ids[-1]
        endgame = [c for c in cycle_ids if case_phase[c] in ("approach", "capture", "hoist", "hold")]
        trials[cid] = save(f"trial-{cid}.json", dict(
            schema="frc-2024-poc/trial-record@1", case=cid, phase="hold", interaction=q["interaction"], basis=q["basis"],
            cycleClockS=[states["approach"][0]["clock"], states["hold"][-1]["clock"]], cases=endgame,
            pathDeviationMm=max(deviations[c] for c in endgame),
            sources=q["sources"], qualification=q["qualification"], limits=q["limits"]))

    # Service: withdraw the index pin from the parked robot, and lift the battery at the service index.
    pin_case = dict(id="withdraw-service-pin", stage="service",
                    basis="Withdraw the lateral battery-service index pin from its guide, from the parked layout.",
                    moves={roster.SERVICE_PIN: [kin.translation([0, PIN_PULL_MM, 0])]}, extract=[roster.SERVICE_PIN],
                    minGapMm=0, maxDepth=10, checkLinearEnvelope=True, payloadCompression=rules, _joints=[])
    stages[pin_case["id"]] = dict(purpose="service", movingGroups=[member_group[roster.SERVICE_PIN]],
                                  allowanceBasis="Full straight withdrawal past the installed robot envelope.")
    cases.append(pin_case)
    index = segments.complete(dict(angle=SERVICE_INDEX_DEG, px=float(cyc.scheduled(SERVICE_INDEX_DEG)), note=states["reset"][0]["note"]))
    index_pose = segments.initial_ops(index, classes)
    lift_case = dict(id="lift-battery-at-service-index", stage="service",
                     basis=f"Empty cassette indexed at {SERVICE_INDEX_DEG:.0f} degrees with the truck on its slide law. The "
                           f"battery lifts straight up {BATTERY_LIFT_MM:.0f} mm with its leads disconnected.",
                     initialPose={n: index_pose[rigid[n]] for n in names if index_pose.get(rigid[n])},
                     moves={roster.BATTERY: [kin.translation([0, 0, BATTERY_LIFT_MM])]},
                     minGapMm=0, maxDepth=10, checkLinearEnvelope=True, payloadCompression=rules, _joints=[])
    if opt.battery:
        stages[lift_case["id"]] = dict(purpose="service", movingGroups=[member_group[roster.BATTERY]],
                                       allowanceBasis="Zero clearance to every installed solid through the straight lift.")
        cases.append(lift_case)

    step_sha = step["sha256"]
    failed_joints = []
    for case in cases:
        entries = []
        for n, (x, y, why, take_up) in enumerate(case.pop("_joints"), 1):
            record = geo.joint_record(case, (x, y), why, step_sha, take_up=take_up)
            if record["status"] != "pass":
                failed_joints.append((case["id"], x, y, round(record["largestCommonMm3"], 3)))
            entries.append(dict(pair=[x, y], basis=why, qualifiedBy=save(f"joint-{case['id']}-{n}.json", record)))
        if entries:
            case["intendedInterfaces"] = entries

    for row in failed_joints:
        print("joint shares material:", *row)
    motion = save("motion.json", dict(schema="agent-frc/mechanical-motion@1", layoutDesignSha256=design["sha256"],
                                      layoutDatum=datum, step=step, cases=cases))

    mass_lines, payload, gross = ledgers.masses()
    inspection = [0.0, 0.0]
    for line in mass_lines:      # the checker's own summation order, so the running range matches bit for bit
        inspection = [a + b for a, b in zip(inspection, line["kg"])]
    running = [a + b for a, b in zip(inspection, payload)]
    duty_table = ledgers.duties()
    duty, electrical = {}, {}
    for cid in cycle_ids:
        d = duty_table[PHASES[case_phase[cid]]["duty"]]
        for condition, pack in ledgers.PACKS.items():
            key = f"{cid}-{condition}"
            duty[key] = dict(stage=cid, duty=d["text"], batteryCondition=condition,
                             requiredMotionMinV=d["requiredV"], sources=list(d["sources"]))
            electrical[key] = dict(loadedKg=running, batterySideA=d["sources"], openCircuitV=[pack["openCircuitV"]] * 2,
                                   resistanceOhm=[pack["resistanceOhm"]] * 2, basis=d["basis"], sourcePins=d["pins"])
    limits_pin = save("limits.json", dict(
        basis="Kickoff inspection limit and the selected 8 V minimum loaded bus. A duty with a motor terminal demand "
              "above 8 V requires that higher voltage.",
        inspectionLimitKg=ledgers.INSPECTION_LIMIT_KG, operatingMinV=ledgers.OPERATING_MIN_V,
        massLines=[r["id"] for r in mass_lines], electricalCases=duty))
    budget = save("budgets.json", dict(issue=ISSUE, step=step, motion=motion, design=design, masses=mass_lines,
                                       runningPayloadKg=payload, requirements=limits_pin, electricalCases=electrical))
    gross_pin = copy(ledgers.GROSS, "gross-power.json")
    hold = dict(id="inspection-mass", unit="kg", scenarioId="inspection-mass-upper",
                scenario="Upper inspection estimate including slide service stock")
    accepted = save("accepted.json", dict(**hold, basis="Kickoff inspection limit applied to the current gross upper allocation.",
                                          phase="preliminary-design", criterion=dict(range=[0, ledgers.INSPECTION_LIMIT_KG]),
                                          interfaces=["Chassis"], source=limits_pin))
    observed = save("observed.json", dict(**hold, valueField="inspectionHighKg", value=gross["inspectionHighKg"],
                                          source=gross_pin, issue=ISSUE, designSha256=design["sha256"], stepSha256=step_sha))
    narrative = copy(NARRATIVE, "strategic-narrative.md")

    # The control repeats a feed or flight case that has no designed joint in motion, so only the displaced tread can reject it.
    control_case = next(c for c in cases if case_phase.get(c["id"]) in ("feed", "flight") and note in c["moves"]
                        and not c.get("intendedInterfaces"))
    presentation = evidence.presentation(components, groups, member_group, rigid, duty_table, timelines, case_phase,
                                         register, operating)
    if not opt.only and OUT == REPO / "cad/preliminary-design/releases/004":
        # The four recorded season runs join the one motion list as replays, bound to their trial files.
        import replay_set
        import replays
        replays.main()
        replay_set.main()
        presentation["replays"] = [dict(file=PREFIX + replay_set.NAME, sha256=sha(OUT / replay_set.NAME))]
    plan = dict(schema="agent-frc/preliminary-layout@1", issue=ISSUE, datum=datum, step=step, design=design, motion=motion,
                groups=groups, mounts=mounts, stages=stages, cycle=cycle_ids, budgets=budget, missingInputs=[],
                premises=[dict(accepted=accepted, current=observed)], view=PREFIX + "package-motion.html", presentation=presentation,
                narrativeReview=dict(source=narrative, objectives=evidence.objectives(case_phase)),
                control=dict(case=control_case["id"], moving=note, displaced="Feed tread · left", offsetMm=[0, -22, 0],
                             basis="Push the left feed tread 22 mm into the NOTE, beyond its registered indentation limit."))
    save("layout.json", plan)
    save("missions.json", evidence.missions(case_phase, trials, cycle_ids, register))
    summary = {p: sum(1 for v in case_phase.values() if v == p) for p in PHASES}
    print("Prepared", len(names), "solids,", len(groups), "groups,", len(mounts), "mounts,", len(cases), "cases", summary)
    print("largest path deviation mm", max(deviations.values(), default=0))
    print("members that touch no other solid when parked:", floating)


if __name__ == "__main__":
    main()

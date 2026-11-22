"""Build the release 004 check inputs from the pinned traces and ledgers.

Usage: python cad/preliminary-design/build-004/build.py PLUGIN [--only PHASE,...] [--tolerance MM]

PLUGIN is an Agent FRC checkout. The script writes every input of the layout check into
cad/preliminary-design/releases/004/. It is deterministic: the same evidence and plugin give the same bytes.
"""
import argparse
import hashlib
import json
import math
import shutil
import sys
from pathlib import Path

import numpy as np

import evidence
import kin
import ledgers
import roster
import segments
import timeline
from timeline import EVIDENCE, REPO, pin, read, sha

HERE = Path(__file__).resolve().parent
OUT = REPO / "cad/preliminary-design/releases/004"
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
]

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
)


def save(name, value):
    path = OUT / name
    path.write_bytes((json.dumps(value, indent=1, allow_nan=False, ensure_ascii=False) + "\n").encode("utf-8"))
    return dict(file=name, sha256=sha(path))


def copy(source, name):
    shutil.copyfile(source, OUT / name)
    return dict(file=name, sha256=sha(OUT / name))


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
    # Each phase starts exactly where the previous one ended.
    order = list(PHASES)
    for prev, nxt in zip(order, order[1:]):
        end = out[prev][-1]
        out[nxt][0] = dict(end, clock=end["clock"])
    return {k: [segments.complete(s) for s in v] for k, v in out.items()}


class Geometry:
    """The plugin's own placement, distance and common-volume functions on the release solids."""

    def __init__(self, plugin, step):
        sys.path.insert(0, str(Path(plugin).resolve() / "scripts"))
        import mechanical_motion
        self.mm = mechanical_motion
        self.shapes = mechanical_motion.load_solids(step)[0]

    def joint_record(self, case, pair, basis, step_sha, fractions=9):
        """Sample one designed joint through a case: the pair must never share material."""
        a, b = pair
        start = {n: self.mm.transform(self.shapes[n], case.get("initialPose", {}).get(n, []), 1) for n in pair}
        rows = []
        for k in range(fractions):
            t = k / (fractions - 1)
            sa = self.mm.transform(start[a], case["moves"].get(a, []), t)
            sb = self.mm.transform(start[b], case["moves"].get(b, []), t)
            gap = self.mm.distance(sa, sb)
            rows.append(dict(t=t, gapMm=gap, volumeMm3=self.mm.common(sa, sb) if gap <= 1e-7 else 0.0))
        worst = max(r["volumeMm3"] for r in rows)
        return dict(schema="frc-2024-poc/joint-qualification@1", stepSha256=step_sha, case=case["id"], pair=[a, b],
                    status="pass" if worst <= 0.01 else "fail", basis=basis,
                    method=f"Exact common volume of the two placed solids at {fractions} evenly spaced path fractions. "
                           "A designed sliding or turning fit passes when no sample shares more than 0.01 mm3.",
                    largestCommonMm3=worst, samples=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plugin", type=Path)
    parser.add_argument("--only", default="", help="Comma-separated phases to build, for a partial trial run")
    parser.add_argument("--tolerance", type=float, default=1.0, help="Largest departure of a case path from its trace, mm")
    opt = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    step_file = OUT / "robot.step"
    import step_names
    fixed, _ = step_names.repair(SOURCE_STEP.read_bytes())
    if not step_file.is_file() or step_file.read_bytes() != fixed:
        step_file.write_bytes(fixed)
    step = dict(file="robot.step", sha256=sha(step_file))
    geo = Geometry(opt.plugin, step_file)
    names = sorted(geo.shapes)
    rigid, groups, member_group, components, scene = roster.build(names)
    near = roster.contacts(geo.shapes, geo.mm.bounds, HERE / "derived/contacts.json", step["sha256"])
    mounts = roster.mounts(groups, member_group, rigid, near)
    classes = sorted(set(rigid.values()))
    by_class = {c: [n for n in names if rigid[n] == c] for c in classes}
    note = by_class["note"][0]

    cyc = timeline.Cycle()
    states = scoring_states(cyc)
    wanted = [p for p in opt.only.split(",") if p] or list(PHASES)
    limits = read(LIMITS)
    rules = [dict(payload=note, contact=n, maxDepthMm=v, basis=limits["basis"]) for n, v in limits["limitsMm"].items()]
    assert all(r["contact"] in names for r in rules)
    quals = evidence.qualifications(cyc)

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
        cuts, s = segments.cut(sts, classes, spec["carried"], opt.tolerance)
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
            joints = [(x, y, why) for x, y, why in JOINTS if moves.get(x, []) != moves.get(y, [])]
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

    # Service: withdraw the index pin from the parked robot, and lift the battery at the service index.
    pin_case = dict(id="withdraw-service-pin", stage="service",
                    basis="Withdraw the lateral battery-service index pin from its guide, from the parked layout.",
                    moves={roster.SERVICE_PIN: [kin.translation([0, PIN_PULL_MM, 0])]}, extract=[roster.SERVICE_PIN],
                    minGapMm=0, maxDepth=10, checkLinearEnvelope=True, payloadCompression=rules, _joints=[])
    stages[pin_case["id"]] = dict(purpose="service", movingGroups=[member_group[roster.SERVICE_PIN]],
                                  allowanceBasis="Full straight withdrawal past the installed robot envelope.")
    cases.append(pin_case)

    step_sha = step["sha256"]
    for case in cases:
        entries = []
        for n, (x, y, why) in enumerate(case.pop("_joints"), 1):
            record = geo.joint_record(case, (x, y), why, step_sha)
            assert record["status"] == "pass", (case["id"], x, y, record["largestCommonMm3"])
            entries.append(dict(pair=[x, y], basis=why, qualifiedBy=save(f"joint-{case['id']}-{n}.json", record)))
        if entries:
            case["intendedInterfaces"] = entries

    motion = save("motion.json", dict(schema="agent-frc/mechanical-motion@1", layoutDesignSha256=design["sha256"],
                                      layoutDatum=datum, step=step, cases=cases))

    mass_lines, payload, gross = ledgers.masses()
    inspection = [sum(r["kg"][i] for r in mass_lines) for i in (0, 1)]
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

    control_case = next(c for c in cases if case_phase.get(c["id"]) == "feed" and note in c["moves"])
    presentation = evidence.presentation(components, groups, member_group, rigid, duty_table, timelines, case_phase)
    plan = dict(schema="agent-frc/preliminary-layout@1", issue=ISSUE, datum=datum, step=step, design=design, motion=motion,
                groups=groups, mounts=mounts, stages=stages, cycle=cycle_ids, budgets=budget, missingInputs=[],
                premises=[dict(accepted=accepted, current=observed)], view="index.html", presentation=presentation,
                narrativeReview=dict(source=narrative, objectives=evidence.objectives(case_phase)),
                control=dict(case=control_case["id"], moving=note, displaced="Feed tread · left", offsetMm=[0, -22, 0],
                             basis="Push the left feed tread 22 mm into the NOTE, beyond its registered indentation limit."))
    save("layout.json", plan)
    save("missions.json", evidence.missions(case_phase, trials, cycle_ids))
    summary = {p: sum(1 for v in case_phase.values() if v == p) for p in PHASES}
    print("Prepared", len(names), "solids,", len(groups), "groups,", len(mounts), "mounts,", len(cases), "cases", summary)
    print("largest path deviation mm", max(deviations.values(), default=0))


if __name__ == "__main__":
    main()

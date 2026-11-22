"""Trial qualifications, narrative coverage, presentation text and the risk register of release 004.

Every qualification statement is checked against the evidence file it cites when the release is built:
the build stops if a cited file no longer shows the stated outcome. A qualification element with no
supporting trial is left out, so the readiness report names it as missing.
"""
import json

from timeline import EVIDENCE, pin, read

CAMPAIGN = EVIDENCE / "counted-reset-power-02"
PICKUP = EVIDENCE / "pickup-brackets-04"
RETENTION = EVIDENCE / "retention-brackets-04"
CLIMB = EVIDENCE / "climb-brackets-04"
WINDOW = EVIDENCE / "scoring-window-04"
LEAF_STOP_LIMIT_DEG = 1.0
FLOOR_ALLOWANCE_MM = 10.0


def _cited(path, **facts):
    """A pin plus the facts read from that file."""
    return dict(pin(path), observed=facts)


AXIS_TEXT = dict(
    yaw=("chassis heading error at the firing station", "deg"),
    y=("lateral position of the firing station", "m"),
    angle=("cassette launch angle", "deg"),
    x=("distance of the firing station from the alliance wall", "m"),
    speed=("launcher belt speed command", "m/s"),
)


def delivery_window():
    """Delivery through the speaker opening at the revised firing lane, from the single-shot campaign.

    Returns (operating point record, brackets). A value passes when every shot of the frozen population passes
    wholly through the opening, and fails when any shot does not. Each bracket is the passing value next to the
    nearest failing value on one side of the operating point.
    """
    summary = read(WINDOW / "summary.json")
    assert not summary["errors"], "The campaign has shots that did not run"
    point = summary["operatingPoints"]["revised"]
    released = summary["operatingPoints"]["released"]
    assert point["frozen"]["delivered"] == point["frozen"]["shots"] >= 34
    assert point["nominal"]["delivered"] == point["nominal"]["shots"] >= 12
    here = point["point"]
    sweeps = summary["sweeps"]["revised"]
    brackets, windows = [], {}
    for axis, (text, unit) in AXIS_TEXT.items():
        line = {r["value"]: r for r in sweeps.get(axis, {}).get("frozen", [])}
        if axis == "speed":      # the low end was found on the nominal carries at the released lane
            for r in summary["sweeps"]["released"]["speed"]["nominal"]:
                line.setdefault(r["value"], r)
        ok = lambda r: r["delivered"] == r["shots"]
        windows[axis] = dict(unit=unit, tried=sorted(line), delivers=sorted(v for v, r in line.items() if ok(r)),
                             fails=sorted(v for v, r in line.items() if not ok(r)))
        for side in (-1, 1):
            values = sorted((v for v in line if (v - here.get(axis, 0.0)) * side > 0), key=lambda v: abs(v - here.get(axis, 0.0)))
            passing = here.get(axis, 0.0)
            for v in values:
                if ok(line[v]):
                    passing = v
                    continue
                r = line[v]
                brackets.append(dict(
                    input=text + (" (lower side)" if side < 0 else " (upper side)"), unit=unit,
                    passing=float(passing), failing=float(v),
                    note=f"At {v:g} {unit}, {r['delivered']} of {r['shots']} shots pass wholly through the opening. "
                         f"At {passing:g} {unit} all {line[passing]['shots'] if passing in line else point['frozen']['shots']} do"
                         + (f", with {line[passing]['smallestOpeningMarginMm']:.1f} mm smallest margin." if passing in line
                            else f", with {point['frozen']['smallestOpeningMarginMm']:.1f} mm smallest margin."),
                    evidence=pin(WINDOW / "summary.json")))
                break
    assert brackets, "No failing value bounds the delivery window"
    record = dict(
        schema="frc-2024-poc/operating-point@1",
        firingStation=dict(xM=here["x"], yM=here["y"], cassetteAngleDeg=here["angle"], launcherCommandMps=here["speed"]),
        trialStation=dict(xM=released["point"]["x"], yM=released["point"]["y"], cassetteAngleDeg=released["point"]["angle"],
                          launcherCommandMps=released["point"]["speed"]),
        basis="The cycle trials fired from the trial station. The single-shot campaign re-fires every carried NOTE of "
              "those trials from other stations, angles and headings; the robot-frame NOTE path up to the opening is the "
              "same at both stations. The firing station is the lane the release selects.",
        atFiringStation=dict(frozenStarts=point["frozen"], nominalStarts=point["nominal"]),
        atTrialStation=dict(frozenStarts=released["frozen"], nominalStarts=released["nominal"]),
        deliveryWindow=windows, allFrozenShots=summary["allFrozenShots"], extendedObservation=summary["extendedObservation"],
        criterion=summary["criterion"], source=pin(WINDOW / "summary.json"))
    return record, brackets


def qualifications(cyc):
    """Per scoring phase: interaction, transporter drives, source pins, limits and the qualification record."""
    carry96, carry48 = cyc.carry, read(CAMPAIGN / "dev-middle-48.json")
    shot96 = cyc.shot
    shot48 = read(CAMPAIGN / "split-tail_.25_0-dev-middle-48-6.88.json")
    dead = read(EVIDENCE / "pickup-height-control-01/dead.json")
    hold = read(EVIDENCE / "pickup-lip/power-off-hold.json")
    bypass = read(EVIDENCE / "bounded-risk-01/s-bypass.json")
    window = read(EVIDENCE / "counter-drive-01/elevation-window.json")["rows"]
    assert carry96["seated"] and carry96["held"] and carry48["seated"] and carry48["held"]
    assert carry96["dt"] == 0.00025 and carry48["dt"] == 0.0005
    assert dead["enabled"] is False and dead["seated"] is False
    open_hold = [r for r in hold if r["flap"] is False]
    kept_hold = [r for r in hold if r["flap"] is True]
    assert all(r["carpetContactS"] is not None for r in open_hold) and all(r["carpetContactS"] is None for r in kept_hold)
    assert shot96["passShot"] and shot48["passShot"] and shot96["both"] and shot48["both"]
    assert bypass["policy"] == "bypass" and bypass["passShot"] is False and bypass["minBusV"] < 8
    passing = next(r for r in window if r["angle"] == 122)
    failing = next(r for r in window if r["worstMarginMm"] < 0)
    assert passing["worstMarginMm"] > 0
    timestep = dict(coarseS=0.0005, fineS=0.00025, sameOutcome=True)
    pick = read(PICKUP / "results.json")
    pick_rows = {r["label"]: r for r in pick["cases"]}
    assert pick_rows["n48-h5-y0-mu0.45-a8"]["passScreen"] and pick_rows["n96-h5-y0-mu0.45-a8"]["passScreen"]

    def pick_bracket(row, passing, failing, repeat=None):
        """One pickup input: the passing value nearest a failing one. Checked against the listed cases."""
        assert passing in row["passingValues"] and failing in row["failingValues"], (row["input"], passing, failing)
        note = f"Passing values {sorted(set(row['passingValues']))}; failing values {row['failingValues']}. 48 ring segments."
        if repeat:
            assert all(pick_rows[k]["passScreen"] is v for k, v in repeat.items())
            note += " Repeated at 96 segments with the same outcome: " + ", ".join(
                f"{k} {'passes' if v else 'fails'}" for k, v in repeat.items()) + "."
        return dict(input=row["input"], unit=row["unit"] or "coefficient", passing=passing, failing=failing, note=note,
                    evidence=pin(PICKUP / "results.json"))
    by_input = {b["input"]: b for b in pick["brackets"]}
    pickup_brackets = [
        pick_bracket(by_input["rigid blade tip height above the carpet"], 10, 12,
                     {"n96-h14-y0-mu0.45-a8": False, "n96-h16-y0-mu0.45-a8": False}),
        pick_bracket(by_input["lateral NOTE offset from the cassette centre"], 0.05, 0.1),
        pick_bracket(by_input["nose roller and pad grip coefficient"], 0.3, 0.2,
                     {"n96-h5-y0-mu0.1-a8": False, "n96-h5-y0-mu0.05-a8": False}),
        pick_bracket(by_input["nose motor phase current limit"], 1.0, 0.5,
                     {"n96-h5-y0-mu0.45-a1": True, "n96-h5-y0-mu0.45-a2": True}),
    ]
    keep = {(r["n"], r["liftS"], r["peakChassisAccelMps2"]): r for r in read(RETENTION / "results.json")["cases"]}
    assert all(r["held"] for r in keep.values()) and keep[48, 4.8, 2.6]["sameAsReleased"] and keep[96, 4.8, 2.6]["sameAsReleased"]
    slow, fast = keep[48, 1.8, 2.6], keep[48, 1.4, 2.6]
    assert slow["stopOvertravelDeg"] <= LEAF_STOP_LIMIT_DEG < fast["stopOvertravelDeg"]
    hardest = max(a for (n, lift_s, a) in keep if lift_s == 4.8)
    lift_bracket = dict(
        input="lift duration", unit="s", passing=slow["liftS"], failing=fast["liftS"],
        note=f"The NOTE stayed in its seat band in every case, down to a {min(k[1] for k in keep):g} s lift and up to "
             f"{hardest:g} m/s2 of chassis acceleration (2.6 m/s2 as released), so retention itself has no failing "
             f"value in the tested range. The first limit reached is the cover leaf leaving its cam stop: "
             f"{slow['stopOvertravelDeg']:.2f} deg of overtravel at {slow['liftS']} s and {fast['stopOvertravelDeg']:.2f} deg at "
             f"{fast['liftS']} s against a {LEAF_STOP_LIMIT_DEG:g} deg limit set here. The released 4.8 s lift gives "
             f"{keep[48, 4.8, 2.6]['stopOvertravelDeg']:.2f} deg.",
        evidence=pin(RETENTION / "results.json"))
    sources = cyc.sources()
    scope = ("Free rigid NOTE on commanded cassette, truck and chassis motion. Carpet grip, tread compliance and "
             "field counter speed are assumed values awaiting measurement.")
    acquire = dict(
        interaction="Floor acquisition over the Profile A lip to the seat",
        basis="Recorded free-NOTE path of the fine nominal pickup trial, same centre and section orientation.",
        mechanisms={"Cassette drive": "Nose roller and centre belt driven at the commanded surface speed",
                    "Metering drive": "Feed belts driven at the commanded surface speed"},
        sources=dict(carry=sources["carry"], run=sources["run"], freeze=sources["freeze"]),
        limits=scope + " The brackets move one input at a time on the first nominal NOTE; combined errors are not bracketed.",
        qualification=dict(
            result="pass",
            observed=dict(seated=carry96["seated"], seatConfirmedS=carry96["beamProofS"],
                          sideBeltTakeoverS=carry96["sideBeltTakeoverS"]),
            control=dict(condition="Nose roller and belts unpowered", rejected=True,
                         evidence=_cited(EVIDENCE / "pickup-height-control-01/dead.json", enabled=dead["enabled"],
                                         seated=dead["seated"], maxRiseMm=dead["maxRiseMm"])),
            brackets=pickup_brackets,
            timestep=dict(timestep, note="The finer step also doubles the NOTE ring segments, 48 to 96.",
                          evidence=[_cited(CAMPAIGN / "dev-middle-48.json", seated=carry48["seated"]),
                                    _cited(CAMPAIGN / "dev-middle-96.json", seated=carry96["seated"])])))
    lift = dict(
        interaction="Retention of the seated NOTE through the loaded lift",
        basis="Recorded free-NOTE path relative to the commanded cassette through the 4.8 s lift.",
        mechanisms={"Piece support": "Seat, runners and rear stop bear the NOTE while the cassette is still",
                    "Metering drive": "Feed belts held stopped against the NOTE",
                    "Launch drive": "Launch belts held stopped ahead of the NOTE"},
        sources=dict(carry=sources["carry"], run=sources["run"], freeze=sources["freeze"]),
        limits=scope + " Retention did not fail in any bracket case; the bracket states the first mechanical limit reached.",
        qualification=dict(
            result="pass",
            observed=dict(held=carry96["held"], stopOvertravelDeg=cyc.part["stopOvertravelDeg"]),
            control=dict(condition="Cover leaf removed with the belts coasting and power off", rejected=True,
                         evidence=_cited(EVIDENCE / "pickup-lip/power-off-hold.json",
                                         withoutLeafOutwardMm=[r["peakOutwardMm"] for r in open_hold],
                                         withLeafOutwardMm=[r["peakOutwardMm"] for r in kept_hold])),
            brackets=[lift_bracket],
            timestep=dict(timestep, note="The finer step also doubles the NOTE ring segments, 48 to 96.",
                          evidence=[_cited(CAMPAIGN / "dev-middle-48.json", held=carry48["held"]),
                                    _cited(CAMPAIGN / "dev-middle-96.json", held=carry96["held"])])))
    operating, delivery_brackets = delivery_window()
    slow = [json.loads(line) for line in (WINDOW / "results/low-speed.jsonl").read_text().splitlines()]
    slow = [r for r in slow if r["speed"] == min(x["speed"] for x in slow)]
    assert slow and all("error" in r or not r["opening"] for r in slow)
    delivery = dict(
        result="pass",
        observed=dict(openingMarginMm=1000 * shot96["opening"]["minimumMarginM"], bothCountersS=shot96["both"],
                      minimumBusV=shot96["minBusV"], releaseSpeedMps=float(sum(v * v for v in shot96["release"]["v"][:3]) ** .5)),
        control=dict(condition=f"Launcher commanded to {slow[0]['speed']:g} m/s, half the released speed", rejected=True,
                     evidence=_cited(WINDOW / "results/low-speed.jsonl", shots=len(slow),
                                     passedThroughOpening=sum(1 for r in slow if r.get("opening"))),
                     note="A second control, the weak-pack firing permissive bypassed during drive acceleration, is "
                          "rejected in bounded-risk-01/s-bypass.json (bus " + f"{bypass['minBusV']:.2f} V)."),
        brackets=delivery_brackets,
        timestep=dict(timestep, note="The finer step also doubles the NOTE ring segments, 48 to 96.",
                      evidence=[_cited(CAMPAIGN / "split-tail_.25_0-dev-middle-48-6.88.json", bothCountersS=shot48["both"]),
                                _cited(CAMPAIGN / "split-tail_.25_0-dev-middle-96-6.88.json", bothCountersS=shot96["both"])]))
    shot_sources = dict(shot=sources["shot"], carry=sources["carry"], run=sources["run"], freeze=sources["freeze"])
    shot_limits = scope + (" A pass here is delivery: the whole NOTE through the speaker opening with the bus at or "
                           "above 8 V. Counting by the speaker's two beams is a separate result; see risk "
                           "scoring-repeatability. The brackets re-fire the carried NOTES of the frozen starts from the "
                           "firing station, one input at a time.")
    feed = dict(interaction="Feed from the metering belts into the launch belts",
                basis="Recorded free-NOTE path of the fine nominal shot trial from feed command to release.",
                mechanisms={"Metering drive": "Feed belts driven at the commanded feed speed",
                            "Launch drive": "Launch belts held at the 6.88 m/s command"},
                sources=shot_sources, limits=shot_limits, qualification=delivery)
    flight = dict(interaction="Free flight from release through the speaker opening",
                  basis="Recorded free-NOTE path of the fine nominal shot trial from release to full passage of the opening.",
                  mechanisms={}, sources=shot_sources, limits=shot_limits, qualification=delivery)
    return dict(acquire=acquire, lift=lift, feed=feed, flight=flight, operating=operating)


def climb():
    """Qualification of the guided climb from its bracket study."""
    rows = {r["label"]: r for r in read(CLIMB / "results.json")["cases"]}
    nominal, fine = rows["cg+0.04-dt0.00025-ramp6.5-normal"], rows["cg+0.04-dt0.000125-ramp6.5-normal"]
    rear, rear_fine = rows["cg-0.04-dt0.00025-ramp6.5-normal"], rows["cg-0.04-dt0.000125-ramp6.5-normal"]
    control = rows["cg+0.04-dt0.00025-ramp6.5-no-catch"]
    released = read(CAMPAIGN / "climb-gross-front.json")
    assert nominal["passed"] and fine["passed"] and rear["passed"] and rear_fine["passed"] and not control["passed"]
    assert nominal["disabledMinTireOrBumperM"] == released["disabledMinTireOrBumperM"]
    cg_ok, cg_bad = rows["cg+0.05-dt0.00025-ramp6.5-normal"], rows["cg+0.06-dt0.00025-ramp6.5-normal"]
    ramp_bad = rows["cg+0.04-dt0.00025-ramp5-normal"]
    assert cg_ok["passed"] and not cg_bad["passed"] and not ramp_bad["passed"]

    def mm(r):
        return 1000 * r["disabledMinTireOrBumperM"]
    rearmost = min((r for r in rows.values() if r["passed"]), key=lambda r: r["cg"])
    return dict(
        interaction="Chain capture, guided hoist, latch on the slide stops and disabled hold",
        basis="Recorded cassette angle, carriage draw, keeper gate and drum coil of the forward centre-of-gravity climb "
              "trial at the gross mass. The approach to the capture angle is commanded; the catch dog, clutch and "
              "stop-pin movements are drawn allocations.",
        sources=dict(climb=pin(CAMPAIGN / "climb-gross-front.json"), rearward=pin(CAMPAIGN / "climb-gross-rear.json"),
                     brackets=pin(CLIMB / "results.json")),
        limits="Rigid chain links and hooks, ideal slide stop pins and pitch lock, two hoist lines sharing load equally. "
               "The centre of gravity is unmeasured. A pass needs both hooks captured, the carriage latched before power "
               f"is cut, a bus at or above 8 V and {FLOOR_ALLOWANCE_MM:g} mm of floor clearance through the disabled hold.",
        qualification=dict(
            result="pass",
            observed=dict(captured=nominal["captured"], latched=nominal["latched"], floorClearanceMm=mm(nominal),
                          rearwardFloorClearanceMm=mm(rear), weakBusMinV=nominal["weakBusMin"],
                          peakCombinedLineN=nominal["maxCombinedCableN"], robotMassKg=nominal["modeledRobotMass"]),
            control=dict(condition="Catch disabled, so the carriage never latches on its stops", rejected=True,
                         evidence=_cited(CLIMB / "cg+0.04-dt0.00025-ramp6.5-no-catch.json", latched=control["latched"],
                                         floorLoadAfterPowerCutN=control["maxDisabledFloorN"])),
            brackets=[
                dict(input="centre of gravity ahead of nominal", unit="m", passing=cg_ok["cg"], failing=cg_bad["cg"],
                     note=f"Floor clearance in the disabled hold: {mm(cg_ok):.1f} mm at {1000 * cg_ok['cg']:.0f} mm and "
                          f"{mm(cg_bad):.1f} mm at {1000 * cg_bad['cg']:.0f} mm against {FLOOR_ALLOWANCE_MM:g} mm. Rearward "
                          f"the climb passes to {1000 * rearmost['cg']:.0f} mm, the furthest tried.",
                     evidence=pin(CLIMB / "results.json")),
                dict(input="hoist ramp duration", unit="s", passing=nominal["ramp"], failing=ramp_bad["ramp"],
                     note=f"A {ramp_bad['ramp']:g} s ramp latches but leaves {mm(ramp_bad):.1f} mm of floor clearance; ramps of "
                          "4 s and shorter do not latch and the robot returns to the carpet.",
                     evidence=pin(CLIMB / "results.json")),
            ],
            timestep=dict(coarseS=nominal["dt"], fineS=fine["dt"], sameOutcome=True,
                          note=f"Floor clearance {mm(nominal):.2f} mm at the coarse step and {mm(fine):.2f} mm at the fine "
                               f"step forward; {mm(rear):.2f} and {mm(rear_fine):.2f} mm rearward.",
                          evidence=[pin(CLIMB / "cg+0.04-dt0.00025-ramp6.5-normal.json"),
                                    pin(CLIMB / "cg+0.04-dt0.000125-ramp6.5-normal.json")])))


def objectives(case_phase):
    phases = set(case_phase.values())

    def shown(phase, text):
        return ("represented", text) if phase in phases else ("out-of-scope", "Not built in this partial run.")
    rows = [
        ("Approach a floor mark with the cassette lowered", *shown("reset", "The reset cases bring the empty cassette "
         "from the launch angle to the floor pickup pose on the commanded slide law.")),
        ("Seat the floor NOTE between the powered belts", *shown("acquire", "The acquire cases follow the recorded free "
         "NOTE over the lip to its seat and through the seat-confirmation dwell.")),
        ("Carry the NOTE while raising the mechanism", *shown("lift", "The lift cases follow the recorded NOTE on the "
         "commanded 4.8 s lift. Driving and turning during the lift are covered by the route screens, not by this check.")),
        ("Feed and deliver the NOTE through the speaker opening", *shown("flight", "The feed and flight cases follow the "
         "recorded NOTE from the feed command through full passage of the opening. Counter registration is recorded in "
         "the pinned shot trial.")),
        ("Six scores in 120 s on at least 19 of 20 frozen starts", "out-of-scope",
         "Not shown. A layout check cannot show a statistical target, and the simulation does not support it: every "
         "perturbed NOTE is delivered through the opening from the firing lane, but about nine in ten are counted by "
         "the speaker within 7 s. It is carried in the risk register as a bench test with pass and fail counts."),
        ("PARK", *shown("stow", "The stow case returns the empty cassette to the parked angle. The PARK route on the "
         "field is covered by the endgame route study.")),
        ("Solo ONSTAGE", *shown("hold", "The approach, capture, hoist and hold cases follow the recorded forward "
         "centre-of-gravity climb from the parked cassette to the disabled hold on the slide stops. The working chain "
         "and the stage are not in the layout solids; chain contact is in the pinned climb trial.")),
        ("Share one chain", "out-of-scope", "Depends on a partner robot. No shared-chain geometry is designed."),
        ("Receive an overhead NOTE", "out-of-scope", "The narrative leaves the entry pose undefined. No source "
         "pickup is designed."),
    ]
    return [dict(objective=o, disposition=d, basis=b) for o, d, b in rows]


REQUIRED = {
    "Four wheels": ["Front left tire", "Rear left tire", "Front right tire", "Rear right tire"],
    "Eight swerve motors": [f"Swerve {kind} motor · {corner}" for corner in ("Front left", "Rear left", "Front right", "Rear right")
                            for kind in ("drive", "steer")],
    "Control and distribution": ["roboRIO flat", "PDH", "radio", "main breaker", "Battery and terminal case"],
    "Pivot supports and shaft": ["Pivot A-frame · left", "Pivot A-frame · right", "Pivot torque shaft",
                                 "Pivot bearing · left", "Pivot bearing · right"],
    "Powered piece handling": ["Launch Belt motor · left", "Launch Belt motor · right", "Feed motor", "Nose motor"],
    "Passive cover": ["Windowed keeper panel", "Mouth retention flap", "Flap axle"],
    "Chain hooks and keeper": ["Chain hook · left", "Chain hook · right", "Common keeper yoke and sliding rack",
                               "Shared lift crosspiece"],
    "Pickup lip and load path": ["Replaceable pickup lip", "Pickup lip crossbar", "Cassette side beam extension · left",
                                 "Cassette side beam extension · right"],
    "Climb catch": ["Catch sector", "Catch receiver", "Catch dog", "Catch actuator and spring"],
}
LOAD_MEMBERS = {
    "Pivot motor": ["Pivot motor", "Pivot reducer"],
    "Slide motor": ["Slide motor"],
    "Four drive motors": [f"Swerve drive motor · {c}" for c in ("Front left", "Rear left", "Front right", "Rear right")],
    "Nose roller motor": ["Nose motor"],
    "Feed roller command": ["Feed motor"],
    "Feed and launch zone motors": ["Feed motor", "Launch Belt motor · left", "Launch Belt motor · right"],
    "Feed plus two launch drives": ["Feed motor", "Launch Belt motor · left", "Launch Belt motor · right"],
    "Shared pivot motor reeling both lines": ["Pivot motor", "Pivot reducer"],
}


def presentation(components, groups, member_group, rigid, duty_table, timelines, case_phase, register, operating):
    names = set(components)
    required = {role: ids for role, ids in REQUIRED.items()}
    assert all(set(ids) <= names for ids in required.values()), [i for ids in required.values() for i in ids if i not in names]
    members = {source: [n for n in ids if n in names] for source, ids in LOAD_MEMBERS.items()}
    return dict(
        title="Guided cassette · preliminary design 004",
        context="FRC 2024 proof of concept",
        architecture="Floor pickup over a removable lip into a pivoting cassette on a translating truck, a close-range "
                     "speaker shot from split feed and launch belts, and a two-hook guided climb on the same pivot drive",
        focus="Play the cycle in order: reset, acquire, lift, feed, flight, stow, then approach, capture, hoist and "
              "hold. The NOTE follows the recorded free-body trial and the climb follows its recorded trial; the cassette "
              "and truck follow their commanded laws. Then open the weight and voltage ledgers: 0.43 kg and 0.34 V are "
              "the smallest margins.",
        notes=[
            f"Firing lane: robot centre {operating['firingStation']['xM']:g} m from the alliance wall at y = "
            f"{operating['firingStation']['yM']:g} m, cassette at {operating['firingStation']['cassetteAngleDeg']:g} degrees. "
            f"The cycle trials fired from y = {operating['trialStation']['yM']:g} m; the single-shot campaign moved the lane "
            "50 mm toward the centre of the opening, where the side clearance is larger.",
            f"From the firing lane {operating['atFiringStation']['frozenStarts']['delivered']} of "
            f"{operating['atFiringStation']['frozenStarts']['shots']} perturbed NOTES pass through the opening and "
            f"{operating['atFiringStation']['frozenStarts']['counted']} are counted by the speaker within 7 s. Six scores in "
            "120 s on 19 of 20 starts is not shown; it is carried as a bench test.",
            "The 232 solids are the release 003 layout with repaired component names and one revision: both launch "
            "belt motors are raised 60 mm along the cassette normal on lengthened drive shafts, because the whole-cycle "
            "check found no clearance between their cans and the floor NOTE riding over the pickup lip. The pickup "
            "trial does not model the motors.",
            "NOTE paths and the cover leaf angle are recorded simulation output. Cassette angle, truck position and "
            "chassis motion are commanded laws. Each case states how far its straight path departs from the trace.",
            "Declared NOTE indentation limits are planning assumptions. No measured force or allowable is claimed.",
            "The upper inspection estimate of 56.270 kg includes a 3 to 6 kg unallocated design reserve.",
            "The layout STEP draws the cover leaf closed at the parked angle. The cam law of the trials holds it open "
            "above 102 degrees, and the cycle cases follow the cam law.",
        ],
        components=components, required=required, loadMembers=members,
        caseKinds={cid: "nominal" for cid in case_phase},
        caseTimelines=timelines, openingCase=next(iter(case_phase), None),
        uncoveredCases=[dict(id=r["id"], condition=r["condition"], reason=r["risk"]) for r in register if r.get("condition")])


def missions(case_phase, trials, cycle_ids, register):
    """Mission cases, the cycle map and the risk register."""
    def last(phase):
        ids = [c for c in cycle_ids if case_phase[c] == phase]
        return ids[-1] if ids else None
    # The publication check accepts four accomplishments: acquired, delivered, scored and climb-held.
    wanted = [("acquire", "acquired", "Seat the floor NOTE between the powered belts", "field-interaction-trial"),
              ("lift", "acquired", "Carry the NOTE while raising the mechanism", "field-interaction-trial"),
              ("feed", "delivered", "Feed and deliver the NOTE through the speaker opening", "field-interaction-trial"),
              ("flight", "delivered", "Feed and deliver the NOTE through the speaker opening", "field-interaction-trial"),
              ("hold", "climb-held", "Solo ONSTAGE", "operating-case")]
    cases = []
    for phase, verb, objective, kind in wanted:
        cid = last(phase)
        if cid and cid in trials:
            cases.append(dict(case=cid, kind=kind, accomplishes=verb, objective=objective, trial=trials[cid]))
    stage_of = dict(reset="reset", acquire="acquire", lift="retain", feed="transfer", flight="deliver", stow="park",
                    approach="endgame", capture="endgame", hoist="endgame", hold="endgame")
    cycle = {}
    for cid in cycle_ids:
        cycle.setdefault(stage_of[case_phase[cid]], dict(cases=[]))["cases"].append(cid)
    cycle.setdefault("service", dict(cases=["withdraw-service-pin"]))
    return dict(missionCases=cases, cycle=cycle, risks=[{k: v for k, v in r.items() if k != "condition"} for r in register])

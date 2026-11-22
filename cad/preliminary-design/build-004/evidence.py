"""Trial qualifications, narrative coverage, presentation text and the risk register of release 004.

Every qualification statement is checked against the evidence file it cites when the release is built:
the build stops if a cited file no longer shows the stated outcome. A qualification element with no
supporting trial is left out, so the readiness report names it as missing.
"""
from timeline import EVIDENCE, pin, read

CAMPAIGN = EVIDENCE / "counted-reset-power-02"


def _cited(path, **facts):
    """A pin plus the facts read from that file."""
    return dict(pin(path), observed=facts)


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
    sources = cyc.sources()
    scope = ("Free rigid NOTE on commanded cassette, truck and chassis motion. Carpet grip, tread compliance and "
             "field counter speed are assumed values awaiting measurement.")
    acquire = dict(
        interaction="Floor acquisition over the Profile A lip to the seat",
        basis="Recorded free-NOTE path of the fine nominal pickup trial, same centre and section orientation.",
        mechanisms={"Cassette drive": "Nose roller and centre belt driven at the commanded surface speed",
                    "Metering drive": "Feed belts driven at the commanded surface speed"},
        sources=dict(carry=sources["carry"], run=sources["run"], freeze=sources["freeze"]),
        limits=scope + " No single-input sensitivity bracket with a failing value exists for the pickup.",
        qualification=dict(
            result="pass",
            observed=dict(seated=carry96["seated"], seatConfirmedS=carry96["beamProofS"],
                          sideBeltTakeoverS=carry96["sideBeltTakeoverS"]),
            control=dict(condition="Nose roller and belts unpowered", rejected=True,
                         evidence=_cited(EVIDENCE / "pickup-height-control-01/dead.json", enabled=dead["enabled"],
                                         seated=dead["seated"], maxRiseMm=dead["maxRiseMm"])),
            timestep=dict(timestep, note="The finer step also doubles the NOTE ring segments, 48 to 96.",
                          evidence=[_cited(CAMPAIGN / "dev-middle-48.json", seated=carry48["seated"]),
                                    _cited(CAMPAIGN / "dev-middle-96.json", seated=carry96["seated"])])))
    lift = dict(
        interaction="Retention of the seated NOTE through the loaded lift",
        basis="Recorded free-NOTE path relative to the commanded cassette through the 4.8 s lift.",
        mechanisms={"Piece support": "", "Metering drive": "", "Launch drive": ""},
        sources=dict(carry=sources["carry"], run=sources["run"], freeze=sources["freeze"]),
        limits=scope + " No sensitivity bracket with a failing value exists for retention on the 4.8 s lift.",
        qualification=dict(
            result="pass",
            observed=dict(held=carry96["held"], stopOvertravelDeg=cyc.part["stopOvertravelDeg"]),
            control=dict(condition="Cover leaf removed with the belts coasting and power off", rejected=True,
                         evidence=_cited(EVIDENCE / "pickup-lip/power-off-hold.json",
                                         withoutLeafOutwardMm=[r["peakOutwardMm"] for r in open_hold],
                                         withLeafOutwardMm=[r["peakOutwardMm"] for r in kept_hold])),
            timestep=dict(timestep, note="The finer step also doubles the NOTE ring segments, 48 to 96.",
                          evidence=[_cited(CAMPAIGN / "dev-middle-48.json", held=carry48["held"]),
                                    _cited(CAMPAIGN / "dev-middle-96.json", held=carry96["held"])])))
    bracket = dict(input="cassette launch angle at its matched firing distance", unit="deg", passing=passing["angle"],
                   failing=failing["angle"],
                   note=f"Free-flight calculation over the recorded releases: worst opening margin "
                        f"{passing['worstMarginMm']:.1f} mm at {passing['angle']} deg and {passing['stationM']:.2f} m, "
                        f"{failing['worstMarginMm']:.1f} mm at {failing['angle']} deg and {failing['stationM']:.2f} m. "
                        "The firing distance changes with the angle, so this is not a single-input bracket.",
                   evidence=pin(EVIDENCE / "counter-drive-01/elevation-window.json"))
    delivery = dict(
        result="pass",
        observed=dict(openingMarginMm=1000 * shot96["opening"]["minimumMarginM"], bothCountersS=shot96["both"],
                      minimumBusV=shot96["minBusV"], releaseSpeedMps=float(sum(v * v for v in shot96["release"]["v"][:3]) ** .5)),
        control=dict(condition="Weak-pack firing permissive bypassed during drive acceleration", rejected=True,
                     evidence=_cited(EVIDENCE / "bounded-risk-01/s-bypass.json", passShot=bypass["passShot"],
                                     minimumBusV=bypass["minBusV"]),
                     note="Run on the earlier 125 deg station; it shows the kind of fault the trial rejects."),
        brackets=[bracket],
        timestep=dict(timestep, note="The finer step also doubles the NOTE ring segments, 48 to 96.",
                      evidence=[_cited(CAMPAIGN / "split-tail_.25_0-dev-middle-48-6.88.json", bothCountersS=shot48["both"]),
                                _cited(CAMPAIGN / "split-tail_.25_0-dev-middle-96-6.88.json", bothCountersS=shot96["both"])]))
    shot_sources = dict(shot=sources["shot"], carry=sources["carry"], run=sources["run"], freeze=sources["freeze"])
    shot_limits = scope + (" The nominal starts pass. Three of seven perturbed starts of the frozen campaign failed; "
                           "see risk scoring-repeatability.")
    feed = dict(interaction="Feed from the metering belts into the launch belts",
                basis="Recorded free-NOTE path of the fine nominal shot trial from feed command to release.",
                mechanisms={"Metering drive": "Feed belts driven at the commanded feed speed",
                            "Launch drive": "Launch belts held at the 6.88 m/s command"},
                sources=shot_sources, limits=shot_limits, qualification=delivery)
    flight = dict(interaction="Free flight from release through the speaker opening",
                  basis="Recorded free-NOTE path of the fine nominal shot trial from release to full passage of the opening.",
                  mechanisms={}, sources=shot_sources, limits=shot_limits, qualification=delivery)
    return dict(acquire=acquire, lift=lift, feed=feed, flight=flight)


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
         "A statistical target cannot be shown by a layout check. The frozen simulation campaign failed it at the "
         "6.88 m/s command. It is carried in the risk register as a bench test with pass and fail counts."),
        ("PARK", *shown("stow", "The stow case returns the empty cassette to the parked angle. The PARK route on the "
         "field is covered by the endgame route study.")),
        ("Solo ONSTAGE", "out-of-scope", "The guided climb cases are not yet in this build of the release."),
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


def presentation(components, groups, member_group, rigid, duty_table, timelines, case_phase):
    names = set(components)
    required = {role: ids for role, ids in REQUIRED.items()}
    assert all(set(ids) <= names for ids in required.values()), [i for ids in required.values() for i in ids if i not in names]
    members = {source: [n for n in ids if n in names] for source, ids in LOAD_MEMBERS.items()}
    return dict(
        title="Guided cassette · preliminary design 004",
        context="FRC 2024 proof of concept",
        architecture="Floor pickup over a removable lip into a pivoting cassette on a translating truck, a close-range "
                     "speaker shot from split feed and launch belts, and a two-hook guided climb on the same pivot drive",
        focus="Play the cycle in order: reset, acquire, lift, feed, flight. The NOTE follows the recorded free-body "
              "trial; the cassette and truck follow their commanded laws. Then open the weight and voltage ledgers: "
              "0.43 kg and 0.34 V are the smallest margins.",
        notes=[
            "The 232 solids are the release 003 layout with repaired component names. No geometry changed.",
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
        uncoveredCases=[dict(id=r["id"], condition=r["condition"], reason=r["risk"]) for r in RISKS if r.get("condition")])


# Open risks. Each is closed by a named design change or carried as a bench test with pass and fail values.
RISKS = [
    dict(id="scoring-repeatability", condition="Twenty perturbed starts of the six-NOTE mission",
         risk="Six scores in 120 s on 19 of 20 starts. In the frozen simulation campaign at the 6.88 m/s command three of "
              "seven perturbed starts failed (R02, R03 and R06) and thirteen were not run.",
         benchTest=dict(measure="Starts out of 20 on a full field speaker that score six floor NOTES within 120 s",
                        **{"pass": 19, "fail": 18}, unit="starts")),
]


def missions(case_phase, trials, cycle_ids):
    """Mission cases, the cycle map and the risk register."""
    def last(phase):
        ids = [c for c in cycle_ids if case_phase[c] == phase]
        return ids[-1] if ids else None
    wanted = [("acquire", "seated", "Seat the floor NOTE between the powered belts"),
              ("lift", "carried", "Carry the NOTE while raising the mechanism"),
              ("feed", "fed", "Feed and deliver the NOTE through the speaker opening"),
              ("flight", "delivered", "Feed and deliver the NOTE through the speaker opening")]
    cases = []
    for phase, verb, objective in wanted:
        cid = last(phase)
        if cid and cid in trials:
            cases.append(dict(case=cid, kind="operating-case", accomplishes=verb, objective=objective, trial=trials[cid]))
    stage_of = dict(reset="reset", acquire="acquire", lift="retain", feed="transfer", flight="deliver", stow="park")
    cycle = {}
    for cid in cycle_ids:
        cycle.setdefault(stage_of[case_phase[cid]], dict(cases=[]))["cases"].append(cid)
    cycle.setdefault("service", dict(cases=["withdraw-service-pin"]))
    return dict(missionCases=cases, cycle=cycle, risks=[{k: v for k, v in r.items() if k != "condition"} for r in RISKS])

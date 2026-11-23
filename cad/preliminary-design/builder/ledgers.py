"""Weight and loaded-voltage ledgers of Guided cassette 1.0.

Every figure is read from a pinned evidence file at build time. The mass lines are the draft layout
ledger with its component names repaired, plus the mass of any solid the revision changed.
"""
import json

from timeline import EVIDENCE, INPUTS, REPO, pin, read

ISSUED_BUDGETS = INPUTS / "draft-budgets.json"
CAMPAIGN = EVIDENCE / "counted-reset-power-02"
GROSS = EVIDENCE / "mounted-slide-02/gross-power.json"
RESET = CAMPAIGN / "reset-budget.json"
CLOCK = CAMPAIGN / "endgame-clock.json"
CLIMB = {side: CAMPAIGN / f"climb-gross-{side}.json" for side in ("front", "rear")}
NOTE_IDS = ("middle", "outer", "inner", "center-high", "center-mid", "center-low")
INSPECTION_LIMIT_KG = 56.69904625     # 125 lb kickoff inspection limit
OPERATING_MIN_V = 8.0
PACKS = dict(weak=dict(openCircuitV=11.8, resistanceOhm=0.027), nominal=dict(openCircuitV=12.6, resistanceOhm=0.018))


def repaired(text):
    return text.replace("Â·", "·")


STEEL_KG_PER_MM3 = 7.85e-6


def masses(revision):
    """The issued mass lines carried to this revision, the running payload interval and the gross study.

    The lines start from the draft layout ledger. A solid whose volume the revision changed adds its
    volume difference at its material density to the line that holds it, so the ledger follows the geometry.
    `revision` is the record written with the layout STEP; every changed solid here is a steel shaft or a motor
    whose volume did not change.
    """
    issued = json.loads(ISSUED_BUDGETS.read_bytes())
    lines = []
    for r in issued["masses"]:
        basis = r["basis"]
        if basis.startswith("Published revision 002"):
            basis = "Draft layout subtotal, preserving additions and incremental stock allowances."
        lines.append(dict(id=repaired(r["id"]), members=[repaired(m) for m in r["members"]], kg=list(r["kg"]), basis=repaired(basis)))
    gross = read(GROSS)
    assert abs(sum(r["kg"][1] for r in lines) - gross["inspectionHighKg"]) < 1e-7      # the issued ledger, before the revision
    added = 0.0
    for name, change in revision["changed"].items():
        before, after = change["volumeMm3"]
        if abs(after - before) < 1e-3:
            continue
        assert "shaft" in name, f"No density is recorded for the changed solid {name}"
        kg = (after - before) * STEEL_KG_PER_MM3
        line = next(r for r in lines if name in r["members"])
        line["kg"] = [line["kg"][0] + kg, line["kg"][1] + kg]
        line["basis"] += (f" Revision {revision['revision']} adds {kg:.4f} kg for {name}: {after - before:.0f} mm3 of"
                          " added solid taken as steel.")
        added += kg
    revised = dict(gross, inspectionHighKg=gross["inspectionHighKg"] + added, runningKg=gross["runningKg"] + added,
                   inspectionMarginKg=gross["inspectionMarginKg"] - added,
                   revision=dict(id=revision["revision"], addedKg=added, issuedInspectionHighKg=gross["inspectionHighKg"],
                                 basis="Volume added to each changed solid at steel density, on the issued gross study",
                                 source=pin(GROSS)))
    return lines, issued["runningPayloadKg"], revised


def duties():
    """Battery-side load sets by duty: {duty: dict(text, sources {name: [low, high] A}, requiredV, basis, pins)}."""
    gross = read(GROSS)
    lift = min((c for c in gross["cases"] if c["auxA"] == 25), key=lambda c: c["weakBusMinV"])
    assert abs(lift["weakBusMinV"] - gross["loaded25A"]["bus"]) < 1e-9
    reset = read(RESET)["cases"]["35"]
    shots = []
    for n in (48, 96):
        for note in NOTE_IDS:
            shot = read(CAMPAIGN / f"split-tail_.25_0-dev-{note}-{n}-6.88.json")
            assert abs(shot["peakPackA"] - shot["estimatedPeakDriveA"] - 27) < 1e-9
            shots.append(shot["estimatedPeakDriveA"])
    carry = read(CAMPAIGN / "dev-middle-96.json")
    climb = max(read(f)["peakBatteryA"] for f in CLIMB.values())
    clock = read(CLOCK)
    drive_leg = max((PACKS["weak"]["openCircuitV"] - row["weakBusBoundDuringFirstLeg"]) / PACKS["weak"]["resistanceOhm"]
                    for row in clock)
    b_lift, b_reset = lift["batteryBreakdownAtPeakA"], reset["batteryBreakdownAtPeakA"]
    standby = "Controls, sensors and standby loads"
    table = {
        "reset": dict(
            text="Empty cassette return with the truck moving",
            sources={standby: [27, 27], "Feed roller command": [0, 8], "Pivot motor": [0, b_reset["pivot"]],
                     "Slide motor": [0, b_reset["slide"]], "Four drive motors": [0, b_reset["fourDrives"]]},
            requiredV=reset["weakBusMinV"] - reset["minimumMotorVoltageReserveV"],
            basis="Empty-return study with 35 A of other loads held through the whole reset. The required voltage is "
                  "the bus at which the pivot motor reaches its terminal demand.",
            pins=[pin(RESET)]),
        "acquire": dict(
            text="Floor pickup while advancing on the NOTE",
            sources={standby: [27, 27], "Nose roller motor": [0, carry["peakPhaseA"]],
                     "Feed and launch zone motors": [0, sum(carry["peakZonePhaseA"])],
                     "Four drive motors": [0, b_lift["fourDrives"]]},
            requiredV=OPERATING_MIN_V,
            basis="Phase-current limits of the pickup trial taken as battery-side upper bounds. The drive bound is "
                  "borrowed from the loaded-lift study; no pickup-specific drive current was recorded.",
            pins=[pin(CAMPAIGN / "dev-middle-96.json"), pin(GROSS)]),
        "lift": dict(
            text="Loaded lift with the truck moving and the robot driving",
            sources={"Controls and other loads, staged to 25 A": [25, 25], "Four drive motors": [0, b_lift["fourDrives"]],
                     "Pivot motor": [0, b_lift["pivot"]], "Slide motor": [0, b_lift["slide"]]},
            requiredV=lift["worstPitchV"],
            basis="Governing loaded-lift case of the six NOTES at the gross running mass, with other loads staged to 25 A. "
                  "The required voltage is the pivot motor terminal demand at the governing instant.",
            pins=[pin(GROSS)]),
        "feed": dict(
            text="Feed and launch with concurrent accessories",
            sources={standby: [27, 27], "Feed plus two launch drives": [0, max(shots)]},
            requiredV=OPERATING_MIN_V,
            basis="Largest feed and launch battery current of the twelve nominal shots at both numerical resolutions.",
            pins=[pin(CAMPAIGN / "run-dev-48.json"), pin(CAMPAIGN / "run-dev-96.json")]),
        "pivot-empty": dict(
            text="Empty cassette pivot move",
            sources={standby: [27, 27], "Pivot motor": [0, b_reset["pivot"]]},
            requiredV=OPERATING_MIN_V,
            basis="Pivot current bound of the empty-return study applied to a shorter unloaded move.",
            pins=[pin(RESET)]),
        "drive": dict(
            text="Drive from the speaker to the stage",
            sources={"All battery loads on the first drive leg": [0, drive_leg]},
            requiredV=OPERATING_MIN_V,
            basis="Weak-pack bus bound of the first endgame drive leg, expressed as its equivalent total current.",
            pins=[pin(CLOCK)]),
        "climb": dict(
            text="Chain capture and guided hoist",
            sources={"Controls and concurrent loads": [25, 25], "Shared pivot motor reeling both lines": [0, climb - 25]},
            requiredV=OPERATING_MIN_V,
            basis="Peak battery current of the forward and rearward centre-of-gravity climb trials at 71.764 kg.",
            pins=[pin(f) for f in CLIMB.values()]),
        "hold": dict(
            text="Disabled hold on the mechanical stops",
            sources={"Controls and concurrent loads": [25, 25]},
            requiredV=OPERATING_MIN_V,
            basis="Motor power is cut. The climb trials keep 25 A of other loads on the bus.",
            pins=[pin(f) for f in CLIMB.values()]),
    }
    return table

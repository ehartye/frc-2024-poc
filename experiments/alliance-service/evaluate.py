"""Join the matched field runs to the released passive service and load screens."""
import argparse
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SOURCES = {
    "stage_rules": "kickoff/stage-rules.json",
    "field": "kickoff/crescendo-field.json",
    "study": "experiments/alliance-service/roles.json",
    "roles": "experiments/alliance-service/role-results.json",
    "mass": "cad/assembly-readiness/mass/result.json",
    "mast": "experiments/stage-climb/static-screen.json",
    "hold": "experiments/stage-climb/solo-trajectory.json",
    "service": "cad/service-readiness/reopen.json",
    "receiving": "cad/service-readiness/r11-reopen.json",
}


def load_inputs():
    inputs, hashes = {}, {}
    for name, filename in SOURCES.items():
        data = (ROOT / filename).read_bytes()
        inputs[name] = json.loads(data)
        hashes[filename] = hashlib.sha256(data).hexdigest()
    for source, field in [("field", "profileSha256"), ("study", "studySha256")]:
        if inputs["roles"][field] != hashes[SOURCES[source]]:
            raise ValueError(f"Rerun field study after changing {source}")
    return inputs, hashes


def kickoff(stage):
    # The pinned V0 comparison independently verifies these three table entries.
    points = stage["matchPoints"]
    park, hang = points["park"], points["onstage"]
    harmony = points["harmonyPerAdditionalRobotOnSameChain"]
    cases = []
    for partner in (False, True):
        background = hang if partner else 0
        success = hang + (harmony if partner else 0)
        cases.append({
            "partnerOnSameUnlitChain": partner,
            "parkAlliancePoints": background + park,
            "successfulHangAlliancePoints": background + success,
            "missOutsideZoneAlliancePoints": background,
            "breakEvenHangProbability": park / success,
            "sensitivityAtHypothetical25Percent": background + 0.25 * success,
        })
    return {"authority": stage["authority"]["kickoffComparison"], "cases": cases}


def elevation(inputs):
    mast = inputs["mast"]
    demand = mast["loads"]["designForceN"]
    service = inputs["service"]
    # An opposed bearing pair reacts to opposite directions. Each comparison
    # uses one bearing. Basic dynamic ratings require a separate life analysis.
    ratings = {
        "nutMaximumDynamicN": 300 * 4.4482216152605,
        "oneBearingBasicDynamicN": 2160,
        "oneBearingBasicStaticN": 2440,
    }
    hold = inputs["hold"]["outcome"]
    shared = next(s for s in mast["stageChecks"] if s["seatOffsetMm"] == 500)
    return {
        "actualInstalledKg": inputs["mass"]["installedRangeKg"],
        "knownInstalledSubtotalKg": inputs["mass"]["knownInstalledRangeKg"],
        "unresolvedArticles": inputs["mass"]["unresolvedInstancesCount"],
        "earlierScenarioUpperKg": mast["upperMassKg"],
        "scenarioStaticN": mast["loads"]["upperStaticN"],
        "scenarioDynamicMultiplier": mast["loads"]["designFactor"],
        "scenarioScreenN": demand,
        "candidateCatalogRatings": ratings,
        "nutRatingMinusDemandN": ratings["nutMaximumDynamicN"] - demand,
        "oneBearingDynamicRatingToDemand": ratings["oneBearingBasicDynamicN"] / demand,
        "oneBearingStaticRatingToStaticDemand": ratings["oneBearingBasicStaticN"] / mast["loads"]["upperStaticN"],
        "assumedDoubleSupportStressMPa": {
            key: value for key, value in mast["loads"].items() if key.endswith("MPa")
        },
        "singleWallAverageBearingDemandMPa": 2 * mast["loads"]["innerTubeBearingMPa"],
        "adverseSharedSeatTensionDifferenceN": shared["linkTensionDifferenceN"],
        "displacedGuideOverlapMm3": service["offsetCarrierGuideControlMm3"],
        "floorGapMmAtFiveSeconds": {
            name: case["atFiveSeconds"]["floorClearanceMm"] for name, case in hold.items()
        },
        "actualAssemblyCapacityN": None,
        "actualAssemblyMarginN": None,
    }


def compare(inputs):
    rows = {}
    for cell in inputs["roles"]["cells"]:
        period = next(p for p in cell["periods"] if p["phase"] == "TELEOP")
        focal = next(r for r in period["robots"] if r["id"] == "B3")
        acquisitions = [e for e in cell["events"] if e.get("robot") == "B3"
                        and e.get("operation") == "acquire" and e.get("outcome") == "acquired"]
        rows[cell["context"], cell["role"]] = {
            "waitSeconds": round(focal["waitSeconds"], 3),
            "finishedPrescribedVisits": focal["visits"],
            "acquiredPieces": sum(len(e["pieces"]) for e in acquisitions),
            "fullMatchMargin": cell["margin"],
        }
    results = {}
    for context in ("separate-lanes", "shared-lane"):
        clear = context == "separate-lanes"
        for concept, role in [("fixed-retrieval-and-hang", "retrieve"),
                              ("grounded-flex-support", "retrieve" if clear else "yield")]:
            row = dict(rows[context, role])
            row.update(concept=concept, case=context, runId=f"{concept}/{context}", simulatedRole=role)
            pack = max(inputs["receiving"]["receivingGaugeParkOverlapMm3"].values())
            row["serviceInterferenceMm3"] = pack
            if concept == "fixed-retrieval-and-hang":
                guide = "carrierGuideSweepMm3" if clear else "offsetCarrierGuideControlMm3"
                row["serviceInterferenceMm3"] = max(pack, inputs["service"][guide])
                hold = inputs["hold"]["outcome"]["engaged" if clear else "missed"]
                row["terminalFloorClearanceMm"] = hold["atFiveSeconds"]["floorClearanceMm"]
            results.setdefault(concept, {})[context] = row
    return results


def save(filename, data):
    path = HERE / filename
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n",
                    encoding="utf8", newline="\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="Verify the tracked receipt in place")
    args = parser.parse_args()
    inputs, hashes = load_inputs()
    result = {
        "sourceSha256": hashes,
        "kickoffOnly": kickoff(inputs["stage_rules"]),
        "elevation": elevation(inputs),
        "concepts": compare(inputs),
        "physicalMatchAvailability": None,
    }
    if args.check:
        if result != json.loads((HERE / "season-measurements.json").read_bytes()):
            raise SystemExit("Stale season measurements; replay the matched study.")
    else:
        save("season-measurements.json", result)

"""Report the 2024 ENSEMBLE qualification scenario separately from match points."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def report(series, rules):
    result = []
    threshold = rules["qualificationRanking"]["ensemble"]
    for match in series["matches"]:
        game = match["game"]
        end = game["endgame"]
        for side in ("blue", "red"):
            attempts = [a for a in end["attempts"] if a["side"] == side]
            onstage = sum(a["awardedAction"] == "onstage" for a in attempts)
            trap_stations = {event["station"] for event in game["events"] if event["robot"][0] == side[0].upper()
                and event["operation"] == "attempt" and event["outcome"] == "credited" and "-trap" in event["station"]}
            traps = len(trap_stations)*rules["matchPoints"]["notePerTrap"]
            stage = end["terminalScore"][side] + traps
            earned = onstage >= threshold["minOnstageRobots"] and stage >= threshold["minStageMatchPoints"]
            result.append(dict(match=match["id"],alliance=side,ordinaryMatchPoints=end["ordinaryScore"][side],
                terminalMatchPoints=end["terminalScore"][side],trapMatchPointsAlreadyInOrdinary=traps,
                totalMatchPoints=game["score"][side],totalStageMatchPoints=stage,onstageRobots=onstage,
                modeledEnsembleRP=threshold["rankingPoints"] if earned else 0,refereeOverride="outside scenario"))
    return result


if __name__ == "__main__":
    series = json.loads((ROOT/"experiments/speaker-cycles/match-series.json").read_bytes())
    rules = json.loads((ROOT/"kickoff/stage-rules.json").read_bytes())
    result = report(series,rules)
    print(json.dumps(result,indent=2))

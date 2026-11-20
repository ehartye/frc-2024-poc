"""Season-level outcomes that a static map or a stock counter cannot verify."""
import hashlib
import json
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
PROFILE=ROOT/"kickoff/physical-match.json"
SERIES=ROOT/"experiments/physical-match/series.json"
def read(path):
    return json.loads(path.read_bytes())

class PhysicalMatch(unittest.TestCase):
    def test_openings_ownership_and_score(self):
        result=read(SERIES)
        self.assertEqual(result["profileSha256"],hashlib.sha256(PROFILE.read_bytes()).hexdigest())
        for m in result["matches"]:
            game=m["game"]
            robots={r["id"] for r in m["roster"]}
            self.assertEqual(robots,{"B1","B2","B3","R1","R2","R3"})
            acquisitions={e["robot"]:e["pieces"][0] for e in game["events"] if e["phase"]=="AUTO" and e["outcome"]=="acquired"}
            self.assertEqual(acquisitions,{k:v["piece"] for k,v in game["opening"].items()})
            self.assertEqual(set(acquisitions),robots)
            self.assertEqual(len(set(acquisitions.values())),6)
            expected={f"{station}/{i+1}" for station,n in game["initial"].items() for i in range(n)}
            self.assertEqual(len(expected),107)
            owners=[p["id"] for pool in ("stock","reserve","cargo") for pieces in game[pool].values() for p in pieces]
            owners.extend(p["id"] for p in game["terminal"])
            owners.extend(r["piece"]["id"] for r in game["returning"])
            self.assertCountEqual(owners,expected)
            self.assertEqual({p["id"] for p in game["motion"]["pieces"]},
                             {p["id"] for pieces in game["stock"].values() for p in pieces})
            scored=set()
            for e in game["events"]:
                if e.get("points",0)>0:
                    self.assertEqual(e["outcome"],"credited")
                    self.assertEqual(len(e["pieces"]),1)
                    self.assertTrue(set(e["pieces"])<=expected)
                    self.assertFalse(scored.intersection(e["pieces"]))
                    scored.update(e["pieces"])
            for side in ("blue","red"):
                ordinary=sum(e.get("points",0) for e in game["events"] if e["robot"].startswith(side[0].upper()))
                self.assertEqual(game["score"][side],ordinary+game["endgame"]["terminalScore"][side])

    def test_source_pacing_and_physical_handoff(self):
        runs=read(SERIES)["matches"]
        for m in runs:
            outlets={}
            for e in m["game"]["events"]:
                if e["operation"]=="release":
                    self.assertEqual(e["phase"],"TELEOP")
                    self.assertGreaterEqual(e["timeS"],4)
                    station=e["station"]
                    self.assertNotIn(station,outlets)
                    outlets[station]=(e["timeS"],e["pieces"][0])
                if e["outcome"]=="acquired" and e["station"].endswith("-source"):
                    prior=outlets.pop(e["station"])
                    self.assertEqual(e["pieces"],[prior[1]])
            self.assertTrue(all(len(pieces)<=1 for pieces in m["game"]["cargo"].values()))
        events=runs[0]["game"]["events"]
        for side in ("blue","red"):
            relay_id=side+"-inner/1"
            self.assertTrue(any(e["station"]==side+"-deposit" and e["pieces"]==[relay_id] and e["outcome"]=="credited" for e in events))
            self.assertTrue(any(e["station"]==side+"-exchange" and e["pieces"]==[relay_id] and e["outcome"]=="acquired" for e in events))
        releases=[e["timeS"] for e in events if e["robot"]=="human/blue-source-player"]
        self.assertGreaterEqual(len(releases),2)
        self.assertGreaterEqual(min(b-a for a,b in zip(releases,releases[1:])),4)

if __name__=="__main__":
    unittest.main()

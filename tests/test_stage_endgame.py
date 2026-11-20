"""Keep the public robot, the disabled support trace and the six-robot score distinct."""
import hashlib
import json
from pathlib import Path
import struct
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"experiments/stage-climb"))
from rankings import report


def read(path):
    return json.loads((ROOT/path).read_bytes())


class StageEndgame(unittest.TestCase):
    def test_match_and_ranking_accounts(self):
        series=read("experiments/speaker-cycles/match-series.json")
        rules=read("kickoff/stage-rules.json")
        self.assertEqual(series["profileSha256"],hashlib.sha256((ROOT/"kickoff/crescendo-field.json").read_bytes()).hexdigest())
        rows=report(series,rules)
        self.assertEqual([(r["totalMatchPoints"],r["modeledEnsembleRP"]) for r in rows],
                         [(17,0),(22,0),(26,1),(19,0),(27,0),(19,0)])
        shared=series["matches"][1]["game"]["endgame"]
        self.assertEqual([b["points"] for b in shared["bonusAwards"] if b["side"]=="blue"],[2,2])
        self.assertEqual([m["game"]["endgame"]["assessmentTimeS"] for m in series["matches"]],[155,152.5,155])
        for attempt in series["matches"][2]["game"]["endgame"]["attempts"]:
            if attempt["robot"]=="R2":
                self.assertEqual((attempt["outcome"],attempt["awardedAction"],attempt["points"]),("failed","park",1))
        shared["terminalScore"]["blue"]=9
        self.assertEqual(report(series,rules)[2]["modeledEnsembleRP"],0)
        shared["terminalScore"]["blue"]=10
        next(r for r in shared["attempts"] if r["robot"]=="B1")["awardedAction"]=None
        self.assertEqual(report(series,rules)[2]["modeledEnsembleRP"],0)

    def test_reopened_archive_replay_and_failed_lock(self):
        receipt=read("cad/student/r8-reopened.json")
        screen=read("experiments/stage-climb/static-screen.json")
        trace=read("experiments/stage-climb/solo-trajectory.json")
        raw=(ROOT/"docs/media/r8-stage.glb").read_bytes()
        header=json.loads(raw[20:20+struct.unpack_from("<I",raw,12)[0]])
        digest=hashlib.sha256((ROOT/receipt["source"]).read_bytes()).hexdigest()
        self.assertEqual({digest,receipt["sha256"],screen["nativeSha256"],trace["nativeSha256"],header["asset"]["extras"]["nativeSha256"]},{digest})
        self.assertEqual(len(receipt["railWallChecksMm"]),8)
        self.assertEqual(header["animations"][0]["name"],"Take the Stage")
        self.assertEqual(screen["soloChainPath"]["possibleContacts"],[])
        self.assertTrue(all(pair[1]=="Swerve frame - interchangeable drilled rail" for pair in screen["internalSweepBoxOverlaps"]))
        for key in ("atAssessment","atFiveSeconds"):
            locked=trace["outcome"]["engaged"][key]
            failed=trace["outcome"]["missed"][key]
            self.assertFalse(locked["powered"] or failed["powered"])
            self.assertGreater(locked["floorClearanceMm"],100)
            self.assertGreater(locked["lockLoadN"],490)
            self.assertAlmostEqual(failed["floorClearanceMm"],0,places=5)
            self.assertEqual(failed["lockLoadN"],0)
        self.assertTrue(all(s["minRadioToClampMm"]>0 for s in read("experiments/stage-climb/clamp-clearance.json")["samples"]))


if __name__=="__main__":
    unittest.main()

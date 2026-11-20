"""Cross-check project custody history against frozen focal receipts and the student replay."""
import hashlib
import json
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
STUDY = ROOT / "experiments/ground-relay"


def read(path):
    return json.loads(path.read_bytes())


class GroundRelay(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cases = read(ROOT / "docs/ground-relay.json")["cases"]

    def test_source_identity(self):
        for name, expected in read(STUDY / "sources.json").items():
            self.assertEqual(hashlib.sha256((ROOT / name).read_bytes()).hexdigest(), expected, name)
        footprint = read(STUDY / "footprint.json")
        native = read(ROOT / "cad/service-readiness/stiffness/native-fit.json")
        self.assertEqual(footprint["fixtureSha256"], native["archiveSha256"])
        self.assertEqual(footprint["selectedBodies"], native["fullParkedVisibleBodies"])
        self.assertEqual(len(set(footprint["extremalBodies"])), 6)

    def test_each_receipt_has_a_prior_physical_state_in_the_model(self):
        transitions = {
            "acquired": ("source", "B3", "B3"),
            "ground release": ("B3", "settling", "B3"),
            "floor deposit": ("settling", "pad", "B3"),
            "received": ("pad", "B2", "B2"),
            "downstream floor": ("B2", "stored", "B2"),
            "lost": ("B3", "lost", "B3"),
            "outside pad": ("settling", "lost", "B3"),
            "empty after traffic sweep": ("source", "unlocated", "B3"),
        }
        summaries = read(STUDY / "custody-summary.json")
        for case, run in self.cases.items():
            with self.subTest(case=case):
                self.assertEqual(run, summaries[case] | {"history": run["history"], "samples": run["samples"]})
                state = dict.fromkeys(run["initialPositions"], "source")
                previous = 15
                for event in run["history"]:
                    self.assertLessEqual(previous, event["t"])
                    self.assertLess(event["t"], 150)
                    previous = event["t"]
                    if event["kind"] not in transitions:
                        continue
                    start, end, actor = transitions[event["kind"]]
                    self.assertEqual(event["actor"], actor)
                    self.assertEqual(state[event["piece"]], start)
                    if end in ("B3", "B2"):
                        self.assertNotIn(end, state.values())
                    if event["kind"] == "acquired":
                        self.assertTrue(event["freshLocalDetection"])
                        self.assertLessEqual(event["residualIn"], 1.5)
                    state[event["piece"]] = end
                self.assertEqual(state, run["finalCustody"])
                self.assertEqual(run["matchPoints"], {"B3": 0, "B2": 0})

    def test_frozen_denominators_and_focal_credit(self):
        for policy, passing, count in (("serial", 2, 3), ("buffer", 5, 5)):
            result = read(STUDY / (policy + "-results.json"))
            executions = read(STUDY / (policy + "-executions.json"))
            self.assertTrue(result["independentlyCheckedFreeze"])
            self.assertEqual((result["passingRuns"], result["qualifiedRuns"]), (passing, count))
            for episode in result["episodes"]:
                replay = self.cases[policy + "/" + episode["id"]]
                receipt = executions[episode["id"]]
                accepted = [e for e in receipt["events"] if e["kind"] == "accept"]
                self.assertEqual([e["piece"] for e in accepted], replay["deposits"])
                self.assertEqual([e["timeS"] for e in accepted],
                                 [e["t"] for e in replay["history"] if e["kind"] == "floor deposit"])
                self.assertEqual(episode["accepted"], len(accepted))
                self.assertTrue(all(e["focalRobot"] == "B3" and e["station"] == "blue-exchange"
                                    for e in accepted))
                self.assertGreater(replay["carryMarginN"], 0)
                self.assertLessEqual(replay["peakLoadedAccelInS2"], 42)
                self.assertGreater(replay["minimumGapIn"], 0)

    def test_receiver_cutoff_and_failures(self):
        late = self.cases["buffer/slow-receiver"]
        direct = self.cases["diagnostic/direct-dispatch"]
        self.assertEqual(late["finalCustody"]["center-1/1"], "pad")
        self.assertEqual(direct["finalCustody"]["center-1/1"], "B2")
        self.assertFalse(direct["partnerReset"])
        for case, count in (("empty", 0), ("zeroRetention", 0), ("dropOutside", 0),
                            ("sweptAway", 1), ("unreserved", 0), ("oneInch", 1)):
            self.assertEqual(len(self.cases["control/" + case]["deposits"]), count, case)


if __name__ == "__main__":
    unittest.main()

"""Regression checks for article registration, per-piece custody, and the credited actor."""
import hashlib
import json
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
STUDY = ROOT / "experiments/registered-custody"


def read(path):
    return json.loads(path.read_bytes())


class RegisteredCustody(unittest.TestCase):
    def test_body_registration_and_control_signs(self):
        members = read(STUDY / "cradle-bounds.json")
        bill = read(ROOT / "cad/assembly-readiness/mass/actual.json")
        footprint = read(ROOT / "experiments/ground-relay/footprint.json")
        self.assertEqual(bill["archiveSha256"], footprint["sourceParkedArchiveSha256"])
        names = {r["body"] for r in bill["bodies"]} - set(footprint["retiredFromR9"])
        self.assertEqual(len(names), 909)
        self.assertEqual(len(members),34)
        self.assertTrue(set(members) <= names)
        evidence = read(STUDY / "contact.json")
        self.assertFalse(evidence["externalDock"])
        for tread in ("534","535"):
            self.assertLess(evidence["floorSeat"][tread][1],-1.3)
        self.assertGreater(evidence["offset3mm"]["534"][0],1.5)
        self.assertLess(evidence["raisedStroke"]["R9 bumper cover envelope / R7 bumper"][1],-.7)
        self.assertTrue(all(pair[0]>0 for name,pair in evidence["floorStroke"].items()
                            if name not in ("534","535")))

    def test_passive_release_depends_on_drag(self):
        coarse, medium, fine = read(STUDY / "loads.json")
        for observation in (coarse,medium,fine):
            force = observation["withdrawalN"]
            self.assertLess(force["free"],observation["floorLimitN"])
            self.assertGreater(force["braked"],observation["floorLimitN"])
            self.assertGreater(observation["tractionN"],observation["weightN"])
        self.assertLess(abs(medium["halfStepWithdrawalN"]["free"]-fine["halfStepWithdrawalN"]["free"]),.005)

    def test_custody_chain_and_reference_throughput(self):
        runs=read(STUDY/"results.json")
        self.assertEqual(runs,read(ROOT/"docs/ground-contact.json")["results"])
        prior=read(ROOT/"experiments/ground-relay/custody-summary.json")
        chain={"bilateral floor seat":("source","B3","B3"),
               "floor support at pad":("B3","supported","B3"),
               "clear floor deposit":("supported","pad","B3"),
               "fixture receiver pickup":("pad","B2","B2"),
               "downstream store":("B2","stored","B2")}
        for name,run in runs.items():
            with self.subTest(case=name):
                last=15
                state=dict.fromkeys(run["custody"],"source")
                count=[]
                for event in run["history"]:
                    self.assertLessEqual(last,event["timeS"])
                    self.assertLess(event["timeS"],150)
                    last=event["timeS"]
                    if event["action"] in chain:
                        a,b,actor=chain[event["action"]]
                        piece=event["piece"]
                        self.assertEqual(actor,event["actor"])
                        self.assertEqual(state[piece],a)
                        if b in ("B3","B2"):
                            self.assertNotIn(b,state.values())
                        state[piece]=b
                        if b=="pad":
                            count.append(piece)
                self.assertEqual(run["deposits"],count)
                self.assertEqual(run["points"],{"B3":0,"B2":0})
                if "/" not in name:
                    self.assertEqual(state,run["custody"])
                    self.assertEqual(len(count),len(prior["buffer/"+name]["deposits"]))
                    self.assertGreater(run["minimumCarryMarginN"],0)
        self.assertEqual(runs["slow-receiver"]["custody"]["center-1/1"],"pad")
        self.assertEqual(runs["diagnostic/directDispatch"]["custody"]["center-1/1"],"B2")
        self.assertEqual(runs["control/sweptAway"]["deposits"],["center-1/1"])
        for mode in ("externalDock","offset3mm","raisedWithdrawal","braked","empty","noGrip",
                     "unreserved","rectangularReservation"):
            self.assertEqual(runs["control/"+mode]["deposits"],[],mode)

    def test_prior_module_results_are_byte_identical(self):
        records = {"buffer-results.json":"c9928d509e5537af8b0da50518d8eb2ff83d295426f0671ad4d09fbf7e0df40c", "serial-results.json":"7a05db504d63b7516ba87790d340a6f5387962c87fb3e645efbe61ad80bff84d"}
        for filename,expected in records.items():
            self.assertEqual(hashlib.sha256((ROOT/"experiments/ground-relay"/filename).read_bytes()).hexdigest(),expected)


if __name__=="__main__":
    unittest.main()

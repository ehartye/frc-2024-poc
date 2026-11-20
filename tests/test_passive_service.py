"""Keep the offline service receipt bound to the released, reopened fixture."""

import hashlib
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
READINESS = ROOT / "cad/service-readiness"


class ServiceFixture(unittest.TestCase):
    def test_reopened_fixture(self):
        receipt = json.loads((READINESS / "reopen.json").read_bytes())
        for key, name in [
            ("parentSha256", "r9-physical-assembly.f3d"),
            ("archiveSha256", "r10-passive-service.f3d"),
        ]:
            self.assertEqual(
                receipt[key], hashlib.sha256((ROOT / "cad/student" / name).read_bytes()).hexdigest()
            )
        self.assertGreater(receipt["bodyCount"], 50)
        self.assertLess(receipt["carrierGuideSweepMm3"], 0.01)
        self.assertGreater(receipt["offsetCarrierGuideControlMm3"], 1)
        self.assertFalse(receipt["maximumCaseRemovalFixture"])
        self.assertFalse(receipt["maximumCaseRemovalR9"])
        self.assertFalse(any(receipt["pinRetractedExtraction"]))
        self.assertFalse(any(receipt["strapToolCollisions"].values()))
        self.assertFalse(any(receipt["carrierGuideAndShaftChecks"].values()))
        self.assertEqual(len(receipt["locatorContactControlsMm3"]), 4)
        self.assertGreater(
            min(
                [
                    receipt["retainedStrapControlMm3"],
                    receipt["pinEngagedWithdrawalControlMm3"],
                    *receipt["undrilledStationControlsMm3"],
                    *receipt["locatorContactControlsMm3"],
                ]
            ),
            0.01,
        )
        self.assertGreater(min(receipt["nominalProjectedSeatMm2"].values()), 1)

    def test_supplier_bracket_rejects_candidate(self):
        review = json.loads((READINESS / "controller-mount-review.json").read_bytes())
        self.assertEqual(review["controllerSolidCount"], 18)
        self.assertLess(review["nominalBracketOverlapMm3"], 0.01)
        self.assertGreater(min(review["controlOverlapMm3"]), 1)
        rejected = [
            i
            for i, s in enumerate(review["stations"], 1)
            if min(s["materialToDeckOutsideEdgeMm"]) < 0
        ]
        self.assertEqual(rejected, [1, 2, 5])
        (pair,) = review["overlappingBracketBounds"]
        self.assertEqual(pair["stations"], [1, 4])
        self.assertGreater(pair["bracketOverlapMm3"], 209)


if __name__ == "__main__":
    unittest.main()

import hashlib
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim/pressure"))
from compare_shot_replays import compare
from mission_runtime import verify_runtime_chain


class ReplayComparison(unittest.TestCase):
    def test_campaign_requires_the_locked_stack_through_every_stage(self):
        stack = dict(python="3.13.12", mujoco="4.0.1", numpy="2.5.5")
        stage = dict(runtime=stack)
        self.assertEqual(verify_runtime_chain([stage, stage]), stack)
        alternate = dict(runtime=dict(stack, mujoco="3.14.0"))
        for chain in ([alternate, alternate], [stage, alternate], [stage, {}], [{}]):
            with self.assertRaises(ValueError):
                verify_runtime_chain(chain)

    def release(self, folder, dt, centers, **overrides):
        folder.mkdir()
        rows = [dict(time=n * dt, q=[x, 0, 0, 1, 0, 0, 0], wheelNormalN=2 * n)
                for n, x in enumerate(centers, 1)]
        trace = b"".join(json.dumps(row).encode() + b"\n" for row in rows)
        (folder / "trace.jsonl").write_bytes(trace)
        receipt = dict(schema="wing-raised-feed/1", traceStride=1, start=0., end=len(rows) * dt,
                       dt=dt, firstLauncherContact=.01, initialQ=[0], initialV=[0],
                       pieceId="middle", sourceTraceSha256="delivery", nativeSha256="geometry",
                       traceSha256=hashlib.sha256(trace).hexdigest())
        receipt.update(overrides)
        (folder / "result.json").write_text(json.dumps(receipt))
        return folder

    def test_nested_steps_compare_only_shared_times(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            left = self.release(root / "coarse", .01, [.01, .02])
            right = self.release(root / "fine", .005, [.005, .01, .015, .021, .025])
            result = compare(left, right)
            self.assertEqual(result["commonSamples"], 2)
            self.assertEqual(result["maxCenterDeltaThroughFirstTouchMm"], 0.)
            self.assertAlmostEqual(result["maxCenterDeltaOverCommonTimeMm"], 1.)
            self.assertEqual(result["firstCenterDeltaAtLeastPointOneMmS"], .02)
            self.assertEqual(result["peakWheelLoadN"], [4, 10])

    def test_different_delivery_and_modified_trace_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            left = self.release(root / "left", .01, [.01])
            right = self.release(root / "right", .01, [.01], sourceTraceSha256="another delivery")
            with self.assertRaisesRegex(ValueError, "starting evidence"):
                compare(left, right)
            (right / "trace.jsonl").write_bytes(b"")
            with self.assertRaisesRegex(ValueError, "differs from its receipt"):
                compare(left, right)

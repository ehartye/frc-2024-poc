"""Compare two fine release traces from the same attained delivery state."""
import argparse
import hashlib
import json
import math
from pathlib import Path


def read_release(folder):
    receipt = json.loads((folder / "result.json").read_bytes())
    path = folder / "trace.jsonl"
    with path.open("rb") as stream:
        if hashlib.file_digest(stream, "sha256").hexdigest() != receipt["traceSha256"]:
            raise ValueError("Release trace differs from its receipt")
        stream.seek(0)
        rows = [json.loads(line) for line in stream]
    if receipt["schema"] != "wing-raised-feed/1" or receipt["traceStride"] != 1 or not rows:
        raise ValueError("Expected a release trace with every integration step")
    for index, row in enumerate(rows, 1):
        expected = receipt["start"] + index * receipt["dt"]
        if abs(row["time"] - expected) > 1e-8:
            raise ValueError("Release trace has a missing or misaligned step")
    if abs(rows[-1]["time"] - receipt["end"]) > 1e-8:
        raise ValueError("Release trace ends before its recorded endpoint")
    return receipt, rows


def compare(left, right):
    a, ar = read_release(left)
    b, br = read_release(right)
    for key in ("sourceTraceSha256", "nativeSha256", "pieceId", "start", "initialQ", "initialV"):
        if a[key] != b[key]:
            raise ValueError(f"Different starting evidence for {key}")
    for key, default in (("duty", None), ("pinchDuty", .3), ("targetRpm", None),
                         ("pitchTrimDeg", 0.), ("heightTrimMm", 0.), ("alignmentDurationS", 0.)):
        if a.get(key, default) != b.get(key, default):
            raise ValueError(f"Different release controller setting for {key}")
    period = max(a["dt"], b["dt"])
    steps = [round(period / r["dt"]) for r in (a, b)]
    if any(abs(s * r["dt"] - period) > 1e-12 for s, r in zip(steps, (a, b))):
        raise ValueError("Use nested integration timesteps")
    common = min(len(ar) // steps[0], len(br) // steps[1])
    if not common:
        raise ValueError("No shared post-integration samples")
    first_touch = min((r["firstLauncherContact"] for r in (a, b)
                       if r["firstLauncherContact"] is not None), default=math.inf)
    precontact_mm = peak_mm = 0.
    first_tenth_mm = None
    for n in range(1, common + 1):
        x, y = ar[n * steps[0] - 1], br[n * steps[1] - 1]
        gap = 1000 * math.dist(x["q"][-7:-4], y["q"][-7:-4])
        peak_mm = max(peak_mm, gap)
        if x["time"] <= first_touch + 1e-9:
            precontact_mm = max(precontact_mm, gap)
        if first_tenth_mm is None and gap >= .1:
            first_tenth_mm = x["time"]
    return dict(sourceDeliveryTrace=a["sourceTraceSha256"],
                traces=[r["traceSha256"] for r in (a, b)],
                commonSamples=common, commonPeriodS=period,
                maxCenterDeltaThroughFirstTouchMm=precontact_mm,
                firstCenterDeltaAtLeastPointOneMmS=first_tenth_mm,
                maxCenterDeltaOverCommonTimeMm=peak_mm,
                peakWheelLoadN=[max(row["wheelNormalN"] for row in rows) for rows in (ar, br)])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference", type=Path)
    parser.add_argument("replay", type=Path)
    args = parser.parse_args()
    print(json.dumps(compare(args.reference, args.replay), indent=2, allow_nan=False))

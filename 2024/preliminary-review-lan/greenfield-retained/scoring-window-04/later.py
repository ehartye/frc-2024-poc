"""Risk-sized follow-up stages of the single-shot campaign. campaign.py loads this for a stage it does not define.

high-NNN: the high edge of the launch angle at the revised lane (y = 5.821 m). Six carries that span the
perturbed population are fired one angle at a time; the search stops at the first angle where any shot fails.
The six are the smallest-margin carry at 124 degrees (R05-middle), the two slowest releases (R02-inner,
R03-inner), the highest and lowest release elevations (R05-outer, R07-center-high) and the largest-margin
carry (R01-center-mid).

released-20s: the perturbed NOTES delivered but not counted within 7 s at the revised operating point,
observed for 20 s.

angle-123-window: the nominal carries at 123 degrees at the revised lane, and the six spanning carries at the
edge values of heading, station, lane and launcher speed.

lane-5796-rest: the carries at y = 5.796 m that the interrupted revised-more stage did not finish.

angle-123-20s: the same observation for the NOTES not counted at 123 degrees at the revised lane.
"""
import json
from pathlib import Path

import campaign

HERE = Path(__file__).resolve().parent
SPAN = ("R05-middle", "R02-inner", "R03-inner", "R05-outer", "R07-center-high", "R01-center-mid")
LANE = dict(y=5.821)


def stages():
    s = {}
    for angle in range(125, 136):
        s[f"high-{angle}"] = [dict(campaign.NOMINAL, **LANE, tag=tag, n=96, angle=float(angle)) for tag in SPAN]
    late = []
    for name in ("lateral.jsonl", "revised-nominal.jsonl"):
        for line in (HERE / "results" / name).read_text().splitlines():
            r = json.loads(line)
            at_point = all(r.get(k, v) == dict(campaign.NOMINAL, **LANE)[k] for k, v in campaign.NOMINAL.items())
            if at_point and "error" not in r and r["opening"] and not r["passShot"]:
                late.append(dict(campaign.NOMINAL, **LANE, tag=r["tag"], n=r["n"], duration=20.0))
    s["released-20s"] = late
    s["angle-123-20s"] = [
        dict(campaign.NOMINAL, **LANE, tag=r["tag"], n=r["n"], angle=123.0, duration=20.0)
        for r in map(json.loads, (HERE / "results" / "revised-angle.jsonl").read_text().splitlines())
        if r["angle"] == 123.0 and "error" not in r and r["opening"] and not r["passShot"]]
    # The window about the recommended operating point: 123 degrees at the revised lane. The twelve nominal
    # carries, then the six spanning carries at the edge values found at 122 degrees.
    point = dict(campaign.NOMINAL, **LANE, angle=123.0)
    at = [dict(point, tag=tag, n=n) for tag, n in campaign.carries("dev")]
    for axis, values in (("yaw", [-4.0, -3.0, 3.0, 4.0]), ("x", [1.45, 1.49]), ("y", [5.771, 5.796, 5.871, 5.921]),
                         ("speed", [6.56, 7.20])):
        at += [dict(point, tag=tag, n=96, **{axis: v}) for v in values for tag in SPAN]
    s["angle-123-window"] = at
    done = {json.loads(line)["id"] for line in (HERE / "results" / "revised-more.jsonl").read_text().splitlines()}
    rest = [dict(campaign.NOMINAL, tag=tag, n=n, y=5.796) for tag, n in campaign.carries("frozen")]
    s["lane-5796-rest"] = [c for c in rest if campaign.case_id(c) not in done]
    return s

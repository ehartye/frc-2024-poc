"""Single-shot scoring campaign: every carried NOTE of the frozen starts, fired over a matrix of launcher
speed, cassette angle, station, heading and field counter drive.

Usage: python campaign.py STAGE [--workers N] [--keep]

Each case runs counter_tail.py in its own working folder beside this one (ignored by Git), then one summary
row is appended to results/STAGE.jsonl. A finished case is skipped on a rerun. --keep also stores the full
shot record under shots/. The solver is deterministic: the same case gives the same row.

The incoming states are the carries of counted-reset-power-02: the nominal route at two resolutions (dev,
12 carries) and the perturbed frozen starts R01 to R07 (34 carries). They are the population the 19-of-20
target was first tried on. Every shot is observed for 7 s after its start.
"""
import hashlib
import itertools
import json
import math
import os
import shutil
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
SOURCE = ROOT / "counted-reset-power-02"
NOMINAL = dict(speed=6.88, angle=122.0, x=1.39, y=5.871, yaw=0.0, lower=0.75, upper=0.75, fieldMu=0.45, duration=7.0)


def carries(kind="all"):
    tags = []
    for f in sorted(SOURCE.glob("*-96.json")):
        name = f.name[:-8]
        if name.startswith("R0") and kind in ("all", "frozen"):
            tags.append((name, 96))
        if name.startswith("dev-") and kind in ("all", "dev"):
            tags.append((name, 96))
    if kind in ("all", "dev"):
        tags += [(f.name[:-8], 48) for f in sorted(SOURCE.glob("dev-*-48.json"))]
    return tags


def grid(tags, **axes):
    """Cases over the product of the stated axes, every other input nominal."""
    keys = list(axes)
    out = []
    for (tag, n), values in itertools.product(tags, itertools.product(*axes.values())):
        out.append(dict(NOMINAL, tag=tag, n=n, **dict(zip(keys, values))))
    return out


def one_at_a_time(tags, base, **axes):
    """Cases that change one input at a time from a base operating point."""
    out = []
    for key, values in axes.items():
        for value in values:
            for tag, n in tags:
                out.append(dict(NOMINAL, **base, tag=tag, n=n, **{key: value}))
    return out


def frange(a, b, step):
    k = int(round((b - a) / step))
    return [round(a + i * step, 4) for i in range(k + 1)]


def by(cases, key):
    """Order cases so each value of one input finishes across the whole population before the next starts."""
    return sorted(cases, key=lambda c: (c[key], c["tag"], c["n"]))


def stages():
    every, frozen = carries("all"), carries("frozen")
    s = {}
    # 1. Launcher speed alone, at the released angle and station, on the frozen starts. The released 6.88 m/s
    #    rows are the records of counted-reset-power-02 (read by analyse.py), which this solver reproduces exactly.
    s["speed"] = by(grid(frozen, speed=[6.56, 6.72, 6.96, 7.04, 7.20, 7.36, 7.52, 7.68]), "speed")
    # 2. One input at a time from the released operating point. The robot fires with its rear bumper 35 mm from the
    #    subwoofer face, so the station can only err away from the speaker.
    s["angle"] = by(grid(frozen, angle=[119.0, 120.0, 121.0, 123.0, 124.0, 125.0]), "angle")
    s["angle-high"] = by(grid(frozen, angle=[126.0, 127.0, 128.0, 129.0, 130.0]), "angle")
    s["heading"] = by(grid(frozen, yaw=[-3.0, -2.0, -1.0, 1.0, 2.0, 3.0]), "yaw")
    s["lateral"] = by(grid(frozen, y=[5.721, 5.771, 5.821, 5.921, 5.971, 6.021]), "y")
    s["station"] = by(grid(frozen, x=[1.42, 1.45, 1.49]), "x")
    # 3. The field's own counter drive, which the robot does not control: fraction of no-load speed, lower and upper.
    s["field"] = [dict(c, upper=c["lower"]) for c in by(grid(frozen, lower=[0.55, 0.90, 1.00]), "lower")]
    #    And the friction of every speaker surface against the NOTE (0.45 as released).
    s["field-friction"] = by(grid(frozen, fieldMu=[0.30, 0.60]), "fieldMu")
    #    Low launcher speeds on the nominal carries, to find where delivery through the opening stops.
    s["low-speed"] = by(grid(carries("dev")[:6], speed=[3.44, 5.20, 5.60, 5.92, 6.08, 6.24]), "speed")
    #    Rear bumper against the subwoofer face: the face is at x = 0.918 m and the bumper 0.435 m behind the robot
    #    centre, so the robot centre is at 1.353 m. The closer station wants a slightly steeper cassette.
    s["contact-station"] = by(grid(frozen, x=[1.353], angle=[120.5, 121.0, 121.5, 122.0]), "angle")
    # 3. The revised firing lane, 50 mm nearer the opening centre (y = 5.821 m), one input at a time.
    lane = dict(y=5.821)
    s["revised-heading"] = by(one_at_a_time(frozen, lane, yaw=[-4.0, -3.0, -2.0, -1.0, 1.0, 2.0, 3.0, 4.0]), "yaw")
    s["revised-angle"] = by(one_at_a_time(frozen, lane, angle=[120.0, 121.0, 123.0, 124.0]), "angle")
    s["revised-lateral"] = by(grid(frozen, y=[5.846, 5.896]), "y")
    s["revised-station"] = by(one_at_a_time(frozen, lane, x=[1.42, 1.45, 1.49]), "x")
    s["revised-speed"] = by(one_at_a_time(frozen, lane, speed=[6.56, 7.20]), "speed")
    s["revised-more"] = (by(grid(frozen, y=[5.796]), "y")
                         + by(one_at_a_time(frozen, lane, angle=[125.0, 126.0]), "angle"))
    s["revised-nominal"] = grid(carries("dev"), y=[5.821])
    #    Counting rose as the heading turned the shot toward the +y side of the opening. Sample that neighbourhood.
    s["aim-search"] = (by(grid(frozen, y=[5.801, 5.841], yaw=[-3.0, -2.0, -1.0]), "yaw")
                       + by(grid(frozen, y=[5.821], yaw=[-2.0], speed=[6.72, 7.04]), "speed"))
    # 4. Every delivered NOTE that was not counted within 7 s, observed for 20 s.
    late = []
    for f in sorted((HERE / "results").glob("*.jsonl")):
        if f.stem == "extended":
            continue
        for line in f.read_text().splitlines():
            r = json.loads(line)
            if "error" not in r and r["opening"] and not r["passShot"]:
                late.append(dict({k: r.get(k, v) for k, v in NOMINAL.items()}, tag=r["tag"], n=r["n"], duration=20.0))
    for f in sorted(SOURCE.glob("split-tail_.25_0-R0*-96-6.88.json")):
        r = json.loads(f.read_bytes())
        if r["opening"] and not r["passShot"]:
            late.append(dict(NOMINAL, tag=f.name[len("split-tail_.25_0-"):-len("-96-6.88.json")], n=96, duration=20.0))
    s["extended"] = late
    return s


def case_id(c):
    return (f"{c['tag']}-{c['n']}_v{c['speed']:.2f}_a{c['angle']:.1f}_x{c['x']:.3f}_y{c['y']:.3f}"
            f"_h{c['yaw']:+.1f}_c{c['lower']:.2f}-{c['upper']:.2f}" + ("" if c["fieldMu"] == 0.45 else f"_m{c['fieldMu']:.2f}")
            + ("" if c["duration"] == 7.0 else f"_t{c['duration']:g}"))


def run(c, keep):
    cid = case_id(c)
    work = ROOT / f"_w-{cid}"
    work.mkdir(exist_ok=True)
    for name in ("shot.py", "lane.py", "counter_tail.py"):
        shutil.copyfile(HERE / name, work / name)
    carry = SOURCE / f"{c['tag']}-{c['n']}.json"
    shutil.copyfile(carry, work / carry.name)
    env = dict(os.environ, EVIDENCE_ROOT=str(ROOT), W_SPEED=str(c["speed"]), W_ANGLE=str(c["angle"]), W_X=str(c["x"]),
               W_Y=str(c["y"]), W_YAW=str(c["yaw"]), W_LOWER=str(c["lower"]), W_UPPER=str(c["upper"]), W_FIELD_MU=str(c["fieldMu"]), W_DURATION=str(c["duration"]))
    done = subprocess.run([sys.executable, "-B", "counter_tail.py", c["tag"], str(c["n"]), ".25", "0"], cwd=work, env=env,
                          capture_output=True, text=True)
    outs = list(work.glob("split-tail_*.json"))
    row = dict(id=cid, **c, carrySha256=hashlib.sha256(carry.read_bytes()).hexdigest())
    if done.returncode or len(outs) != 1:
        row.update(error=(done.stderr or done.stdout)[-600:])
    else:
        raw = outs[0].read_bytes()
        r = json.loads(raw)
        rel, last = r["release"], r["trace"][-1]
        v = rel["v"][:3] if rel else None
        row.update(
            passShot=r["passShot"], feedTime=r["feedTime"], enableTime=r["enableTime"], opening=r["opening"],
            beams=r["beams"], both=r["both"], minBusV=r["minBusV"], peakZ=r["peakZ"], contacts=r["contacts"],
            releaseT=rel["t"] if rel else None, releaseV=v,
            releaseSpeed=math.sqrt(sum(a * a for a in v)) if v else None,
            releaseElevationDeg=math.degrees(math.atan2(v[2], math.hypot(v[0], v[1]))) if v else None,
            endT=last["t"], endQ=last["q"][:3], angleUsed=r["angle"], pivot=r["pivot"],
            shotSha256=hashlib.sha256(raw).hexdigest())
        if keep:
            (HERE / "shots").mkdir(exist_ok=True)
            shutil.copyfile(outs[0], HERE / "shots" / f"{cid}.json")
    shutil.rmtree(work, ignore_errors=True)
    return row


def main():
    stage = sys.argv[1]
    workers = int(sys.argv[sys.argv.index("--workers") + 1]) if "--workers" in sys.argv else 12
    keep = "--keep" in sys.argv
    table = stages()
    if stage not in table:
        import later
        table.update(later.stages())
    cases = table[stage]
    out = HERE / "results" / f"{stage}.jsonl"
    out.parent.mkdir(exist_ok=True)
    seen = {json.loads(line)["id"] for line in out.read_text().splitlines()} if out.is_file() else set()
    todo = [c for c in cases if case_id(c) not in seen]
    print(stage, len(cases), "cases,", len(todo), "to run", flush=True)
    with ThreadPoolExecutor(max_workers=workers) as pool, out.open("a", newline="\n") as f:
        for k, row in enumerate(pool.map(lambda c: run(c, keep), todo), 1):
            f.write(json.dumps(row, separators=(",", ":")) + "\n")
            f.flush()
            if k % 25 == 0 or k == len(todo):
                print(k, "/", len(todo), flush=True)


if __name__ == "__main__":
    main()

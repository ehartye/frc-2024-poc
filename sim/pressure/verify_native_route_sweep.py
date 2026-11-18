"""Verify that native results cover every input interval, including failed parents."""
import hashlib
import json
from pathlib import Path


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_level(poses, receipts, source_digest):
    expected = json.loads(poses.read_bytes())
    completed = [json.loads(line) for line in receipts.read_text().splitlines()]
    if len(expected) != len(completed):
        raise ValueError("Incomplete native response")
    packet_digest = digest(poses)
    for a,b in zip(expected,completed):
        if (b.get("archiveSha256") != source_digest or b.get("posePacketSha256") != packet_digest
                or any(b.get(k) != v for k,v in a.items()) or b["last"] <= b["first"]):
            raise ValueError("Native result differs from its expected pose or revision")
    return completed


def partition(rows, first, last):
    cursor = first
    for row in rows:
        if row["first"] != cursor or not cursor < row["last"] <= last:
            raise ValueError("Native interval gap, overlap or repeated subdivision")
        cursor = row["last"]
    if cursor != last:
        raise ValueError("Missing native interval")


def certify(levels, last, source_digest):
    pending=[(0,last)]
    totals=[]
    for poses,receipts in levels:
        rows=read_level(poses,receipts,source_digest)
        i=0
        next_pending=[]
        for lo,hi in pending:
            begin=i
            while i<len(rows) and rows[i]["first"]<hi:
                i+=1
            group=rows[begin:i]
            partition(group,lo,hi)
            next_pending.extend((r["first"],r["last"]) for r in group if r["hits"])
        if i != len(rows):
            raise ValueError("Unexpected native results outside the pending intervals")
        totals.append(dict(intervals=len(rows),needsRefinement=len(next_pending),
            booleanChecks=sum(r["checks"] for r in rows),posesSha256=digest(poses),receiptsSha256=digest(receipts)))
        pending=next_pending
    if pending:
        raise ValueError("Unresolved native overlap or geometry-kernel failure")
    return dict(archiveSha256=source_digest,coveredSteps=last,levels=totals)

"""Sample new attained lift/pitch extrema with the same native structural test used for the raised route."""
import hashlib
import json
import pathlib
import re
import subprocess
import sys

import numpy as np

base = pathlib.Path(__file__).parent
case = sys.argv[1]
shot = base / case
packet = json.loads((shot/"result.json").read_bytes())
trace = shot/"trace.jsonl"
with trace.open("rb") as stream:
    assert hashlib.file_digest(stream,"sha256").hexdigest() == packet["traceSha256"]
with trace.open("rb") as stream:
    poses = [json.loads(line) for line in stream]
q = np.asarray([r["q"][3:5] for r in poses])
times = np.asarray([r["time"] for r in poses])
indices = set([0,len(q)-1])
for axis in q.T:
    indices.update([int(np.argmax(axis)),int(np.argmin(axis))])
for when in ("feedStart","firstLauncherContact","lastLauncherContact"):
    if packet.get(when) is not None:
        indices.add(int(np.argmin(abs(times-packet[when]))))
values = [(float(times[i]),float(q[i,1]),25.4+1000*float(q[i,0])) for i in sorted(indices)]
bound = (base/"fusion-native-outer-trim-plus15-24-0.py").read_text()
borrow = bound[bound.index("import adsk.core,adsk.fusion,hashlib,json"):bound.index("def run(context):")]
borrow = borrow.replace("def snapshot(doc):","def registered_document(doc):")
run = bound[bound.index("def run(context):"):]
checks = run[run.index("    app="):run.index("    check(")]
checks = checks.replace("state=snapshot(doc)","state=registered_document(doc)")
old = (base.parent/"turn-lift-20261118/fusion-turn-structure-fine.py").read_text()
old = re.sub(r"    app=C.Application.get\(\); original=.*?\n",lambda _:checks+"    docs=[doc]\n",old,count=1)
old = re.sub(r"        poses=\[.*?\]\n",lambda _:"        poses="+repr(values)+"\n",old,count=1)
result = base/("native-"+case+"-structure.jsonl")
old = old.replace("(P/'turn-structure-fine-left.jsonl')","pathlib.Path("+repr(str(result))+")")
out = base/("fusion-structure-"+case+".py")
out.write_text("from pathlib import Path\n"+borrow+"\n"+old)
probe = base.parent/"native-loading-gate-20261118/fusion_probe.py"
p = subprocess.run([sys.executable,"-B",str(probe),str(out)],capture_output=True)
if p.returncode:
    print(p.stderr.decode(errors="replace")[-900:])
    sys.exit(p.returncode)
response = json.loads(p.stdout)
if response.get("result",{}).get("isError") or not result.exists():
    raise RuntimeError(str(response)[-1500:])
samples = [json.loads(line) for line in result.read_bytes().splitlines()]
assert len(samples)==len(values)+1
summary = dict(native=packet["nativeSha256"],trace=packet["traceSha256"],samples=len(values),changes=sum(len(r["changes"]) for r in samples[1:]),maxPitchDeg=float(np.rad2deg(q[:,1].max())),maxPivotMm=25.4+1000*float(q[:,0].max()))
(base/("native-"+case+"-structure-summary.json")).write_text(json.dumps(summary,indent=2))
print(summary)

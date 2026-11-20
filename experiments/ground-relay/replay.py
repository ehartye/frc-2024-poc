"""Replay both frozen cohorts, the weakest matched condition, and explicit no-go controls."""
import json
import subprocess
import sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]

def run(plugin):
    subprocess.run(["node",HERE/"field.mjs",plugin],check=True)
    from relay import read, save, ref, simulate, FIELD, PEERS, PADS, SOURCES, B3, PORT, STAGES, RADIUS
    lock=read(HERE/"freeze-lock.json")
    demand=read(HERE/"demand.json")
    outcomes={}
    scores={}
    for name in ("serial","buffer"):
        cohort_path=HERE/(name+"-freeze.json")
        freeze=read(cohort_path)
        source=ref(cohort_path)
        if source["sha256"]!=lock[name]:raise ValueError("Frozen cohort changed")
        receipts={};entries=[]
        for c in freeze["cases"]:
            receipt,outcome=simulate(c,freeze["revision"],lock[name],demand)
            receipts[c["id"]]=receipt
            outcomes[name+"/"+c["id"]]=outcome
        receipt_path=HERE/(name+"-executions.json")
        save(receipt_path,receipts)
        for case in freeze["cases"]:
            source_ref=ref(receipt_path)|{"pointer":"/"+case["id"]}
            entries.append({"id":case["id"],"source":source_ref})
        study_path=HERE/(name+"-study.json")
        save(study_path,{"schema":"agent-frc/throughput-study@2","cohort":source,"episodes":entries})
        result=subprocess.check_output(["node",plugin/"hardening.mjs","throughput",study_path,ROOT,
                                        "--freeze-sha256",lock[name]],text=True)
        scores[name]=json.loads(result)
        save(HERE/(name+"-results.json"),scores[name])
        assert scores[name]["qualifiedRuns"]==len(freeze["cases"])
    # The characterization is frozen separately. Compare both policies on the same
    # withheld settings without inserting supplemental replays in its official denominator.
    for c in read(HERE/"buffer-freeze.json")["cases"]:
        if c["partition"]=="held-out":
            _,outcomes["serial/"+c["id"]]=simulate(c,"serial-confirm",lock["serial"],demand)
    controls={}
    clear=read(HERE/"buffer-freeze.json")["cases"][0]
    for label,change in (list(lock["controls"].items())+
                         [("sweptAway",{"sweptAway":True}),("unreserved",{"unannouncedTraffic":True}),("oneInch",{"clearanceIn":1.})]):
        _,outcome=simulate(clear,"two-pad-buffer",lock["buffer"],demand,change)
        outcomes["control/"+label]=outcome
        controls[label]={k:v for k,v in outcome.items() if k not in ("history","samples")}
    weakest = read(HERE/"buffer-freeze.json")["cases"][-1]
    for name,change in [("preposition",{"preposition":True}),
                        ("exposed-store",{"preposition":True,"localStore":"east"}),
                        ("direct-dispatch",{"preposition":True,"localStore":"west"})]:
        _,outcomes["diagnostic/"+name]=simulate(weakest,"two-pad-buffer",lock["buffer"],demand,change)
    save(HERE/"controls.json",controls)
    summary={k:{a:b for a,b in v.items() if a not in ("history","samples")} for k,v in outcomes.items()}
    save(HERE/"custody-summary.json",summary)
    evidence=["cad/student/r14-removable-frame-tie.f3d","cad/service-readiness/stiffness/native-fit.json",
              "sim/pressure/evidence/maxswerve-development-carry.json",
              "sim/pressure/evidence/retained-pickup-20261118/short-lift-verified.json",
              "kickoff/crescendo-field.json","experiments/ground-relay/footprint.json",
              "experiments/ground-relay/peer-tracks.json","experiments/ground-relay/relay.py"]
    save(HERE/"sources.json",{f:ref(ROOT/f)["sha256"] for f in evidence})
    view={"field":[FIELD["field"]["length"],FIELD["field"]["width"]],"protectedStages":STAGES,
          "box":B3,"port":PORT,"ringRadius":RADIUS,"dt":PEERS["dt"],"peers":PEERS["traces"],"pads":PADS,
          "sources":SOURCES,"initial":PEERS["initial"],"cases":outcomes,"freeze":lock}
    save(ROOT/"docs/ground-relay.json",view)

if __name__=="__main__":
    run(Path(sys.argv[1]).resolve())

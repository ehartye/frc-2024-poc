"""Export proposed chassis motion on the original six-robot clock. Custody credit is deferred."""
import json
import math
import sys
from pathlib import Path

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/"registered-custody"))
import rehearse as match

def write(path,value):
    path.write_bytes((json.dumps(value,separators=(",",":"))+"\n").encode())

def profile(case,control,measured,setback=4):
    design=match.relay.read(HERE/"part.json")
    class Carrier(match.relay.Bot):
        def propose(self):
            while self.path and match.relay.near(self.p,self.path[0]):
                self.path.pop(0)
                self.origin=None
            if not self.path:
                return self.p
            if self.origin is None and self.cargo:
                self.origin=self.p
                length=math.dist(self.p,self.path[0])
                duration=max(1.5*length/42,math.sqrt(6*length/design["loadedAccelerationInS2"]))
                self.ticks=math.ceil(duration/match.DT)
                self.progress=0
            return super().propose()

    options={"carrierClass":Carrier,"liftMm":design["liftMm"],"seatMm":design["seatXmm"],"exitSetbackIn":setback,"accelerationLimitMps2":.55,"cargoAllowanceMm":15,"frames":[],"settledOffsetsMm":measured}
    run=match.run(case,control,planning=options)
    plans=[]
    for index,item in enumerate(match.relay.STOCK):
        events={e["action"]:e["timeS"] for e in run["history"] if e["piece"]==item and e["actor"]=="B3"}
        if "clear floor deposit" not in events:
            continue
        lifted=events["seated lift complete"]
        up=events["bilateral floor seat"]
        floor=events["floor support at pad"]
        deposit=events["clear floor deposit"]
        source=list(match.relay.SOURCES[item.split("/")[0]])
        if index:source[1]+=case["parameters"]["offset"]
        rows=[r for r in options["frames"] if r[5]==item and r[6]=="entry"]
        start=rows[0][0]
        frames=[]
        def smooth(u):return u*u*(3-2*u)
        for t,x,y,*_ in options["frames"]:
            if t<start or t>deposit:continue
            phase,z="entry",0
            if t>=up:
                phase,z="lift",design["liftMm"]*smooth(min(1,(t-up)/.8))
            if t>=lifted:phase,z="carry",design["liftMm"]
            if floor-.8<=t<floor:phase,z="lower",design["liftMm"]*(1-smooth((t-floor+.8)/.8))
            if t>=floor:phase,z="withdraw",0
            frames.append([t,round(x*25.4,5),round(y*25.4,5),round(z,5),phase])
        plans.append({"id":item,"sourceMm":[v*25.4 for v in source],"padMm":[v*25.4 for v in match.PADS[index]],
                      "depositS":deposit,"frames":frames})
    return {"plans":plans,"schedule":run}

def generate(folder,measured):
    cases=match.relay.read(match.PARENT/"buffer-freeze.json")["cases"]
    out={}
    for name,case,control in [(c["id"],c,"revised") for c in cases]+[("directDispatch",cases[-1],"directDispatch")]:
        out[name]=profile(case,control,measured.get(name,{}))
    write(folder/"clocks.json",out)
    rejected=profile(cases[0],"revised",{},setback=0)["schedule"]
    write(HERE/"lane-control.json",{"stop":rejected["stop"],"history":rejected["history"][-2:],
                                   "peakLoadedAccelerationMps2":rejected["peakLoadedAccelerationMps2"]})

if __name__=="__main__":
    directory=Path(sys.argv[1])
    predictions=match.relay.read(directory/"settled.json") if len(sys.argv)>2 else {}
    generate(directory,predictions)

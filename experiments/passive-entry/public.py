"""Publish the student replay, with a simple load-path screen from the final geometry."""
import json
import math
import sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
def read(path):return json.loads(path.read_bytes())
def save(path,x):path.write_bytes((json.dumps(x,separators=(",",":"))+"\n").encode())

def build(folder):
    records=read(HERE/"field.json")
    load=max(p["peakRailN"] for run in records.values() for p in run["pieces"].values())
    cases={}
    for name,force in (("observed",load),("bench20N",20.)):
        # One rail takes the entire applied force. Simply supported wire and
        # cantilever carrier idealizations omit joint flexibility and impact.
        wireI=math.pi*4**4/64
        beamI=6.35*11**3/12
        cases[name]={"loadN":round(force,3),"spineBendMPa":round(force*150*2/(4*wireI),2),
                     "spineDeflectionMm":round(force*150**3/(48*200000*wireI),3),
                     "carrierBendMPa":round(force*194*5.5/beamI,2),
                     "carrierDeflectionMm":round(force*194**3/(3*68300*beamI),3),
                     "outerScrewTensionN":round(force*(500-293)/(2*13),2)}
    save(HERE/"load-path.json",cases)
    frames=read(folder/"section-traces.json")
    save(ROOT/"docs/passive-entry.json",{"design":read(HERE/"part.json"),"runs":records,
         "bench":{k:frames[k] for k in ("-6","-3","0","3","6")},"service":read(HERE/"service.json")})

if __name__=="__main__":build(Path(sys.argv[1]))

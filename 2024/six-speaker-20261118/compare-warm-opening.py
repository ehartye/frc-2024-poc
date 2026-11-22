import json, sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path.cwd()/'sim/pressure'))
from wing_speaker_passage import screen
from wing_field_frame import WingFrame
base=Path(sys.argv[1])
for name in sys.argv[2:]:
    folder=base/name
    rc=json.loads((folder/'result.json').read_bytes())
    with (folder/'trace.jsonl').open('rb') as f:
        f.seek(-16384,2)
        row=json.loads(f.readlines()[-1])
    frame=WingFrame(rc['firstMark'],rc['fieldYawDeg'])
    results={}
    for dx in (0,-.02,-.04,-.06,-.08,-.10,.02):
        shifted=dict(row)
        shifted['q']=row['q'].copy()
        shifted['q'][-7:-4]=(np.array(row['q'][-7:-4])+frame.rotation.T@np.array([dx,0,0])).tolist()
        obs=screen(shifted,512,.0002,frame=frame)
        results[dx]=round(1000*obs['minimumCertifiedMarginM'],2)
    print(name,results,flush=True)

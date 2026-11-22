"""Private same-trace prefeed sensitivity pilot. Verify the unchanged-duty recovery before use."""
import json, sys
from pathlib import Path
root=Path.cwd()/'sim/pressure'
sys.path.insert(0,str(root))
from run_axial_shear_lift import digest
import mujoco,numpy as np
source=root/'wing_note_release.py'
baseline=Path(__file__).parent/'high-forward-seated-trim'
base=json.loads((baseline/'result.json').read_bytes())
assert digest(baseline/'trace.jsonl')==base['traceSha256']
def prefeed_prefix(args,p,stream):
    expected=(base['sourceTraceSha256'],base['pitchTrimDeg'],base['heightTrimMm'],base['targetRpm'],base['alignmentDurationS'],base['dt'],base['grid'],base['segments'],base['duty'])
    actual=(json.loads((args.delivery/'result.json').read_bytes())['trace'],args.pitch_trim_deg,args.height_trim_mm,args.target_rpm,args.align_seconds,args.dt,args.grid,args.segments,args.duty)
    assert actual==expected
    minimum=p.bus
    with (baseline/'trace.jsonl').open() as logged:
        for line in logged:
            row=json.loads(line)
            stream.write(line)
            minimum=min(minimum,row['busV'])
            if row['time']==base['feedStart']: break
        else:raise ValueError('Controller boundary missing')
    assert row['beltNormalN']>0 and row['wheelNormalN']==0
    p.data.qpos[:]=row['q'];p.data.qvel[:]=row['v'];p.data.time=row['time']
    p.rotors=np.array(row['externalRotors']);p.shear=np.array(row['shearM'])
    p.bus=row['busV'];p.source_work=row['sourceJ']
    p.ready_ticks=3;p.pose_ticks=3
    mujoco.mj_forward(p.model,p.data)
    return minimum
s=source.read_text()
needle='        for i in range(round(duration/args.dt)):'
assert s.count(needle)==1
s=s.replace(needle,'        minimum_bus=prefeed_prefix(args,p,stream)\n'+needle)
exec(compile(s,str(source),'exec'),globals())

from pathlib import Path
p=Path(__file__).resolve().parent
parent=p.parent/'split-field-02'
src=(parent/'backing_check.py').read_text(encoding='utf8')
s=src.split('rows=[]')[0].replace('np.arange(160)*math.tau/160','np.arange(2048)*math.tau/2048').replace('-18.;dz','-17.25;dz')
s=s.replace('def checkPose(positions,quat,angle,origin):', '''def checkPose(positions,quat,angle,origin):
    if len(angle)>48:
        return np.concatenate([checkPose(positions[i:i+48],quat[i:i+48],angle[i:i+48],origin[i:i+48]) for i in range(0,len(angle),48)])''')
env=dict(__file__=str(parent/'backing_check.py'));exec(s,env)
import json,math
rows=[]
attempts=sorted(parent.glob('attempt-*.json'))
for file in attempts:
    a=json.loads(file.read_bytes())
    for piece in a['parts']:
        tag=f"{a['attempt']}-{piece['id']}-{a['n']}"
        rows.append(env['leg'](parent/(tag+'.json')))
        if 'shot' in piece:rows.append(env['fire'](parent/('split-ready-'+tag+'-6.88.json')))
worst=min(rows,key=lambda x:x['minimumSampleMm']); bound=math.pi*152.4/2048
out=dict(covered=[f.name for f in attempts],rows=rows,worst=worst,angularAllowanceMm=bound,minimumAtRecordedPosesMm=worst['minimumSampleMm']-bound,model='Analytic rigid backing ending at 17.25 mm, 2048 nodes on the circular ring centerline. One-half angular interval deducted using the Lipschitz distance bound. Covers saved poses only. STEP displaced controls in result.json.')
(p/'backing-review.json').write_text(json.dumps(out,indent=2));print(len(rows),worst,out['minimumAtRecordedPosesMm'])


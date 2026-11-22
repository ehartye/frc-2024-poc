"""Compare middle-of-field dynamics before and after a matched change to the field transfer surface."""
from pathlib import Path
import json,hashlib
p=Path(__file__).resolve().parent;old=p.parent/'split-field-02';read=lambda f:json.loads(f.read_bytes());rows=[]
for k,v,cmd in [('R01-center-low-96',6.88,.6),('R01-center-low-96',6.88,-.6),('dev-center-high-96',6.74,.6),('dev-center-high-96',6.88,.6)]:
    orig=f'split-ready-{k}-{v}.json';base=(old if v==6.88 else p)/orig;test=p/f'split-readywheel{cmd}-{k}-{v}.json'
    a,b=read(base),read(test)
    assert all(a[z]==b[z] for z in ['initialState','initialVelocity','sourceCarry','rotorsAtFeed','release','exit','opening'])
    rows.append(dict(case=k,launcherMps=v,fieldWheelTrialMps=cmd,baselineBeamS=a['both'],trialBeamS=b['both'],sameReleaseAndOpening=True))
out=dict(drawing='FIRST GE-24370 and GE-24385, PDF pages 206 and 222',pdfSha256=hashlib.sha256((p/'field-drawing-source.pdf').read_bytes()).hexdigest(),source='https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024FieldDrawings-CRESCENDOSpecific.pdf',measuredGripShaftXZmm=[-404.48,1509.61],equalPulleyXZmm=[[-767.66,1473.25],[-404.48,1509.61]],wheelSpeedBasis='±0.60 m/s sensitivity. Field operating voltage, speed, slip and command are unverified.',tests=rows,decision='The stationary wheel approximation can change post-opening credit. Keep the robot command unchanged while completing and validating the field transport model. The reversed test supplies sensitivity only. No retrospective score awarded.')
(p/'counter-review.json').write_text(json.dumps(out,indent=2));print(rows)

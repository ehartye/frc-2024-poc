"""Read completed bounded trials and record the actual next decision; never credit a partly finished case."""
from pathlib import Path
import json,hashlib
p=Path(__file__).resolve().parent
read=lambda f:json.loads(f.read_bytes())
states=[read(p/f'six-{s}-{n}.json') for s,n in [('dev',48),('dev',96),('R01',96)]]
sensor=[read(p/(s+'.json')) for s in ['R08-inner-96','R08-inner-48','blind-R08-inner-96']]
shot=read(p/'split-ready-R08-inner-96-6.74.json');assert shot['sourceCarry']==hashlib.sha256((p/'R08-inner-96.json').read_bytes()).hexdigest()
window=read(p/'results.json')
lines=['# Launch speed and pickup confirmation','','The same missed center-line NOTE was replayed from its saved pose, velocity and three rotor speeds. The 6.88 m/s baseline cleared the opening by 106.3 mm but missed at least one official detector pair. Both 6.74 and 7.02 m/s counted that shot and both nominal controls. The lower command gave 115.7 mm on the failed start and shorter nominal detector times. It was then applied to every piece.','','| Full six-piece replay | Scored | Last clock | Mission |','|---|---:|---:|---|']
for r in states:
    lines.append(f"| {r['start']} · {r['n']} segments | {r['score']}/6 | {r['rows'][-1]['arrival']:.2f} s | {'Pass' if r['passMission'] else 'Miss at '+r['rows'][-1]['id']} |")
lines+=['','At the original simulated sensor location the left sensor housing conflicts with the catch sector. At x60 mm, z50.4 mm a 450 mm transverse beam and two 6 × 12 × 8 mm housing envelopes clear the occupied cassette. The new beam was replayed through the actual moving NOTE with 40 ms continuous detection and a 1.5 second timeout.']
for label,r in zip(['R08 at 96','R08 at 48','Blind control at 96'],sensor):
    lines.append(f"- {label}: {'captured and held' if r['held'] else 'inhibited'} after {r['seatDelay']*1000:.1f} ms extra dwell.")
lines.append(f"The recovered fine R08 NOTE {'counted both detector pairs' if shot['passShot'] else 'missed the field scoring criterion'} at the selected 6.74 m/s command. This individual shot and the commissioning replays do not revise the original six successful starts, two failed starts and twelve unrun.")
if all(r['passMission'] for r in states):
    lines+=['','Advance 6.74 m/s to the whole-route paired test with the selected pickup sensor active on every piece. Resolve coarse capture and incorporate physical sensor mounts, tolerances and the revised robot mass before claiming mission repeatability.']
else:
    lines+=['','Hold the global speed change. Separate time lost to detector dwell from missed capture and actual field discharge before another full campaign. Keep the 19-of-20 gate.']
(p/'DECISION.md').write_text('\n'.join(lines)+'\n',encoding='utf8')
print('\n'.join(lines[-5:]))

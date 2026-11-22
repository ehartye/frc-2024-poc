"""Keep the tested cases and their bounds together, including adverse controls."""
from pathlib import Path
import json,math
from paired_cycle import read,sha,save
p=Path(__file__).resolve().parent
capture=[]
for f in sorted(p.glob('capture-*.json')):
    d=read(f);_,n,pc,dt,*_=f.stem.split('-');pads=int(pc);step=float(dt)
    r=.011;h=.001;spacing=(2*(.625-.091)+2*math.pi*r)/pads
    over=(math.hypot(r+h,.48*spacing)-(r+h))*1000
    s=p/f'split-paired_.75_.75-{f.stem}-6.88.json'
    capture.append(dict(case=f.stem,n=int(n),pads=pads,dt=step,padTipExcessMm=over,controlledCarry=sha(f),seat=d['seated'],firstDrivenSideS=d['sideBeltTakeoverS'],sensorAtS=d['beamProofS'],heldThroughNominalWindow=d['held'],shot=sha(s) if s.exists() else None,scored=read(s)['passShot'] if s.exists() else None,counterS=read(s)['both'] if s.exists() else None))
save(p/'pickup-refinement.json',capture)
m=[read(p/f'cycle-{s}-{n}.json') for s,n in [('dev',48),('dev',96),('R01',96)]]
text="""# Six-NOTE scoring review

Profile A remains the default removable lip. The front of the robot and the 6.88 m/s scoring command are unchanged. Two separate effects explain the original misses.

## Field counter

The season drawing assigns nine grip wheels, an upper conveyor and a separate lower conveyor to the speaker. The original solver left the upper conveyor and the grip wheels stationary. It also drove the lower conveyor at 1.50 m/s. The lower 12:1 drive and measured belt geometry imply about 1.13 m/s at the catalog 12 V no-load speed. The upper conveyor and grip wheels turn on opposite sides of a gear mesh. Absolute polarity and loaded field RPM still need on-field confirmation.

On the same R01 final launch, the original model missed. With both drives at 55%, 75% and 90% of nominal no-load speed, the two modeled counters registered at 5.949, 4.747 and 5.319 seconds. At 75% with reversed upper drive neither registered. All runs preserve exactly the incoming state, wheel speeds at feed, release velocity and full opening crossing. The full NOTE cleared the opening by 106 mm. The paired calculation captures a real change in the field model; its operating speeds are assumptions. These times are from start of spin-up.

| Starting setup | Scored | Elapsed | Lowest shot bus |
|---|---:|---:|---:|
"""
for r in m:
    text+=f"| {r['case']} · {r['n']} NOTE segments | {r['score']}/6 | {r['legs'][-1]['clock']:.2f} s | {min(x['minBusV'] for x in r['legs']):.3f} V |\n"
text+="""
The mission clock retains the conservative wait for both field beams before each departure. The table reuses the three incoming sequences to isolate the field change. The previous six-of-eight results remain a separate campaign.

## Floor acquisition

The coarse R08 simulation stopped on the carpet beyond the blade. Waiting longer for a seated-piece signal left the NOTE on the floor. The flat rectangles used to represent the nose belt protrude at the 11 mm pulley radius. Halving each rectangle length while keeping the same stroke, pitch, total moving mass, surface speed and width tests whether that representation changes acquisition. Independent timestep and ring-segmentation changes also affect the coarse result.

| NOTE segments | Belt elements | Step (ms) | Pad corner excess (mm) | Side drive takes over (s) | Seat beam confirms (s) | Shot counted |
|---:|---:|---:|---:|---:|---:|---|
"""
for q in capture:
    counted='pending' if q['shot'] is None and q['pads']>38 else 'untested' if q['shot'] is None else 'yes' if q['scored'] else 'no'
    text+=f"| {q['n']} | {q['pads']} | {1000*q['dt']:.2f} | {q['padTipExcessMm']:.2f} | {q['firstDrivenSideS'] or 0:.3f} | {q['sensorAtS'] or 0:.3f} | {counted} |\n"
text+="""
The 76- and 152-element belts both transferred the same free R08 NOTE to the side drives and confirmed its seat at 2.82 s on both clocks. These are controlled model refinements. Actual compression, friction, carpet and production lip height still need the physical pickup fixture. A matched campaign with the 76-element belt, 40 ms seat interlock and coupled field drive is recorded leg by leg in the mission receipts. It resets the prespecified count, keeps the 120 s project limit and the 8 V minimum shot bus, and stops once a nomination cannot pass. The earlier campaign and timings remain accessible.

## Review

- [Watch the driven field counter](http://192.168.1.129:8764/?concept=guided&case=counter-driven)
- [Watch the reversed upper drive](http://192.168.1.129:8764/?concept=guided&case=counter-reversed)

The two clips display the measured free NOTE and official field fixture. Field drive motion is represented in contact forces. The displayed field mesh remains stationary. A frozen whole-robot decision also needs the loaded cassette/drive budgets and the battery-service slide restraint.

Sources: FIRST season-specific GE-24351, GE-24370 and GE-24385 drawings, the issued field mesh, the V0 manual §6.5.1, and the AndyMark am-0255 12 V motor curve. The field measurements and gear checks are reproducible in counter.py and transmission.json.
"""
(p/'REVIEW.md').write_text(text,encoding='utf8')


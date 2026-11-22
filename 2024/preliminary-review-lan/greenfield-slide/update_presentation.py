from pathlib import Path
p=Path('present.py');s=p.read_text(encoding='utf8')
s=s.replace("'Pivot support':(2.4,4.2)","'Pivot support':(3.8,6.1)").replace("'Pivot':['Pivot motor']","'Pivot and translating support':['Pivot motor','Slide drive motor']").replace('(5,15)','(5,18)').replace('(6,24)','(9,30)').replace('(8,28)','(12,34)')
s=s.replace("if case==cases[4]","if case in cases[4:]").replace("['Pack lead','Distribution lead'] if case","['Pack lead','Distribution lead',held] if case")
a=s.index("notes=[");b=s.index("data=dict",a)
s=s[:a]+'''eng=json.loads((HERE/'engineering.json').read_bytes())
notes=[f'Greenfield P2.2 · 92 planning articles. A pair of outboard frames supports the cassette pivot on a translating carriage. The carriage retracts for the middle of the swing. Belts reverse for intake and launch. No game score is established.',
'P1 fixed-pivot control: the entry bracket alone exceeded forward extension by 177.2 mm. On P2 the local bounds of the actual moving cassette fit the rectangle used by the slide schedule. Its continuously bounded front reaches at most robot X634 against X644.8 permitted, leaving 10.8 mm nominal. A stalled slide reproduces the overreach. Stops, sensors and tracking tolerance remain open.',
'P2 first fit: each side beam entered the bumper by 106.72 mm³ and the nose reached 2.90 mm below carpet. Raising the pivot 14 mm cleared both at the 25 checked operating poses. The lower roller makes first contact after 16.13 mm of ideal straight approach. The modeled tip-up can lose powered nose contact by 52.95 mm. Capturing a free NOTE under load is open.',
'Battery withdrawn straight up 370 mm, cables disconnected. At the 125° launch pose it intersects the belt by 8644.65 mm³ and one runner by 365.97 mm³. At 80° the entire swept case clears by 5 mm excluding its support deck. An installed mechanical service prop, handle and wiring are still needed.',
'Shown field outline uses the official speaker corners; the NOTE has a 355.6 mm outer diameter and a 50.8 mm section. The simulation here prescribes the NOTE path. Power lines are battery-side scenarios, and the masses are subsystem planning allowances including separate integration and design reserves.',
f'The outboard A-frames retain {identity_control["minLateralGapMm"]:.1f} mm lateral clearance from the centered NOTE throughout motion. The deliberately displaced NOTE cuts the left A-frame by {identity_control["controlMm3"]:.1f} mm³. Other displayed operating results are 25-pose screens.']
''' +s[b:]
s=s.replace('Greenfield P1.2 · Swing-through belt cassette','Greenfield P2.2 · Sliding-pivot cassette').replace('Held · extension limit','Held · floor capture').replace("focus='Horizontal sweep exceeds front extension by at least '+str(round(tip-limit))+' mm. Select Deploy to floor to inspect the swing.'","focus='Front extension reserve 10.8 mm. Floor capture is unproved. Compare the two battery withdrawal poses.'")
s=s.replace('view=HERE/\'pivot-cassette.step\'',"view=HERE/'sliding-pivot.step'")
p.write_text(s,encoding='utf8')


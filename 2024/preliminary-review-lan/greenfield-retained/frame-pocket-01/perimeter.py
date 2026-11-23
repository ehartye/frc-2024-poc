"""Second material-removal study. Keep bumper-facing surfaces continuous around the entire perimeter."""
from pathlib import Path
p=Path(__file__).resolve().parent
script=(p/'frame.py').read_text()
needle='hits=[];tested=0'
change="""oldRing=design[frame]
ring=oldRing
for side,along,coord,openings in [
    ('right',True,-330,[(-215,-165),(-129,-52),(51,128),(161,216)]),
    ('left',True,330,[(-215,-165),(-129,-52),(51,128),(161,216)]),
    ('front',False,340,[(-208,-153),(-121,-47),(47,121),(153,208)]),
    ('rear',False,-340,[(-208,-153),(-121,-47),(47,121),(153,208)])]:
    # 14 mm-deep inside pocket in an 18 mm-wide rail; 4 mm remains on the bumper side.
    start=coord+4 if coord<0 else coord-18.5
    tool=(0,start,14.5,True) if along else (start,0,14.5,False)
    for low,high in openings:
        ring=cut(ring,pocket(tool,low,high));cuts.append([frame+' '+side,low,high])
assert BRepCheck_Analyzer(ring).IsValid()
beforeRingKg=V(oldRing)*2.7e-6
design[frame]=ring;extraSaving=(V(oldRing)-V(ring))*2.7e-6
"""
assert script.count(needle)==1
script=script.replace(needle,change+needle)
script=script.replace("oldFrameSolidKg=(sum(before.values())+V(design[frame]))*2.70e-6","oldFrameSolidKg=sum(before.values())*2.70e-6+beforeRingKg,perimeterSavingKg=extraSaving")
script=script.replace("for n,s in subjects.items()},separators","for n,s in {**subjects,frame:ring}.items()},separators")
exec(compile(script,'inside-face perimeter pockets','exec'))

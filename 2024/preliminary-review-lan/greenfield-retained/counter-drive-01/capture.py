"""Disentangle NOTE tessellation, timestep and belt-pad discretization on the frozen missed pickup."""
from pathlib import Path
import sys
p=Path(__file__).resolve().parent
n,pads,dt=sys.argv[1:4]
tag=f'capture-{n}-{pads}-{dt}'
sys.argv=[__file__,'inner','R08',n,'working']
text=(p.parent/'seat-interlock-01/recovery.py').read_text()
def replace(a,b):
    global text
    assert text.count(a)==1,a
    text=text.replace(a,b)
replace('(local[:,0]-.055)','(local[:,0]-.060)')
replace('abs(local[:,1])-.050','abs(local[:,1])-.225')
replace('return interlock(original_split(source))','''s=interlock(original_split(source))
    assert s.count('count=38;')==1 and s.count('    points=[];')==1 and s.count('mass=".07" priority')==1
    s=s.replace('count=38;',f'count={pads};').replace('    points=[];',f'    dt={dt}\\n    points=[];').replace('mass=".07" priority',f'mass="{.07*38/int(pads)}" priority')
    return s''')
replace("('blind-' if blind else '')+attempt", "tag+'-'+attempt")
exec(compile(text,'tested beam and matched pickup refinements','exec'))


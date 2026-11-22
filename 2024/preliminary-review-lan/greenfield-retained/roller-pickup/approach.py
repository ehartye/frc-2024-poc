from pathlib import Path
import sys
p=Path(__file__).with_name('run.py')
s=p.read_text().replace("'0.36',offset","'0.44',offset")
k="exec(src);r=run'''"
assert k in s
s=s.replace(k,"""src=src.replace('.7866309 {offset}', '.8666309 {offset}')
src=src.replace('round(2.5/dt)','round(2.82/dt)')
# Keep the ring/carpet, flap, supports and opposed traction belt parameters fixed.
# Only change the contacts on the moving intake belt and the exposed nose drums.
old='mass=".07" contype="2"'
assert old in src
src=src.replace(old, 'mass=".07" priority="1" friction="'+str(grip)+' .003 .001" contype="2"')
old='mass=".16" contype="2"'
assert old in src
src=src.replace(old, 'mass=".16" priority="1" friction="'+str(grip)+' .003 .001" contype="2"')
exec(src);r=run'''""").replace('else 0,float(grip))','else 0,.75)').replace('driveMode=mode','driveMode=mode,noseContactFriction=float(mu),initialShiftM=.08')
exec(compile(s,str(p),'exec'))


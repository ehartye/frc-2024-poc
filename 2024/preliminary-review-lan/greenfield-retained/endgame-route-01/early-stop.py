"""Compare two lower load-bearing angles with the same sensors, clearances, proof duration and true free chain."""
from pathlib import Path
import sys
p=Path(__file__).resolve().parent
s=(p/'neighbor-dynamics.py').read_text(encoding='utf8')
stop=float(sys.argv.pop(5));delta=158-stop
needle="    guard('(t-.55)/2.4',"
assert s.count(needle)==1
s=s.replace(needle,"""    for a in [165,160,159.5,158,157.4]:
        key='np.deg2rad('+str(a)+')'
        assert key in src,(a,src.count(key))
        src=src.replace(key,'np.deg2rad('+str(a-delta)+')')
    guard('158 if armed','%s if armed'%stop)
    src=src.replace('cut=8.0','cut=9.0').replace('int((8.0+5.)/dt)','int((9.0+5.)/dt)')
    assert 'cut=9.0' in src
"""+needle)
exec(compile(s,'lower angular seat prototype','exec'))


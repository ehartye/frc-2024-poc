"""Run the fixed optical-seat controller for a named piece and start on the original roster."""
from pathlib import Path
import sys
p=Path(__file__).resolve().parent
attempt,leg,n,mode=sys.argv[1:5]
dt='.00025' if n=='96' else '.0005'
sys.argv=[__file__,n,'76',dt]
s=(p/'capture.py').read_text()
a="sys.argv=[__file__,'inner','R08',n,'working']"
assert s.count(a)==1
s=s.replace(a,"tag='mission-'+mode;sys.argv=[__file__,leg,attempt,n,mode]")
exec(compile(s,'fixed capture for the named floor piece','exec'))


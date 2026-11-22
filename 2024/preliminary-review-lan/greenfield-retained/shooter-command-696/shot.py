"""Use the inherited free NOTE and actual independent rotor states in the steeper full-gravity firing pose."""
from pathlib import Path
import json,sys
p=Path(__file__).resolve().parent;root=p.parent
sys.path.insert(0,str(root/'counter-drive-01'))
from counter import bind
sys.path[:0]=['C:/Users/ehart/Documents/FRC Native/2024/six-speaker-20261118/field-query-libs','C:/Users/ehart/repos/frc-2024-poc/sim/pressure']
import run_speaker_field as model
tag,n=sys.argv[1:3]
c=json.loads((p/f'{tag}-{n}.json').read_bytes())
assert c['endPitchDegrees']==122 and c['held'] and c['route']['loaded'][-1]==[1.39,5.871]
bind(model,.75,.75)
sys.argv=[__file__,tag,n,'revised','6.96']
src=(root/'split-cycle-01/split_fire.py').read_text()
marker="exec(compile(script,'fire"
assert src.count(marker)==1
src=src.replace(marker,"assert script.count(\"'125','1.53'\")==1\nscript=script.replace(\"'125','1.53'\",\"'122','1.39'\")\n"+marker)
exec(compile(src,'revised firing point','exec'))


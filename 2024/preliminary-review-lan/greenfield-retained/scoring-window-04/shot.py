"""Fire one carried NOTE at a stated launcher speed, cassette angle and station. Copy of
counted-reset-power-02/shot.py with the speed, angle, station and field drive fractions read from the
environment. The carried NOTE is posed with the robot, so a station offset moves NOTE and launcher together.
The carry ended at 122 degrees; another angle re-poses only the launcher model and is a perturbation of the
aim, not a re-run of the lift."""
from pathlib import Path
import json,os,sys
p=Path(__file__).resolve().parent;root=Path(os.environ['EVIDENCE_ROOT']).resolve()
repo=root.parents[2]
sys.path.insert(0,str(root/'counter-drive-01'))
from counter import bind
sys.path[:0]=[str(repo/'2024/six-speaker-20261118/field-query-libs'),str(repo/'sim/pressure')]
import run_speaker_field as model
tag,n=sys.argv[1:3]
SPEED=os.environ.get('W_SPEED','6.88');ANGLE=os.environ.get('W_ANGLE','122');X=os.environ.get('W_X','1.39');Y=os.environ.get('W_Y','5.871')
c=json.loads((p/f'{tag}-{n}.json').read_bytes())
assert c['endPitchDegrees']==122 and c['held'] and c['route']['loaded'][-1]==[1.39,5.871]
bind(model,float(os.environ.get('W_LOWER','.75')),float(os.environ.get('W_UPPER','.75')))
sys.argv=[__file__,tag,n,'revised',SPEED]
src=(root/'split-cycle-01/split_fire.py').read_text()
marker="exec(compile(script,'fire"
assert src.count(marker)==1
src=src.replace(marker,"assert script.count(\"'125','1.53'\")==1\nscript=script.replace(\"'125','1.53'\",\"'122','1.39'\")\n"+marker)
exec(compile(src,'revised firing point','exec'))

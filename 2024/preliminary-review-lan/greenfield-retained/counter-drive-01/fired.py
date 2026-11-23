"""Re-run the full powered robot and speaker with the measured paired shafts installed."""
from pathlib import Path
import sys,shutil,json,hashlib
from counter import bind
p=Path(__file__).resolve().parent
sys.path[:0]=['R:/2024/six-speaker-20261118/field-query-libs','R:/sim/pressure']
import run_speaker_field as model
tag,n,setting,lo,hi=sys.argv[1:6]
src=None
for folder in [p,p.parent/'split-field-02',p.parent/'launch-window-01']:
    f=folder/f'{tag}-{n}.json'
    if f.exists():src=f;break
assert src
if src.parent!=p:shutil.copyfile(src,p/src.name)
t,v=bind(model,float(lo),float(hi))
sys.argv=[__file__,tag,n,f'paired_{lo}_{hi}',setting]
exec(compile((p.parent/'split-field-02/fire.py').read_text(),'original powered shot and revised counter','exec'))

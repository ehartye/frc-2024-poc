"""Compare a bounded-acceleration command on the same lateral route and unrestrained game piece."""
from pathlib import Path
import sys,json,hashlib
p=Path(__file__).resolve().parent;root=p.parent
sys.path.insert(0,str(root/'follower-01'))
from smooth import table,times
source=root/'lateral-station-01'
assert (p/'plan.json').read_bytes()==(source/'plan.json').read_bytes()
raw=(source/'carry.py').read_text()
edit="""marker='        tj=jid(\'truck\');'
    assert s.count(marker)==1 and s.count('    for step in range')==1
    s=s.replace('    for step in range',"    posTable,velTable=table(deploy,122,g['pickPivot'][0])\\n    for step in range")
    s=s.replace(marker,"        profileTime=t-startLift-delay;X=float(np.interp(profileTime,times,posTable));delta=(X-robotX)*1000;assert abs(robotX+delta/1000-X)<1e-12;vx=float(np.interp(profileTime,times,velTable))\\n"+marker)
    """
# Use double quotes around the literal that contains a quoted MuJoCo joint name.
edit=edit.replace("marker='        tj=jid('truck');'",'marker="        tj=jid(\'truck\');"')
marker='    return s.replace(\'125-deploy\''
assert raw.count(marker)==1
raw=raw.replace(marker,'    '+edit.replace('\\n','\\\\n')+marker.lstrip())
exec(compile(raw,'finite acceleration actual free-body carry','exec'))


"""Pilot bounding-box projections using the registered moving body axes."""
from pathlib import Path
import sys
root=Path.cwd()/'sim/pressure'
sys.path.insert(0,str(root))
source=(root/'field_route_clearance.py').read_text()
needle='                gap=min(gap,float(clear.min()))'
add='''                flagged=np.flatnonzero(clear<=0)
                if len(flagged):
                    field_center=ob[index].mean(axis=0)
                    field_half=(ob[index,1]-ob[index,0])/2
                    local_gap=(np.abs((field_center-placed[flagged])@rot)
                               -field_half@np.abs(rot)-half[flagged]-pad)
                    clear[flagged]=np.maximum(clear[flagged],local_gap.max(axis=1))
'''
assert source.count(needle)==1
exec(compile(source.replace(needle,add+needle),str(root/'field_route_clearance.py'),'exec'),{'__name__':'__main__','__file__':str(root/'field_route_clearance.py')})

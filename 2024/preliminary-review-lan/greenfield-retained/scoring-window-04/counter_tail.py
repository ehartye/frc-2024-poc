"""Extend a saved scored shot through the whole stationary reset, adding collection-floor contact."""
from pathlib import Path
import sys
p=Path(__file__).resolve().parent
tag,n,mu,height=sys.argv[1:5]
assert mu in ('.25','.75') and height in ('0','.05')
suffix='tail_'+mu+'_'+height
raw=(p/'lane.py').read_text().replace("sys.argv[3]='outerlane'","sys.argv[3]=suffix")
mark="exec(compile(base,'constant lateral offset trial','exec'))";assert raw.count(mark)==1
# Insert into the source-bound injection immediately before the solver builds its XML.
patch="sub('<worldbody><body pos=',"+repr('<worldbody><geom name="collectionFloor" type="plane" pos="0 0 '+height+'" size="0 0 .1" friction="'+mu+' .005 .0001" contype="2" conaffinity="1"/><body pos=')+")\nsub('if center[2]<.17 or (not withfield and center[2]<1 and release and d.qvel[2]<0):break','if not withfield and center[2]<1 and release and d.qvel[2]<0:break')\n"
patch+="sub('command=-.40 if not feeding else 1.35','command=(0. if feedT is not None and t>=feedT+1. else (-.40 if not feeding else 1.35))')"+chr(10)
patch+="sub('row=dict(t=d.time,q=q.tolist(),beltMps=belts.tolist(),busV=bus)','row=dict(t=d.time,q=q.tolist(),feedMps=feedSpeed,beltMps=belts.tolist(),busV=bus)')"+chr(10)
needle="build=script.split("
ix=raw.index(needle)
raw=raw[:ix]+"assert script.count('env.update(params=coefficients()')==1\nscript=script.replace('env.update(params=coefficients()',patch+'env.update(params=coefficients()')\n"+raw[ix:]
exec(compile(raw,'extend scored shot with a floor','exec'))


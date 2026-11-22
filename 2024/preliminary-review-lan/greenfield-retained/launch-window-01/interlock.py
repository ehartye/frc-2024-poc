"""Replay the R08 free NOTE with a mountable full-width beam and the same 40 ms proof time."""
from pathlib import Path
import sys
p=Path(__file__).resolve().parent;src=p.parent/'seat-interlock-01/recovery.py'
text=src.read_text()
assert text.count('(local[:,0]-.055)')==1 and text.count('abs(local[:,1])-.050')==1
text=text.replace('(local[:,0]-.055)','(local[:,0]-.060)').replace('abs(local[:,1])-.050','abs(local[:,1])-.225')
exec(compile(text,'450 mm physical optical path at x60 z50.4','exec'))

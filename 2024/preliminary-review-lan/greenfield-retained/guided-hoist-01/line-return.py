"""Compare an open circuit at disable with a preloaded take-up, using the same free chassis and chain."""
from pathlib import Path
import sys
p=Path(__file__).resolve().parent
code=(p/'winch.py').read_text(encoding='utf8')
old="exec(compile(src,'guided cable hoist on existing pivot motor','exec'))"
assert code.count(old)==1
insert="""sub('em=velocity*reduction/rDrum*ke','''if latched:
        target=stroke-.003+.0005+.00020;tv=0.
    em=velocity*reduction/rDrum*ke''')
sub('force=fDrive-cable-75*velocity','''# A modeled 0.30 Nm drum spring maintains line tension after the pin carries the chassis.
    bias=10.0 if at is not None else 0.
    force=fDrive+bias-cable-75*velocity''')
sub('coil=coil,bus=bus','coil=coil,latchedNow=latched,bus=bus')
exec(compile(src,'bounded passive take-up sensitivity','exec'))"""
exec(compile(code.replace(old,insert),'winch with positive cable return','exec'))

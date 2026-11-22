"""Compare the station readiness dwell while retaining the same pose and load gates."""
from pathlib import Path
import hashlib
import sys

source = Path.cwd() / "sim/pressure/wing_note_delivery.py"
assert hashlib.sha256(source.read_bytes()).hexdigest() == "c2ed9549facc26496778f6b0a0f4466971ced58d592f2cd92777de4cbb316c81"
sys.path.insert(0, str(source.parent))
text = source.read_text()
anchor = 'if d.time-hold >= .15:outcome="AT_SHOOTING_POSE"'
assert text.count(anchor) == 1
text = text.replace(anchor, anchor.replace(".15", ".02"))
anchor = '    (args.output/"result.json").write_text('
assert text.count(anchor) == 1
text = text.replace(anchor, '    result["stationReadyDwellS"] = .02\n'+anchor)
exec(compile(text, str(source), "exec"), globals())

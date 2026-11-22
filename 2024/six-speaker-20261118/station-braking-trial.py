"""Private closed-loop approach comparison on the recorded native robot."""
from pathlib import Path
import hashlib
import sys

source = Path.cwd() / "sim/pressure/wing_note_delivery.py"
assert hashlib.sha256(source.read_bytes()).hexdigest() == "c2ed9549facc26496778f6b0a0f4466971ced58d592f2cd92777de4cbb316c81"
sys.path.insert(0, str(source.parent))
text = source.read_text()
anchor = "            want_speed=min(want_speed,.62+.95*max(0.,station_distance-.65))"
extra = """
            if station_distance <= 2.:
                braking_speed = math.sqrt(2 * 1.3 * station_distance)
                target_speed = min(self.transit_speed, 1.90, braking_speed, 3.4 * speed_distance)
                want_speed = target_speed * max(0., math.cos(error))**6"""
assert text.count(anchor) == 1
text = text.replace(anchor, anchor + extra)
anchor = '    (args.output/"result.json").write_text('
assert text.count(anchor) == 1
text = text.replace(anchor, '    result["stationBrakingTrial"] = {"distanceM": 2., "accelerationMps2": 1.3, "maxSpeedMps": 1.90, "positionGainPerS": 3.4}\n'+anchor)
exec(compile(text, str(source), "exec"), globals())

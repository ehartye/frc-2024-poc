"""Replay the original receiver clock, then plan its paths to measured piece positions."""
import json
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent

def run(program, *args):
    command = ["node"] if program.endswith(".mjs") else [sys.executable, "-B"]
    subprocess.run([*command, str(HERE / program), *map(str, args)], check=True)

if __name__ == "__main__":
    with tempfile.TemporaryDirectory() as scratch:
        run("clocks.py", scratch)
        run("field.mjs", scratch)
        offsets = Path(scratch) / "settled.json"
        first = json.loads(offsets.read_bytes())
        run("clocks.py", scratch, "measured")
        run("field.mjs", scratch)
        second = json.loads(offsets.read_bytes())
        for case, pieces in first.items():
            for piece, xy in pieces.items():
                assert max(abs(a-b) for a,b in zip(xy,second[case][piece])) < .05
        run("bench.mjs")
        run("sensitivity.mjs", scratch)
        run("service.py")
        run("public.py", scratch)
        run("showcase-audit.mjs")

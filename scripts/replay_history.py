"""Materialize a copy of the recorded pressure-entry workspace for local historical replay."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
CAMPAIGN = Path("experiments/a2-campaign")
FROZEN = Path("experiments/pressure-entry/frozen")
# The input register names the published digest of each restored version.
OVERRIDES = {
    "sim/pressure/fast_geometry.py": FROZEN / "sim/pressure/fast_geometry.py",
    "tests/test_pressure_fast_geometry.py": FROZEN / "tests/test_pressure_fast_geometry.py",
    "tools/blockout/SOURCE.json": CAMPAIGN / "frozen/blockout-source.json",
    "tools/blockout/index.html": CAMPAIGN / "frozen/blockout-index.html",
}


def materialize(destination):
    destination = destination.resolve()
    if destination.exists():
        raise ValueError("Choose a new output directory")
    if destination.is_relative_to(ROOT) and not destination.is_relative_to(ROOT / "artifacts"):
        raise ValueError("Place a local replay under artifacts/ or outside the checkout")
    destination.parent.mkdir(parents=True, exist_ok=True)
    tracked = subprocess.check_output(["git", "-C", str(ROOT), "ls-files", "-z"]).decode().rstrip("\0").split("\0")
    with tempfile.TemporaryDirectory(prefix="history-", dir=destination.parent) as tmp:
        tree = Path(tmp) / "project"
        def copy(source, relative):
            output = tree / relative
            output.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, output)
        for name in tracked:
            copy(ROOT / name, name)
        for original, stored in (("tools/blockout", Path("kickoff/blockout")), ("tools/showcase", Path("scripts/showcase")),
                                 (".", CAMPAIGN)):
            for name in map(Path, tracked):
                if not name.is_relative_to(stored):
                    continue
                suffix = name.relative_to(stored)
                if stored == CAMPAIGN and suffix.parts[0] in {"cad", "frozen", "CAMPAIGN.md"}:
                    continue
                copy(ROOT / name, Path(original) / suffix)
        for name, source in OVERRIDES.items():
            copy(ROOT / source, name)
        pins = json.loads((ROOT / "experiments/pressure-entry/input-register.json").read_bytes())
        for name, expected in pins["sha256"].items():
            if hashlib.sha256((tree / name).read_bytes()).hexdigest() != expected:
                raise ValueError("Historical input differs from the registered hash: " + name)
        tree.rename(destination)
    return destination


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("destination", type=Path)
    print(materialize(parser.parse_args().destination))


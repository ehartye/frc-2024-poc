"""Restore the original engineering checkout from the issued Git bundle parts."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("destination", type=Path, help="New directory for the engineering checkout")
    destination = parser.parse_args().destination.resolve()
    if destination.exists():
        parser.error("Choose a new destination directory.")
    source = Path(__file__).resolve().parent
    manifest = json.loads((source / "bundle.json").read_text())
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=destination.parent) as scratch:
        bundle = Path(scratch) / "engineering-iterations.bundle"
        with bundle.open("wb") as output:
            for item in manifest["parts"]:
                part = source / item["file"]
                if part.stat().st_size != item["bytes"]:
                    parser.error(f"Run git lfs pull to download {part.name}.")
                with part.open("rb") as chunk:
                    shutil.copyfileobj(chunk, output, length=8 * 1024 * 1024)
        subprocess.run(["git", "clone", "--branch", manifest["branch"], str(bundle), str(destination)], check=True)
    # The immutable source is this branch's bundle. The temporary bundle cannot serve as a remote.
    subprocess.run(["git", "-C", str(destination), "remote", "remove", "origin"], check=True)
    print(f"Restored {destination} at {manifest['commit']}")


if __name__ == "__main__":
    main()

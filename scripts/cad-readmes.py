"""Refresh or verify the preliminary design still at the top of every project README."""
import argparse
import hashlib
import json
import os
import re
import subprocess
from pathlib import Path
from urllib.parse import quote

START = "<!-- cad-still -->"
END = "<!-- /cad-still -->"
# Withdrawn pages are kept byte for byte, so their READMEs are never rewritten.
ARCHIVES = ("docs/detailed-design/archive/",)
STILL = "cad/preliminary-design/media/002-preliminary-robot.png"
RECORD = "cad/preliminary-design/media/002-preliminary-robot.json"


def checked(path, expected):
    data = path.read_bytes()
    if hashlib.sha256(data).hexdigest() != expected:
        raise ValueError(f"Changed CAD evidence: {path}")
    return data


def output(file, data, update):
    previous = file.read_bytes() if file.exists() else None
    if previous == data:
        return
    if not update:
        raise ValueError(f"Stale or missing: {file}")
    file.write_bytes(data)


def run(root, update):
    # No detailed design is published, so every README features the current preliminary design.
    record = json.loads((root / RECORD).read_bytes())
    catalog = root / "docs/preliminary-design"
    current = json.loads((catalog / "releases.json").read_bytes())["current"]
    scene = catalog / "releases" / current
    if record["release"] != current:
        raise ValueError("The preliminary still names a release other than the current one")
    checked(root / STILL, record["pngSha256"])
    # The published receipt names the STEP the layout check and the review scene were built from.
    if json.loads((scene / "receipt.json").read_bytes())["step"]["sha256"] != record["stepSha256"]:
        raise ValueError("Preliminary still and release refer to different geometry")
    still = (root / "cad/preliminary-design/readme-still.md").read_text(encoding="utf-8").strip()
    if still.count(STILL) != 1 or still.count(RECORD) != 1:
        raise ValueError("The README still block must link its image and its record once each")
    paths = subprocess.check_output(["git", "ls-files", "-co", "--exclude-standard", "-z"], cwd=root)
    listed = (n.decode() for n in paths.split(bytes(1)) if n)
    names = sorted(set(n for n in listed if Path(n).name.lower() in ("readme.md", "readme") and not n.startswith(ARCHIVES)))
    for name in names:
        file = root / name
        if not file.exists():
            continue
        original = file.read_bytes()
        text = original.decode("utf-8-sig", errors="surrogateescape").replace("\r\n", "\n")
        text = re.sub(r"\n*" + START + ".*?" + END + r"\n*", "\n\n", text, flags=re.S)
        lines = text.splitlines(keepends=True)
        heading = next((i for i, line in enumerate(lines) if line.startswith("# ")), None)
        if heading is None:
            raise ValueError(f"README needs a title: {name}")

        def link(target):
            return quote(Path(os.path.relpath(root / target, file.parent)).as_posix(), safe="/.-_")

        block = still.replace(STILL, link(STILL)).replace(RECORD, link(RECORD))
        merged = "".join(lines[:heading+1]).rstrip("\n") + "\n\n" + block + "\n\n" + "".join(lines[heading+1:]).lstrip("\n")
        if b"\r\n" in original:
            merged = merged.replace("\n", "\r\n")
        bom = b"\xef\xbb\xbf" if original.startswith(b"\xef\xbb\xbf") else b""
        output(file, bom + merged.encode(errors="surrogateescape"), update)
    print(f"Verified the preliminary still, its release geometry and {len(names)} README files.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("--update", action="store_true")
    args = parser.parse_args()
    run(args.root.resolve(), args.update)

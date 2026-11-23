"""Replace personal machine paths in tracked text files with machine-neutral ones.

Usage: python cad/preliminary-design/source-handoff/scrub_paths.py [--check]

The archived trial scripts and logs were written on one workstation and name its folders. Three of those
folders are this repository's root on that workstation (the original working folder, the main checkout and
any worktree); they become the drive `R:`. Every other path under the home directory becomes `%USERPROFILE%`.
Map the drive before running an archived script:

    subst R: <path to this clone>

Only the path prefix changes. Each changed file is listed in scrub-map.json with its SHA-256 before and
after, so a hash recorded before the scrub can still be traced to the file. Binary files are not touched.
"""
import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
MAP = Path(__file__).resolve().parent / "scrub-map.json"
SEP = rb"(?:\\\\\\\\|\\\\|\\|/)"          # one separator as written: /, \, \\ or \\\\
HOME = rb"(?:[Cc]:|/[Cc]|/mnt/c)" + SEP + rb"[Uu]sers" + SEP + rb"ehart"
ROOTS = [
    HOME + SEP + rb"Documents" + SEP + rb"FRC(?: |%20)Native",
    HOME + SEP + rb"repos" + SEP + rb"frc-2024-poc-worktrees" + SEP + rb"[A-Za-z0-9._-]+",
    HOME + SEP + rb"repos" + SEP + rb"frc-2024-poc(?![A-Za-z0-9._-])",
]
RULES = [(re.compile(b"|".join(ROOTS)), b"R:"),
         (re.compile(HOME), b"%USERPROFILE%"),
         (re.compile(rb"[A-Za-z]--Users-ehart"), b"USERPROFILE")]


def sha(data):
    return hashlib.sha256(data).hexdigest()


def main():
    check = "--check" in sys.argv
    files = subprocess.run(["git", "ls-files", "-z"], cwd=REPO, capture_output=True, check=True).stdout.split(b"\0")
    known = json.loads(MAP.read_bytes())["files"] if MAP.is_file() else {}
    changed = 0
    for raw in filter(None, files):
        name = raw.decode("utf-8")
        path = REPO / name
        if not path.is_file() or path.resolve() == Path(__file__).resolve():
            continue
        data = path.read_bytes()
        if b"\0" in data[:8000] or not re.search(rb"[Uu]sers", data):
            continue
        out = data
        for pattern, value in RULES:
            out = pattern.sub(value, out)
        if out != data:
            changed += 1
            if not check:
                path.write_bytes(out)
                known[name] = dict(before=known.get(name, {}).get("before", sha(data)), after=sha(out))
    if check:
        print(changed, "tracked text files still name a personal machine path")
        sys.exit(1 if changed else 0)
    MAP.write_bytes((json.dumps(dict(
        note="SHA-256 of each file before and after its machine paths were replaced. R: is the repository root.",
        files=dict(sorted(known.items()))), indent=1) + "\n").encode())
    print("rewrote", changed, "files;", len(known), "in the map")


if __name__ == "__main__":
    main()

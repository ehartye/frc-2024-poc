"""Regenerate the four common-tool reports from the season fixtures."""
import argparse
import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def save_report(filename, command):
    output = subprocess.check_output(command, cwd=ROOT)
    data = json.loads(output)
    (HERE / filename).write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n",
                                 encoding="utf8", newline="\n")


def record_solver_sources(scripts):
    pending = ["strategy-lab.mjs", "design-review.mjs", "hardening.mjs",
               "field-rehearsal.mjs", "rehearsal-series-core.mjs"]
    files = {}
    while pending:
        name = pending.pop()
        if name in files:
            continue
        data = (scripts / name).read_bytes()
        files[name] = hashlib.sha256(data).hexdigest()
        for relative in re.findall(rb"""(?:from\s*|import\s*)["'](\./[^"']+\.mjs)["']""", data):
            dependency = (scripts / name).parent / relative.decode("ascii")
            pending.append(dependency.resolve().relative_to(scripts).as_posix())
    path = HERE / "solver-sources.json"
    path.write_text(json.dumps(dict(sorted(files.items())), indent=2) + "\n",
                    encoding="utf8", newline="\n")


def pin_sources(filename):
    """Rebind the known generated receipts after a fresh simulation run."""
    path = HERE / filename
    manifest = json.loads(path.read_bytes())

    def bind(value):
        if isinstance(value, list):
            for entry in value:
                bind(entry)
        elif isinstance(value, dict):
            if "sha256" in value and "path" in value:
                source = (ROOT / value["path"]).resolve(strict=True)
                source.relative_to(HERE)
                value["sha256"] = hashlib.sha256(source.read_bytes()).hexdigest()
            for child in value.values():
                bind(child)

    bind(manifest)
    path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf8", newline="\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scripts", type=Path, help="Agent FRC scripts directory")
    scripts = parser.parse_args().scripts.resolve(strict=True)

    record_solver_sources(scripts)
    save_report("role-results.json", ["node", scripts / "strategy-lab.mjs",
                ROOT / "kickoff/crescendo-field.json", HERE / "roles.json"])
    subprocess.run([sys.executable, "-B", HERE / "evaluate.py"], check=True, cwd=ROOT)
    subprocess.run(["node", HERE / "transport.mjs", scripts], check=True, cwd=ROOT)

    for name in ["architectures.json", "load-inventory.json", "elevation.json", "throughput.json"]:
        pin_sources(name)
    for name, mode in [("elevation", "elevation"), ("throughput", "throughput")]:
        save_report(name + "-results.json", ["node", scripts / "hardening.mjs", mode,
                    HERE / (name + ".json"), ROOT])
    save_report("architecture-results.json", ["node", scripts / "design-review.mjs",
                HERE / "architectures.json", ROOT])

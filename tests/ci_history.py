"""Run the existing season evidence recipes with their original generator revisions."""
import argparse
from contextlib import nullcontext
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parents[1]
PINS = {
    "stage": "6670cb44e3e3f35ae82309d86d18dcb4c069682e",
    "physical": "92ddf41933dd6026b26493c1d2c91505e9d15914",
    "alliance": "344fac71aa4aecbec66cf4015c7646b1149a542b",
    "spatial": "d508f4bcb5d02edfea8766f8839628310564192c",
    "ground": "687618cdeedaca33ba8271e1aab4b7ce2697b805",
}


def replay_tree(temp):
    # Several archived generators write outputs in place. Run those on a copy
    # of the candidate, including local input edits, and compare against its index.
    tree = temp / "project"
    subprocess.run(["git", "clone", "--quiet", "--local", "--no-checkout", str(ROOT), str(tree)], check=True)
    head = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    subprocess.run(["git", "checkout", "--quiet", "--detach", head], cwd=tree, check=True)
    def paths(*argv):
        return set(subprocess.check_output(["git", *argv], cwd=ROOT).decode().split("\0")) - {""}
    for name in paths("diff", "--name-only", "--no-renames", "-z", "HEAD") | paths("ls-files", "--others", "--exclude-standard", "-z"):
        source, target = ROOT / name, tree / name
        if source.is_file():
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, target)
        elif not source.exists():
            target.unlink(missing_ok=True)
    subprocess.run(["git", "add", "-A"], cwd=tree, check=True)
    return tree


def verify(suite, temp):
    writes_project = suite in {"alliance", "ground", "custody", "passive"}
    root = replay_tree(temp) if writes_project else ROOT
    env = dict(os.environ, OPENBLAS_NUM_THREADS="1", PYTHONDONTWRITEBYTECODE="1")
    def run(*argv, output=None):
        with (open(output, "wb") if output else nullcontext()) as stream:
            subprocess.run(argv, cwd=root, env=env, stdout=stream, check=True)
    def python(*args):
        run(sys.executable, "-B", *args)
    def historical(name):
        repo = os.environ.get("AGENT_FRC_PLUGIN_ROOT")
        if not repo:
            raise ValueError("Set AGENT_FRC_PLUGIN_ROOT to the agent-frc Git checkout with historical revisions.")
        folder = temp / name
        archive = temp / (name + ".zip")
        run("git", "-C", repo, "archive", "--format=zip", "--output", str(archive), PINS[name])
        with zipfile.ZipFile(archive) as z:
            z.extractall(folder)
        return folder / "scripts"
    def same(generated, expected):
        if generated.read_bytes() != (root / expected).read_bytes():
            raise ValueError("Replay differs from " + expected)
    if suite == "stage":
        scripts = historical("stage")
        series = temp / "stage.json"
        run("node", scripts / "rehearsal-series.mjs", "kickoff/crescendo-field.json", series)
        same(series, "experiments/speaker-cycles/match-series.json")
        python("tests/test_stage_endgame.py")
    elif suite == "physical":
        scripts = historical("physical")
        series, lab = temp / "physical.json", temp / "physical-lab.html"
        run("node", scripts / "rehearsal-series.mjs", "kickoff/physical-match.json", series)
        same(series, "experiments/physical-match/series.json")
        run("node", scripts / "strategy-lab.mjs", "kickoff/physical-match.json", "experiments/physical-match/roles.json",
            "--html", lab, output=temp / "roles.json")
        python("experiments/physical-match/publish.py", lab)
        same(lab, "docs/field-lab.html")
        python("tests/test_physical_match.py")
    elif suite == "mass":
        result = temp / "r9-mass.json"
        run("node", historical("stage") / "assembly-accounting.mjs", "cad/assembly-readiness/mass/mapping.json", output=result)
        env["R9_CHECKED_MASS"] = str(result)
        python("tests/test_r9_physical_assembly.py")
    elif suite == "alliance":
        python("experiments/alliance-service/replay.py", historical("alliance"))
        run("git", "diff", "--exit-code", "--", "experiments/alliance-service")
        result = temp / "spatial.json"
        run("node", "experiments/alliance-service/spatial-saddle.mjs", historical("spatial"), output=result)
        same(result, "experiments/alliance-service/spatial-saddle.json")
    elif suite == "ground":
        python("experiments/ground-relay/replay.py", historical("ground"))
        python("tests/test_ground_relay.py")
        run("git", "diff", "--exit-code", "--", "experiments/ground-relay", "docs/ground-relay.json")
    elif suite == "custody":
        for script in ("contact", "loads", "rehearse", "lab"):
            python("experiments/registered-custody/" + script + ".py")
        python("tests/test_registered_custody.py")
        run("git", "diff", "--exit-code", "--", "experiments/registered-custody", "docs/ground-contact.json")
    elif suite == "passive":
        python("experiments/passive-entry/replay.py")
        python("tests/test_passive_entry.py")
        run("git", "diff", "--exit-code", "--", "experiments/passive-entry/showcase-audit.json", "experiments/passive-entry/service.json")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("suite", choices=["stage", "physical", "mass", "alliance", "ground", "custody", "passive"])
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="frc-evidence-") as work:
        verify(args.suite, Path(work))

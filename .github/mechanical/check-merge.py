"""Check branch-served Pages bytes against the owner-approved commit and verifier."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

def git(project, *args):
    return subprocess.check_output(["git", "-C", str(project), *args])

def approved_file(project, commit, path):
    return git(project, "show", f"{commit}:{path}")

def audit(project, commit, verifier, field_generator):
    project, verifier, field_generator = (p.resolve(strict=True) for p in (project, verifier, field_generator))
    command = ["node", str(verifier / "scripts/public-model-release.mjs")]
    if (project / "docs").is_symlink():
        raise ValueError("The Pages root must be a real directory")
    with tempfile.TemporaryDirectory() as work:
        temp = Path(work)
        baseline = temp / "approved-docs"
        generated = temp / "independent-field"
        generated.mkdir()
        for entry in git(project, "ls-tree", "-rz", commit, "--", "docs").split(b"\0"):
            if not entry:
                continue
            meta, raw = entry.split(b"\t", 1)
            mode, kind, oid = meta.split()
            if mode not in (b"100644", b"100755") or kind != b"blob":
                raise ValueError(f"Unsupported item in approved Pages tree {raw!r}")
            path = Path(raw.decode("utf-8"))
            target = baseline / path.relative_to("docs")
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(git(project, "cat-file", "blob", oid.decode()))
        policy_path = temp / "served-docs.json"
        policy_path.write_bytes(approved_file(project, commit, ".github/mechanical/served-docs.json"))
        env = os.environ.copy()
        env.pop("FRC_VERIFIED_FIELD_ROOT", None)
        served_policy = json.loads(policy_path.read_bytes())
        for page in served_policy["mutableFieldPages"]:
            if page != "field-lab.html":
                raise ValueError(f"No trusted regeneration rule for {page}")
            if (baseline / page).read_bytes() == (project / "docs" / page).read_bytes():
                continue
            for name in ("kickoff/physical-match.json","experiments/physical-match/roles.json"):
                p=project / name
                if p.is_symlink() or p.resolve()!=p:
                    raise ValueError(f"Redirected field input {name}")
            output = generated / page
            subprocess.run(["node", str(field_generator / "scripts/strategy-lab.mjs"),
                            "kickoff/physical-match.json", "experiments/physical-match/roles.json",
                            "--html", str(output)], cwd=project, stdout=subprocess.DEVNULL, check=True)
            publisher=temp / "publish.py"
            publisher.write_bytes(approved_file(project, commit, "experiments/physical-match/publish.py"))
            subprocess.run([sys.executable, "-I", str(publisher), str(output)], cwd=temp, check=True)
            env["FRC_VERIFIED_FIELD_ROOT"] = str(generated)
        packets = [project / ".github/mechanical/receipts" / name for name in ("branch.json", "docs.json")]
        signed = any(os.path.lexists(packet) for packet in packets)
        key = temp / "reviewer.pem"
        if signed:
            for packet in packets:
                if not packet.is_file() or packet.resolve() != packet:
                    raise ValueError("Both original source and published reviewer packets are required")
            key.write_text(env["REVIEWER_KEY"] + "\n", encoding="ascii")

        def run(args, packet):
            if signed:
                args.extend([str(packet), str(key)])
            return json.loads(subprocess.check_output(command + args, env=env, text=True))

        branch = run(["audit", str(project), commit, ".github/mechanical/public-model.json"], packets[0])
        final = run(["audit-staged", str(baseline), str(project / "docs"), str(policy_path)], packets[1])
        if signed:
            subprocess.run(
                command + [
                    "verify-pair", str(project), str(packets[0]), str(project / "docs"),
                    str(packets[1]), str(key), served_policy["project"],
                ],
                env=env, stdout=subprocess.DEVNULL, check=True,
            )
        print(json.dumps(dict(commit=commit,branch=branch,finalDocs=final,fieldRegenerated=bool(env.get("FRC_VERIFIED_FIELD_ROOT"))),indent=2))

if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("project",type=Path)
    parser.add_argument("commit")
    parser.add_argument("verifier",type=Path)
    parser.add_argument("field_generator",type=Path)
    args=parser.parse_args()
    audit(args.project,args.commit,args.verifier,args.field_generator)

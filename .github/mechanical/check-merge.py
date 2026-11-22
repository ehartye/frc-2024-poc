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
        # A one-time owner-reviewed consolidation can reissue the same preliminary
        # design. Both complete Git trees are named by the trusted predecessor.
        # All other transitions retain the ordinary append-only comparison.
        root = "docs/preliminary-design"
        plan = json.loads(approved_file(project, commit, ".github/mechanical/preliminary-consolidation.json"))
        before = git(project, "rev-parse", f"{commit}:{root}").decode().strip()
        after = git(project, "rev-parse", f"HEAD:{root}").decode().strip()
        consolidate = (before, after) == (plan["beforeTree"], plan["afterTree"])
        if consolidate:
            # Verify the actual candidate bytes and inventory against the reviewed tree.
            release_dir = project / root
            expected = {}
            for entry in git(project, "ls-tree", "-rz", after).split(b"\0"):
                if entry:
                    meta, name = entry.split(b"\t", 1)
                    mode, kind, oid = meta.split()
                    if mode != b"100644" or kind != b"blob":
                        raise ValueError("Unsupported preliminary artifact")
                    expected[name.decode("utf-8")] = oid.decode()
            paths = list(release_dir.rglob("*"))
            if any(path.is_symlink() for path in paths):
                raise ValueError("Redirected preliminary artifact")
            actual = {path.relative_to(release_dir).as_posix(): path for path in paths if path.is_file()}
            if actual.keys() != expected.keys():
                raise ValueError("Unexpected consolidation inventory")
            for name, path in actual.items():
                if path.read_bytes() != git(project, "cat-file", "blob", expected[name]):
                    raise ValueError(f"Unapproved consolidation bytes: {name}")
            # Compare the robot, all trials and every generated mechanical artifact
            # to the deployed issue, even though the presentation is changing.
            preserved = ("scene.json.gz", "budgets.json", "limits.json", "motion.json")
            if plan.get("purpose") == "canonical-review-template":
                issue = f"{root}/releases/003"
                old_record = json.loads(approved_file(project, commit, f"{issue}/release.json"))
                new_record = json.loads((project / issue / "release.json").read_bytes())
                for field in ("id", "issue", "geometry", "cases", "missionCases"):
                    if old_record[field] != new_record[field]:
                        raise ValueError(f"Template upgrade changed the commissioned {field}")
                stock = (verifier / "templates/preliminary-review/review.html").read_bytes()
                for page in ("index.html", "package-motion.html"):
                    if (project / issue / page).read_bytes() != stock:
                        raise ValueError(f"Expected the verified plugin's stock template at {page}")
            else:
                preserved += ("package-motion.geometry.json.gz", "package-motion.html", "receipt.json")
            for name in preserved:
                path = f"{root}/releases/003/{name}"
                if (project / path).read_bytes() != approved_file(project, commit, path):
                    raise ValueError(f"Consolidation changed engineering evidence: {name}")
        for entry in git(project, "ls-tree", "-rz", commit, "--", "docs").split(b"\0"):
            if not entry:
                continue
            meta, raw = entry.split(b"\t", 1)
            mode, kind, oid = meta.split()
            if mode not in (b"100644", b"100755") or kind != b"blob":
                raise ValueError(f"Unsupported item in approved Pages tree {raw!r}")
            path = Path(raw.decode("utf-8"))
            if consolidate and path.is_relative_to(root):
                # The unmodified first-party verifier performs a full initial-site
                # validation of every release, catalog file and commissioned hash.
                continue
            target = baseline / path.relative_to("docs")
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(git(project, "cat-file", "blob", oid.decode()))
        policy_path = temp / "served-docs.json"
        policy_path.write_bytes(approved_file(project, commit, ".github/mechanical/served-docs.json"))
        env = os.environ.copy()
        env.pop("FRC_VERIFIED_FIELD_ROOT", None)
        served_policy = json.loads(policy_path.read_bytes())
        if consolidate:
            served_policy["preliminaryReleases"] = plan["preliminaryReleases"]
            # Use the reviewed presentation policy for future checks as well.
            if served_policy != json.loads((project / ".github/mechanical/served-docs.json").read_bytes()):
                raise ValueError("Consolidated site policy differs from approved policy")
            policy_path.write_text(json.dumps(served_policy), encoding="utf-8")
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

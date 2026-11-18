"""Run the native NOTE experiment from the loose floor start through the handoff lift."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
PRESSURE = ROOT / "sim/pressure"
BOUND = PRESSURE / "evidence/native-signed-20261118"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", required=True, type=Path, help="Private relieved-tread Fusion archive")
    parser.add_argument("--output", required=True, type=Path, help="New private run directory")
    parser.add_argument("--turn", choices=("left", "right", "straight"),
                        help="Drive during the lift after the active stop")
    args = parser.parse_args()
    archive, output = args.archive.resolve(strict=True), args.output.resolve()
    with archive.open("rb") as stream:
        actual = hashlib.file_digest(stream, "sha256").hexdigest()
    binding = BOUND / "native-pressure-binding.json"
    if actual != json.loads(binding.read_bytes())["nativeArchiveSha256"]:
        parser.error("Native archive differs from the pressure contact binding")
    output.mkdir(parents=True, exist_ok=False)

    def execute(script, *arguments):
        subprocess.run([sys.executable, "-B", str(PRESSURE/script), *map(str, arguments)], cwd=ROOT, check=True)

    # The floor producer also records the coast-stop counterexample after transport.
    # Active braking starts at the actual transport endpoint, before that branch.
    ground = output/"ground"
    stop = output/"stop"
    common = ("--project", ROOT, "--binding", binding, "--dt", ".0005")
    execute("run_native_signed.py", *common, "--contract", BOUND/"signed-native-contract.json", "--output", ground)
    execute("continue_native_signed_stop.py", *common, "--trajectory", ground/"receipt.json",
            "--contract", BOUND/"native-signed-active-stop-contract.json", "--output", stop)
    lift = output / "lift"
    inputs = ("--ground", ground/"receipt.json", "--stop", stop/"receipt.json",
              "--archive", archive, "--output", lift)
    if args.turn:
        execute("run_turning_lift.py", *inputs, "--case", args.turn, "--grid", "1024", "--dt", ".00025")
        from verify_turning_lift import assess
        measured, _ = assess(lift)
        (output / "retention.json").write_text(json.dumps(measured, indent=2) + "\n", encoding="utf-8")
        if not measured["passed"]:
            raise SystemExit("Driving lift did not meet its retention and motion criteria")
        print(f"NOTE held through {measured['headingChangeDegrees']:.1f} degrees of heading change.")
        return
    execute("run_retained_route.py", *inputs)
    result = json.loads((output/"lift/result.json").read_bytes())
    if result["outcome"] != "HORIZON":
        raise SystemExit("Lift ended early: " + result["outcome"])
    from verify_retained_route import assess
    _, measured = assess(output / "lift")
    (output / "retention.json").write_text(json.dumps(measured, indent=2) + "\n", encoding="utf-8")
    if not measured["retained"]:
        raise SystemExit("The NOTE did not meet the full-height retention criteria")
    print(f"NOTE held at {measured['finalNoteCenterHeightM']:.3f} m. "
          f"Minimum clearance during the hold was {measured['holdMinimumClearanceM']:.3f} m.")


if __name__ == "__main__":
    main()

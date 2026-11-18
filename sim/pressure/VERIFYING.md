# Verifying current and historical pressure experiments

Source-bound historical receipts intentionally retain older source hashes. Do **not** treat broad `test_pressure*` discovery at a changed source revision as a meaningful all-history test run, and do not update old hashes to make tests pass.

## Current corrected source

Run from the repository root using the documented Python/NumPy environment:

```text
python -m unittest discover -s tests -p test_pressure_fast_geometry.py -v
python -m unittest discover -s tests -p test_pressure_benchmark_receipt.py -v
python -m unittest discover -s tests -p test_pressure_corrected_lineage.py -v
python -m unittest discover -s tests -p test_pressure_early_policy.py -v
python -m unittest discover -s tests -p test_pressure_historical.py -v
```

## Historical selected tests

Use a clone containing full Git history. Each output path must be new:

```text
python sim/pressure/verify_historical.py entry artifacts/history-entry.json
python sim/pressure/verify_historical.py half-step artifacts/history-half.json
python sim/pressure/verify_historical.py continuation artifacts/history-continuation.json
python sim/pressure/verify_historical.py late-policy artifacts/history-late-policy.json
```

The verifier accepts only a fixed reviewed registry of receipt hashes, dependency paths/hashes and test hashes. It extracts a known tracked snapshot into a temporary directory, rejects links/special archive entries and unsafe paths, restores only registered dependency blobs whose SHA256 matches, executes only named unittest files and rechecks dependencies afterward. It does not alter the working tree or repin receipts. Original commit IDs are unnecessary; exact blobs must be available in history. A shallow clone missing them fails closed. The executable is the current Python environment; no software installation or arbitrary shell command is performed.

Recorded selected-test counts: entry7, half-step1, continuation6, late-policy6. These are test replays, not fresh full-duration trajectory reruns. The corrected 0-to0.68s trajectory was separately rerun from rest and matches all680 physical rows of the old segmented trajectory exactly. Its new source identity is distinct; old receipts remain historical.

The verifier is an additive workflow subject to independent review before public use. Numerical agreement is not material calibration, mechanical acceptance or capture proof.

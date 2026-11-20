# Historical 2024 A2 campaign

The original A2 runner, its Java project, CAD inputs and receipts share paths relative to this directory. Run `run-a2.ps1` here with the documented JDK and Gradle arguments. The project-root `run.ps1` continues to launch the earlier preloaded regression. [A2 simulation](A2-MISSION-SIMULATION.md) · [Original source hashes](PROVENANCE.json)

The campaign's CAD snapshot preserves all fifteen filenames and byte hashes listed in the original provenance record. Its contract is also present in `mission/`. The active CAD authoring files are under the project-root `cad/`. The student blockout and its field-lab link now live under `kickoff/blockout/`. [Student blockout](../../kickoff/blockout/README.md)

For historical pressure-entry or policy replay, run `python scripts/replay_history.py artifacts/pressure-history` from the project root. This writes a new working copy with the original paths, the recorded contact solver and test, the pressure study's later CAD contract and the original blockout index. Run the historical checks from that copy. The input register identifies both CAD revisions and the restored source hashes. [Historical input register](../pressure-entry/input-register.json)

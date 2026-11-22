# Release 004 plan

Working record for building preliminary design release 004 of the guided cassette. A new session resumes from this file and the branch `feature/release-004`. Keep the three lists current.

Target: the whole mission cycle checked for clearance, mass and voltage; every mission-critical interaction bound to a qualified physics trial; every open risk closed or carried as a bench test with pass and fail numbers. Release 004 enters as in-progress. The owner decides whether it becomes effective.

## Inputs

- Evidence tree: `2024/preliminary-review-lan/greenfield-retained/` (to be relocated; one constant, `EVIDENCE` in `cad/preliminary-design/build-004/timeline.py`, names it).
- Layout solids: `guided-hoist-01/service-locked-robot.step` (232 solids, SHA-256 `a29a146d…bc88`). Release 004 uses the same solids with repaired names: `cad/preliminary-design/releases/004/robot.step`, written by `build-004/step_names.py`.
- Scoring cycle traces: `counted-reset-power-02/` fine nominal run (`run-dev-96.json`), first NOTE (`dev-middle-96.json`, `split-tail_.25_0-dev-middle-96-6.88.json`, `empty-dev-middle-96.json`, `empty-dev-outer-96.json`).
- Climb traces: `counted-reset-power-02/climb-gross-front.json` and `climb-gross-rear.json`.
- Rigid-group membership: the `rig` maps of the published release 003 replay scene (`docs/preliminary-design/releases/003/scene.json.gz`, SHA-256 `5a1a5861…`).
- Plugin: Agent FRC 0.111.1 (`4ab39ad`); readiness report from branch `feature/effective-readiness` (draft PR 178).
- Python: the `frc-cad` environment (`cadquery-ocp`, NumPy). Run plugin scripts with `-B`, not `-I`.

## Done

- Inventory of the pushed inputs: all 3,106 manifest files hash correctly.
- Release 003 reproduces on plugin 0.111.1: nine cases, every pair row, the rejected control and both budgets are identical. Only the view digest differs, because the stock page changed after the release was issued.
- `build-004/step_names.py`: repairs the 144 double-encoded names in the STEP (254 lines changed, geometry bytes untouched). Result SHA-256 `91ac459b…0b74`, 232 solids.
- `build-004/kin.py`: poses of each rigid group and the motion-study operations.
- `build-004/timeline.py`: joint timeline of one scoring cycle from the pinned traces. It agrees with the published replay within 0.0002 mm on 307 compared frames.
- Timing probe: 0.2 s of loaded lift checks in 150 s. The only unresolved pairs are the five joints (pivot shaft in both bearings and the reducer, flap axle in the leaf, slide screw in the travel nut) and the declared NOTE contacts.

## Findings

1. **Scoring repeatability is failed, not open.** `counted-reset-power-02` holds run files for R01–R07: R02 (2 scored), R03 (5 by the 7 s window; the sixth counts at 118.26 s when observed longer) and R06 (0 scored) failed. Thirteen starts are unrun. The frozen rule (two failures end the campaign) is already lost at the 6.88 m/s command.
2. **No trial can be rerun on this machine yet.** MuJoCo 4.0.1 is not installed, and the shot and counter solvers read `Documents/FRC Native/2024/six-speaker-20261118/` (`field-query-libs`, `field-source`), which is absent. That folder also holds the only official field CAD.
3. **Scene size.** Plugin 0.111.1 tessellates every body again for each start pose. A whole-cycle study that follows the traces within 1–2 mm needs about 110 cases and would give a scene near 250 MB against the 128 MB limit. Reported to the team lead with two options (plugin stores one mesh per body, or coarse cases with chord-deviation allowances).
4. **View location.** Plugin 0.111.1 refuses a view in a sub-folder when the repository is the project root. Until that changes the release folder is the check's project, as in release 003.
5. **Evidence gaps for the trial qualifications** (control, brackets, timestep): no timestep repeat for the guided climb; no single-input failing bracket for floor pickup or for retention during the lift; the field-counter bracket has no failing magnitude.

## Builder

`python cad/preliminary-design/build-004/build.py PLUGIN` writes every check input into this folder. `--count` prints the case count per phase and writes nothing. `--only reset,acquire,...` builds some phases for a trial run. `--tolerance MM` sets the largest departure of a case path from its trace (default 1.0). `--battery` adds the battery lift at the service index (not yet passing; see below).

| File | Role |
| --- | --- |
| `step_names.py` | Repairs the STEP names |
| `kin.py` | Poses and motion-study operations |
| `timeline.py` | Scoring-cycle joint timeline from the traces |
| `segments.py` | Rigid-class poses, case segmentation by path deviation |
| `roster.py` | Rigid class, plan group, review text and mount per solid |
| `ledgers.py` | Mass lines and battery duties from the evidence |
| `evidence.py` | Trial qualifications, narrative coverage, presentation, missions |
| `risks.py` | Risk register with bench tests |
| `build.py` | Orchestrator; also writes the joint qualifications |

Check: `python -B PLUGIN/scripts/preliminary_design.py cad/preliminary-design/releases/004 layout.json --write-view --output receipt.json --require-checked`. Readiness: `node PLUGIN/scripts/design.mjs effective-readiness cad/preliminary-design/releases/004`.

At 1.0 mm the cycle is 139 cases: reset 25, acquire 49, lift 32, feed 4, flight 10, stow 1, approach 3, capture 5, hoist 7, hold 3. A NOTE-only case checks in about 10 s, a cassette-moving case in 100 to 150 s.

## Partial runs so far

- Feed, flight and stow (22 cases): all pass, the displaced control is rejected, mass and voltage pass.
- Endgame (approach, capture, hoist, hold): running, to find the joint pairs of the hoist.

## Next steps

1. Add the hoist joint pairs the endgame run reports to `JOINTS` in `build.py`.
2. Full build and full check (about 2.5 hours). Commit the generated inputs and the receipt.
3. Write the view once the plugin stores one mesh per body (plugin branch `feature/scene-size`), or cut to coarse cases if told to.
4. Battery lift at the service index: decide how the disconnected pack lead is represented.
5. Fixtures: a project-drawn speaker opening and stage need a drive case that moves the field in the robot frame, because every case must carry the same fixtures and the cycle must be continuous. The working chain deforms and needs the plugin's deforming fixture.
6. Trials still needed for criterion 2 (need MuJoCo 4.0.1; shots also need the missing field folder): a single-input failing bracket for pickup and for retention on the lift, and a timestep repeat of the guided climb.
7. Readiness report, independent review, handoff document, report to the team lead.

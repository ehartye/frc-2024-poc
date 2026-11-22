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

## Next steps

1. `build-004/build.py`: regroup the 232 solids into rigid, functional groups; generate the cycle cases (reset, acquire, lift, feed, flight, stow, climb, hold) and the service cases from the traces; write `design.json`, `motion.json`, `budgets.json`, `limits.json`, `missions.json`, trial records and `layout.json`.
2. Joint interface qualifications for the five joint pairs in every case that moves them.
3. Fixtures: project-drawn speaker opening, stage and chain as STEP fixtures (no official field CAD is in the repository).
4. Run the check; fix what it finds; write the view once the scene-size decision is made.
5. Risk register with bench tests; readiness report; independent review; handoff document.

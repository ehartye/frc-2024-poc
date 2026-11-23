# Release 004 plan

Working record for preliminary design release 004 of the guided cassette. A new session resumes from this file and the branch `feature/release-004`.

`EV/` below means `2024/preliminary-review-lan/greenfield-retained/`.

## Target

Release 004 enters as in-progress. The owner decides whether it becomes effective. The owner then does the detailed design in Fusion, starting from `DETAILED-DESIGN-HANDOFF.md`.

Pass criterion for the preliminary design (owner decision, 2026-11-22): the NOTE passes wholly through the speaker opening and is eventually counted. The 7 s count rate and the target of six scores on 19 of 20 starts are bench tests with pass and fail numbers. They are not more simulation. If only a narrower window meets a target, state the window and bench-test it.

## What 004 is

Release 004 is a revised layout. It is not release 003 with repaired names.

- Revision 004-A raises both launch belt motors 60 mm along the cassette normal on lengthened shafts. The whole-cycle check found each motor can exactly at the NOTE outer radius (177.8 mm). Source: `cad/preliminary-design/build-004/revise_step.py`, `step-revision.json`.
- The firing lane moves from y = 5.871 m to y = 5.821 m. The smallest opening margin over 34 perturbed shots rises from 17.6 mm to 56.7 mm. Source: `operating-point.json`, `EV/scoring-window-04/summary.json`.
- The launch angle stays 122° (decided 2026-11-22). The window is 121° to 125°. 123° has more margin but counts 9 of 12 nominal NOTES against 12 of 12, so it is recorded as an alternative.
- The 144 double-encoded component names are repaired.

## Done

- Trials run on this machine. They need a separate environment: see `REPRODUCIBILITY.md`.
- Single-shot scoring campaign, about 1,900 shots: `EV/scoring-window-04/` (`campaign.py`, `analyse.py`, `results/*.jsonl`, `summary.json`).
- Pickup brackets: `EV/pickup-brackets-04/results.json`. Retention brackets: `EV/retention-brackets-04/results.json`. Climb brackets: `EV/climb-brackets-04/results.json`.
- Risk register with bench tests: `cad/preliminary-design/build-004/risks.py`, written into `missions.json`. Review: `RISK-REVIEW.md`.
- Handoff for the Fusion author: `DETAILED-DESIGN-HANDOFF.md`.

## Open

1. The literal criterion is not met for 3 of 34 perturbed NOTES at the released point. They bounce back out through the opening within 3 s without reaching a beam. 31 are counted. The owner decides; bench tests B1 and B2 of the handoff carry it.
2. Why those three carries bounce out. Nothing recorded at release separates them. NOTE attitude and spin at the opening were not compared.
3. Archived scripts outside the release's trials still name the `R:` drive. The scoring campaign no longer does. See `REPRODUCIBILITY.md`.
4. The mass ledger adds the lengthened launch shafts of revision 004-A from the layout solids (0.074 kg). The taller motor mounts are not drawn or estimated, and the voltage budget is the study made before the revision.
5. Launch angle 122° is 1° above a thin low edge. Bench test B5.
6. Battery lift at the service index: not a checked case.
7. Independent review, status decision by the owner.

## Builder

`python cad/preliminary-design/build-004/build.py PLUGIN` writes the check inputs into this folder. `--count` prints the case count per phase and writes nothing. `--only reset,acquire,...` builds some phases. `--tolerance MM` sets the largest departure of a case path from its trace. `--battery` adds the battery lift at the service index.

| File in `build-004/` | Role |
| --- | --- |
| `revise_step.py` | Writes `robot.step`: repaired names and revision 004-A |
| `step_names.py` | Earlier name-only repair, kept for reference |
| `kin.py` | Poses and motion-study operations |
| `timeline.py` | Scoring-cycle joint timeline from the traces |
| `segments.py` | Rigid-class poses, case segmentation by path deviation |
| `roster.py` | Rigid class, plan group, review text and mount per solid |
| `ledgers.py` | Mass lines and battery duties from the evidence |
| `evidence.py` | Trial qualifications, narrative coverage, presentation, missions |
| `risks.py` | Risk register with bench tests |
| `replays.py`, `replay_set.py` | Recorded trial replays for the review view |
| `build.py` | Orchestrator; also writes the joint qualifications |

Check: `python -B PLUGIN/scripts/preliminary_design.py cad/preliminary-design/releases/004 layout.json --write-view --output receipt.json --require-checked`.

# Guided-cassette engineering handoff

<!-- cad-still -->
[![Preliminary design of the robot](../media/guided-cassette-preliminary.png)](https://ehartye.github.io/frc-2024-poc/preliminary-design/)

**Project CAD · Preliminary design of the robot · Guided cassette 1.0 · effective.** The robot is at the start of a cycle, with a floor NOTE ahead. The angled cassette picks up, retains and shoots a NOTE. The parallel guides support the climb. The still shows the layout solids of the preliminary design review in its opening pose. [Open the design review](https://ehartye.github.io/frc-2024-poc/preliminary-design/)
<!-- /cad-still -->

This branch supplies source material for the 2024 guided cassette. The published review is Guided cassette 1.0, which was drawn on this source. The 19-of-20 scoring campaign requires its own results.

| Requested source | Location |
| --- | --- |
| Parked layout, 232 solids | [guided-hoist-01/service-locked-robot.step](../../../2024/preliminary-review-lan/greenfield-retained/guided-hoist-01/service-locked-robot.step). |
| Original engineering repository | [Git bundle parts and restore tool](../../../engineering-iterations/). The bundle contains the history through d4ebf59, including 8b7559e. The [selection](../../../iterations/active.json), [foundation](../../../iterations/002/design/foundation.md) and [handoff](../../../iterations/002/design/cad/guided-cassette/HANDOFF.md) also open directly on this branch. |
| Complete trial and control folder | [2024/preliminary-review-lan/greenfield-retained](../../../2024/preliminary-review-lan/greenfield-retained/). Includes adverse starts, failed controls, raw outputs and replay scripts. |
| Package builder, motion and budget | The commissioning script and the builder are in the Git history at commit `8e5d857`; [release inputs](../releases/1.0/) they wrote, [package geometry and generators](../../../2024/preliminary-review-lan/greenfield-retained/guided-hoist-01/), [replay scene](../builder/inputs/replay-scene.json.gz). |
| Scoring, climbing and service traces | [48- and 96-resolution scoring trajectories](../../../2024/preliminary-review-lan/greenfield-retained/shared-counter-03/), [climb solver and counter-clock transfer including front/rear trials](../../../2024/preliminary-review-lan/greenfield-retained/counted-reset-power-02/), [two-hook winch and return cases](../../../2024/preliminary-review-lan/greenfield-retained/guided-hoist-01/), [supported slide motor service](../../../2024/preliminary-review-lan/greenfield-retained/mounted-slide-02/). The [LAN data](../../../2024/preliminary-review-lan/www/data.json) contains the four named replays under concepts.guided.cases and the cartridge withdrawal under slideService.case. |
| Native wear blade and both trials | [Editable Fusion blade, STEP and placement readback](../../../iterations/002/design/cad/guided-cassette/profile-a/), [lip STEP](../../../2024/preliminary-review-lan/greenfield-retained/pickup-lip/shared-cassette-lip.step), [swappable-blade trial STEP](../../../2024/preliminary-review-lan/greenfield-retained/swap-blade/shared-cassette-swappable.step). |
| Authority and lineage | [Root historical authority](../../fusion-authority.json) identifies Material-03. The [iteration authority](../../../iterations/002/design/cad/fusion-authority.json) and [its actual lineage](../../../iterations/002/design/cad/linked-side-drive/first-party-lineage/lineage.json) identify the earlier side-finger robot. The guided-cassette record is [its handoff evidence](../../../iterations/002/design/cad/guided-cassette/handoff-evidence.json) and [native blade readback](../../../2024/private-readiness-20261120/iteration-002-native/poc24-guided-profile-a03/readback.json). The named engineering commit has no top-level cad/lineage.json and identifies the blade as the first native detail. |
| Earlier toe actuator | [plugin-qa/toe-actuator-20261121](../../../plugin-qa/toe-actuator-20261121/). Keep this earlier architecture separate from the cassette selection. |

## Use the source on another station

From a clone of ehartye/frc-2024-poc with Git LFS installed:

```sh
git fetch origin private/2024-guided-cassette
git switch private/2024-guided-cassette
git lfs pull
python engineering-iterations/restore.py ../engineering-iterations
```

The last command is optional. It creates the complete original repository checkout, including the historical Fusion assemblies and both requested commits. The direct STEP, blade, scripts and replay outputs can be used from the season branch. The raw branch takes the 2024 folder and plugin-qa folder relative to the original FRC Native directory. [Direct file inventory](source-files.json)

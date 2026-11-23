# Builder inputs carried from the draft layout

<!-- cad-still -->
[![Preliminary design of the robot](../../media/guided-cassette-preliminary.png)](https://ehartye.github.io/frc-2024-poc/preliminary-design/)

**Project CAD · Preliminary design of the robot · Guided cassette 1.0 · effective.** The robot is at the start of a cycle, with a floor NOTE ahead. The angled cassette picks up, retains and shoots a NOTE. The parallel guides support the climb. The still shows the layout solids of the preliminary design review in its opening pose. [Open the design review](https://ehartye.github.io/frc-2024-poc/preliminary-design/)
<!-- /cad-still -->

Guided cassette 1.0 was drawn on a draft layout of 232 solids. The drafts were published while the design was in progress and left the site when the design was numbered. Revision A is the one geometry change from the draft layout to Guided cassette 1.0: both launch belt motors are raised 60 mm ([revise_step.py](../revise_step.py)).

Guided cassette 1.0's builder read these files from the drafts. Their bytes are unchanged. The builder itself was removed when Guided cassette 1.1 became a design source; it is in the Git history at commit `8e5d857`.

| File | Content | SHA-256 | Read by |
| --- | --- | --- | --- |
| `draft-design.json` | Editable envelope of the draft layout | `c09b9c28bebcb3601183995e468d587ba22a0d59e324c07dcad82ebdfb349abf` | `build.py` |
| `draft-budgets.json` | Mass lines of the draft layout | `a6394d7eb0245db1c4d79f563b6f5ee7fae309470b835fc91566cda3e4ed0354` | `ledgers.py` |
| `replay-scene.json.gz` | Replay scene of the draft layout: the rigid body of each solid and the grouped transforms of the four recorded runs | `5a1a5861cc9df6e3fc43ec7038701a64322de008b01f0e557416839b7943e0b5` | `roster.py`, `replays.py`, `replay_set.py`, the three commissioning scripts in the parent folder |
| `replay-scene-record.json` | Record published with the replay scene: its SHA-256 and the SHA-256 of the STEP it was drawn on | `b437ca19efecf8976a5e274f49b7e997acc9970bc4d26ebd2a702a4b157d13fa` | The three commissioning scripts in the parent folder |
| `official-field-replays.json.gz` | The four recorded runs with the official speaker and stage, tessellated by [field.py](../field.py) from FIRST's 2024 field STEP | `1603cfe8ef4912256538878259ac5418e01380163a5b6539675710fcafc6ca0d` | `build.py` |

The draft layout STEP is `2024/preliminary-review-lan/greenfield-retained/guided-hoist-01/service-locked-robot.step`. The trial files of the four runs are in `2024/preliminary-review-lan/greenfield-retained/counted-reset-power-02/`. The builder checks each one against the SHA-256 the replay set holds for it.

The drafts and the script that assembled `official-field-replays.json.gz` are in the Git history at commit `4f9a79b`.

# 2024 guided-cassette preliminary design

The [preliminary design](https://ehartye.github.io/frc-2024-poc/preliminary-design/) opens Guided cassette 1.0 in the stock Agent FRC review page. The page has one view and one motion list. The list holds the checked cycle and service cases, the displaced control, both six-NOTE scoring clocks and the front and rear climbs. Add `?case=ID` to the release page URL to open one entry, for example `releases/1.0/?case=continuous-cycle-96`.

Run `python cad/preliminary-design/serve.py` from the season checkout to serve the page locally.

## What is checked and what is replayed

The layout check follows one whole cycle on all 232 robot and piece solids: reset, pickup, lift, feed, flight and stow, then the chain approach, capture, hoist and hold. It also checks the straight withdrawal of the service index pin. Those cases have collision, mass and loaded-voltage results in `receipt.json`.

The four match replays play back the pinned trial files in `2024/preliminary-review-lan/greenfield-retained/counted-reset-power-02/`. A replay is recorded evidence. It is not a clearance case, and the receipt's decision does not depend on it. Each replay lists the motions taken from its trial files and the motions prescribed for display. The runs were recorded on the draft layout, before revision A raised the launch motors. [Draft layout and revision A](builder/inputs/README.md)

The speaker, stage and chains are tessellated from FIRST's `FE-2024.STEP` for display. The trials use their own contact surfaces.

## Rebuild the release

The inputs are in `cad/preliminary-design/releases/1.0/`. `PLUGIN` is a checkout of Agent FRC at the `AGENT_FRC_REF` in `.github/workflows/design-publication.yml`. Use the Agent FRC geometry environment.

1. Write the check inputs. The builder reads the pinned traces and ledgers and the [carried inputs](builder/inputs/README.md), and checks each trial file against its SHA-256.

   ```sh
   python cad/preliminary-design/builder/build.py PLUGIN --tolerance 40 --refine acquire=1,feed=1
   ```

2. Run the layout check, then stage and check the release.

   ```sh
   node PLUGIN/scripts/design.mjs preliminary-check . cad/preliminary-design/releases/1.0/layout.json --write-view --require-checked --output cad/preliminary-design/releases/1.0/receipt.json
   python cad/preliminary-design/stage.py 1.0
   node PLUGIN/scripts/design-publication.mjs check . --skip-rerun
   python scripts/cad-readmes.py . --update
   ```

Guided cassette 1.0 is effective, so staging it again is refused. A change to the design is staged as `1.1`. [Name and number a design](https://github.com/ehartye/agent-frc/blob/main/docs/design-publication.md#name-and-number-a-design)

The field parts in the carried replay set were tessellated with `python cad/preliminary-design/builder/field.py FE-2024.STEP field-meshes.json`. Download FIRST's 2024 field STEP archive and pass its `FE-2024.STEP`. The script checks the file's SHA-256.

The **Design publication** workflow reruns the layout check from the committed inputs on every push. [Publish and check designs](https://github.com/ehartye/agent-frc/blob/main/docs/design-publication.md)

## Measured margins to check on the prototype

At discharge the unloaded NOTE passes the right entry bearing bracket with a smallest sampled clearance of 0.035 mm. An independent interval calculation bounds the nominal clearance above 0.015 mm throughout that segment. Assembly error or variation in the NOTE could consume that margin. Check it on the prototype. [Bracket clearance calculation](motion-evidence-002/exit-bracket-clearance.json)

The center intake belt has an assumed one-sided NOTE indentation limit of 8 mm. The two replay resolutions reach 7.67 and 7.55 mm. A 7 mm limit rejects both. Each additional allowance names its own contact surface. The bracket uses rigid geometry. [Contact assumptions](compression-limits-002.json) · [Two-clock measurements](compression-results-002.json)

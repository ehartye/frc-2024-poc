# 2024 guided-cassette preliminary design

The [preliminary design](https://ehartye.github.io/frc-2024-poc/preliminary-design/) opens Guided cassette 1.0 in the stock Agent FRC review page. The page has one view and one motion list. The list holds the checked cycle and service cases, the displaced control, both six-NOTE scoring clocks and the front and rear climbs. Add `?case=ID` to the release page URL to open one entry, for example `releases/1.0/?case=continuous-cycle-96`.

The page loads its scene over HTTP. To view it locally, serve `docs/` with any static file server, for example `python -m http.server -d docs`, and open `/preliminary-design/`.

## What is checked and what is replayed

The layout check follows one whole cycle on all 232 robot and piece solids: reset, pickup, lift, feed, flight and stow, then the chain approach, capture, hoist and hold. It also checks the straight withdrawal of the service index pin. Those cases have collision, mass and loaded-voltage results in `receipt.json`.

The four match replays play back the pinned trial files in `2024/preliminary-review-lan/greenfield-retained/counted-reset-power-02/`. A replay is recorded evidence. It is not a clearance case, and the receipt's decision does not depend on it. Each replay lists the motions taken from its trial files and the motions prescribed for display. The runs were recorded on the draft layout, before revision A raised the launch motors. [Draft layout and revision A](builder/inputs/README.md)

The speaker, stage and chains are tessellated from FIRST's `FE-2024.STEP` for display. The trials use their own contact surfaces.

## Rebuild the release

Guided cassette 1.0 is effective and does not change. Its inputs in `cad/preliminary-design/releases/1.0/` were written by a builder that has since been removed; it is in the Git history at commit `8e5d857`.

Guided cassette 1.1 is the same design as data. Its folder, `cad/preliminary-design/releases/1.1/`, holds the design source `design.json`, the judgement file `judgement.json`, the trials under `trials/` and what the design library generates from them. No file in this repository builds, measures or checks it. The person or coding agent doing the design work runs, from the repository root, with the design library `frc-design-kernel` installed and their own copy of Agent FRC shown as `PLUGIN`:

```sh
python -m frc_design_kernel build cad/preliminary-design/releases/1.1/design.json --out cad/preliminary-design/releases/1.1/generated
python -m frc_design_kernel try cad/preliminary-design/releases/1.1/design.json --out cad/preliminary-design/releases/1.1/trials
node PLUGIN/scripts/design.mjs preliminary-check . cad/preliminary-design/releases/1.1/judgement.json --write-view --output cad/preliminary-design/releases/1.1/receipt.json
python PLUGIN/scripts/stage_preliminary_review.py 1.1
node PLUGIN/scripts/design-publication.mjs check .
```

A change to the page alone, such as a title, is rebuilt with `--view-only` in place of `--write-view`. [Name and number a design](https://github.com/ehartye/agent-frc/blob/main/docs/design-publication.md#name-and-number-a-design)

The project's CI runs no layout check. It scans every push for personal paths and secrets and checks that the published site is whole. [Project checks](../../tests/project-checks.md)

## Measured margins to check on the prototype

At discharge the unloaded NOTE passes the right entry bearing bracket with a smallest sampled clearance of 0.035 mm. An independent interval calculation bounds the nominal clearance above 0.015 mm throughout that segment. Assembly error or variation in the NOTE could consume that margin. Check it on the prototype. [Bracket clearance calculation](motion-evidence-002/exit-bracket-clearance.json)

The center intake belt has an assumed one-sided NOTE indentation limit of 8 mm. The two replay resolutions reach 7.67 and 7.55 mm. A 7 mm limit rejects both. Each additional allowance names its own contact surface. The bracket uses rigid geometry. [Contact assumptions](compression-limits-002.json) · [Two-clock measurements](compression-results-002.json)

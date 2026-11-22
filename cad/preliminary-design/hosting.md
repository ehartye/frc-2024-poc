# 2024 preliminary design releases

The season's GitHub Pages source is `main:/docs`. Open [the release index](https://ehartye.github.io/frc-2024-poc/preliminary-design/) for the latest review, or pin a specific revision with [release 001](https://ehartye.github.io/frc-2024-poc/preliminary-design/?release=001) or [release 002](https://ehartye.github.io/frc-2024-poc/preliminary-design/?release=002). The optional `case` parameter opens a named motion.

From the season checkout, run `python -m http.server 8000 --directory docs` and open `http://localhost:8000/preliminary-design/`. For a LAN review, use the host's LAN address in the browser. The static files also run under the repository subpath on GitHub Pages.

Each numbered directory contains its launch page, scripts, styling, compressed scene, budgets, motion traces and release record. The viewer loads the compressed JSON in the browser. A new release gets a new directory and an entry in `releases.json`; append the new ID in chronological order and update `current` after verifying the release. The selector reads descriptive titles from each revision and shows the newest first. Keep previously published directories intact.

The geometry is a preliminary robot blockout. The field display uses dimensioned presentation primitives. Inspect the evidence and open qualifications within each replay before using a simulation result for a design decision.

### Compare a viewer change with the previous revision

Serve the reference checkout and the candidate on separate ports. Run these commands with the same Playwright installation, browser and machine. Set `BROWSER_CHANNEL=msedge` to select an installed Edge browser.

```sh
node cad/preliminary-design/check-viewer.cjs http://localhost:8000/preliminary-design/ > reference.jsonl
node cad/preliminary-design/check-viewer.cjs http://localhost:8001/preliminary-design/ reference.jsonl
```

The comparison opens all six featured links across both revisions and exercises the camera, inventory, part focus, playback, labels, floor and load drill-down. It compares opening canvas pixels, end-of-motion readouts, inventory counts and the rendered ledger text. Keep both browser runs on the same rendering backend for a meaningful pixel comparison.

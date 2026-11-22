# 2024 preliminary design releases

The season's GitHub Pages source is `main:/docs`. Open [the release index](https://ehartye.github.io/frc-2024-poc/preliminary-design/) for the latest review, or pin a specific revision with [release 001](https://ehartye.github.io/frc-2024-poc/preliminary-design/?release=001) or [release 002](https://ehartye.github.io/frc-2024-poc/preliminary-design/?release=002). The optional `case` parameter opens a named motion.

From the season checkout, run `python -m http.server 8000 --directory docs` and open `http://localhost:8000/preliminary-design/`. For a LAN review, use the host's LAN address in the browser. The static files also run under the repository subpath on GitHub Pages.

Each numbered directory contains its launch page, scripts, styling, compressed scene, budgets, motion traces and release record. The viewer loads the compressed JSON in the browser. A new release gets a new directory and an entry in `releases.json`; append the new ID in chronological order and update `current` after verifying the release. The selector reads descriptive titles from each revision and shows the newest first. Keep previously published directories intact.

The geometry is a preliminary robot blockout. The field display uses dimensioned presentation primitives. Inspect the evidence and open qualifications within each replay before using a simulation result for a design decision.

# 2024 guided-cassette preliminary design

The [preliminary viewer](https://ehartye.github.io/frc-2024-poc/preliminary-design/) opens the current six-NOTE run. Select the other scoring clock or either climb approach from the motion menu. Copy the page URL to share the selected motion. The robot, four replays, weight and power ledgers, mechanical companion, and FIRST field fixtures ship together in `docs/preliminary-design/releases/003/`.

Git preserves the earlier presentation iterations. Earlier catalog links using `?release=001`, `002` or `003` open this same design. Both current scoring runs use the official blue speaker. Both climbs use the official blue stage, anchors and chains. FIRST supplies the field archive directly to the browser. The NOTE follows the measured free-piece trajectory using the project’s simulation geometry.

### What the mechanical replay checks


The season viewer plays the continuous six-NOTE trials and both climb approaches. Its [companion mechanical review](../../docs/preliminary-design/releases/003/package-motion.html) follows one discharge from the fine-clock trial on all 232 robot and piece solids. It also shows straight withdrawal of the service index pin. Those operations have their own collision, mass and loaded-voltage records. The field trial and mechanical review answer different questions.

At discharge the unloaded NOTE passes the right entry bearing bracket with a smallest sampled clearance of 0.035 mm. An independent interval calculation bounds the nominal clearance above 0.015 mm throughout that segment. Assembly error or variation in the NOTE could consume that margin. Check it on the prototype. [Bracket clearance calculation](motion-evidence-002/exit-bracket-clearance.json)

The center intake belt has an assumed one-sided NOTE indentation limit of 8 mm. The two replay resolutions reach 7.67 and 7.55 mm. A 7 mm limit rejects both. Each additional allowance names its own contact surface. The bracket uses rigid geometry. [Contact assumptions](compression-limits-002.json) · [Two-clock measurements](compression-results-002.json)

### Compare a viewer change with the previous revision

Serve the reference checkout and the candidate on separate ports. Run these commands with the same Playwright installation, browser and machine. Set `BROWSER_CHANNEL=msedge` to select an installed Edge browser.

```sh
node cad/preliminary-design/check-viewer.cjs http://localhost:8000/preliminary-design/ > reference.jsonl
node cad/preliminary-design/check-viewer.cjs http://localhost:8001/preliminary-design/ reference.jsonl
```

The comparison exercises all four current replays and checks that older catalog links resolve. It compares the final readouts, inventory counts and weight and power ledgers with the reference. It also checks camera controls, selection, speaker cutaway, phone framing, and which of the three stage chains moves during each climb.

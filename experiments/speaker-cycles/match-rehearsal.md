# Three six-robot CRESCENDO rehearsals

Open the [field lab](https://ehartye.github.io/frc-2024-poc/field-lab.html#rehearsal) and choose **Play match series at 10×**. For offline use, open the committed [standalone page](../../docs/field-lab.html) in a browser. The three matches run for 45 seconds of playback, with 15 seconds of AUTO and 135 seconds of TELEOP on each game clock. The robot cards stay under their own alliance when the user alternates red and blue additions.

| Match | Change in the scenario | Blue / red NOTE points |
| --- | --- | --- |
| Wing opening | Each alliance starts with local scoring, then the red partners take assigned midfield lanes. | 17 / 19 |
| Shared center | Additional partners converge on the same midfield corridor. Robot footprints prevent both from occupying it. | 16 / 16 |
| Miss and stage | Blue 1 misses a prescribed shot and returns to the SOURCE. Red 3 attempts a TRAP with an unresolved mechanism. | 26 / 18 |

The profiles assign six robots individual AUTO and TELEOP programs. The field has six wing NOTES, five shared center NOTES, blue and red SOURCES, Speakers, AMPs and three TRAP faces per STAGE. Each claim on a NOTE consumes a specific identity. A robot can carry one. A scored or missed piece leaves its cargo. The score records scripted ordinary Speaker and AMP deliveries. It excludes amplification, mobility, penalties, climb and the unresolved TRAP. Robot traffic and empty stations can prevent a scripted attempt. A score cannot occur without available cargo. [Six-robot role descriptions](../../kickoff/alliance-profiles.md) · [Editable profile, routes and field geometry](../../kickoff/crescendo-field.json) · [Per-period outcomes, piece identities and events](match-series.json)

Coordinates use inches from the blue end wall and scoring-table border. Wing and center marks come from the 2024 layout. Nominal station markers reference the installed field CAD: tag pairs 1–2 and 9–10 for the SOURCES, tags 5–6 for the AMPs, tags 4 and 7 for lateral Speaker alignment, and tags 11–16 for the six TRAP faces. The foot-level interaction poses and 12-inch reach radius are tactical assumptions. Blue wing inner uses an 11-inch approach offset to keep the robot footprint clear of the STAGE post. The model uses six conservative post footprints from the installed field and omits detailed perimeter fixtures, cables and elevated geometry. The profile assigns nine NOTES per SOURCE before taking preloads and leaves processed pieces terminal. This is a finite practice supply. [Field layout](https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf) · [Field drawings](https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024FieldDrawings.pdf)

### Reproduce

With [Agent FRC 0.31.1](https://github.com/ehartye/agent-frc/tree/266805cd793fa5be4dbc68700e7b275bf782a60e), run the following from this project. The HTML builder writes to an unused filename. The JSON contains the SHA-256 of the exact project profile, the 0.05 s simulation step, both period cutoffs and all three final rosters.

```powershell
node "$AgentFrc/scripts/rehearsal-series.mjs" kickoff/crescendo-field.json $Results
node "$AgentFrc/scripts/kickoff.mjs" --profile kickoff/crescendo-field.json --output $NewStandaloneHtml
```

The [contact simulation](mission.md) tracks the real geometry of an individual modeled NOTE through the scoring sensors. Its outcomes answer a separate engineering question.

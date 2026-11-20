# Three six-robot CRESCENDO rehearsals

Open the [field lab](https://ehartye.github.io/frc-2024-poc/field-lab.html#rehearsal) and choose **Published scenarios**, then **Play match series at 10×**. To repeat your own setup three times, select **My roster · 3 matches**. **Reset** restores the robots, period behaviors and obstacle setup you edited before playback. For offline use, open the committed [standalone page](../../docs/field-lab.html) in a browser. The three matches run for 45 seconds of playback, with 15 seconds of AUTO and 135 seconds of TELEOP on each game clock. The robot cards stay under their own alliance when the user alternates red and blue additions.

All scores below are blue / red. The control run removes every endgame assignment and replays the same stock, programs and starting poses.

| Match | Control NOTE points | NOTE points with endgame | STAGE points | Match total | Modeled ENSEMBLE RP |
| --- | --- | --- | --- | --- | --- |
| Wing opening | 17 / 19 | 17 / 19 | 0 / 3 | 17 / 22 | 0 / 0 |
| Shared center | 16 / 16 | 16 / 16 | 10 / 3 | 26 / 19 | 1 / 0 |
| Miss and stage | 26 / 18 | 27 / 18 | 0 / 1 | 27 / 19 | 0 / 0 |

In Wing opening, Blue 1 arrives at 112.30 s TELEOP and finishes service at 122.30 s. Both its passive support and PARK observations are unknown. Red 2 reaches its own chain and meets its scenario criteria. In Shared center, Blue 1 and 2 give up the final 24 s of their ordinary roles. They reach separate seats on one chain at 112.45 and 111.65 s. Each uses 10 s to align, capture and lift after arrival. Base scores of 6, SPOTLIGHT of 2 and HARMONY of 2 yield 10 STAGE points. Two ONSTAGE robots also satisfy ENSEMBLE. Red 3's parking route is blocked. In Miss and stage, Blue 1 arrives with 4.10 s left. Red 2 loses support and has an independent bumper-in-zone observation for 1 PARK point. The traffic change also lets Blue earn one additional ordinary point.

The rosters prescribe contact, microphone and disabled-retention observations. Completing service alone earns no points. The engine stops powered motion at 150 s. The modeled assessment times are 155, 152.5 and 155 s respectively. The lab represents passive settling through terminal observations. Compare the mechanical study before projecting these results onto hardware. [Match ledger](match-series.json) · [Qualification ranking calculation](../stage-climb/rankings.py) · [Robot study](../stage-climb/study.md) · [Six-robot assignments](../../kickoff/alliance-profiles.md)

Coordinates use inches from the blue end wall and scoring-table border. Wing and center marks come from the 2024 layout. Nominal station markers reference the installed field CAD: tag pairs 1–2 and 9–10 for the SOURCES, tags 5–6 for the AMPs, tags 4 and 7 for lateral Speaker alignment, and tags 11–16 for the six TRAP faces. The foot-level interaction poses and 12-inch reach radius are tactical assumptions. Blue wing inner uses an 11-inch approach offset to keep the robot footprint clear of the STAGE post. The model uses six conservative post footprints from the installed field and omits detailed perimeter fixtures, cables and elevated geometry. The profile assigns nine NOTES per SOURCE before taking preloads and leaves processed pieces terminal. This is a finite practice supply. [Field layout](https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf) · [Field drawings](https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024FieldDrawings.pdf)

### Reproduce

With [Agent FRC 0.36.0](https://github.com/ehartye/agent-frc/tree/4154d301e28d36f0168f6df3f568fe57b549791f), run the following from this project. The HTML builder writes to an unused filename. The JSON contains the SHA-256 of the exact project profile, the 0.05 s simulation step, both period cutoffs and all three final rosters.

```powershell
node "$AgentFrc/scripts/rehearsal-series.mjs" kickoff/crescendo-field.json $Results
node "$AgentFrc/scripts/kickoff.mjs" --profile kickoff/crescendo-field.json --output $NewStandaloneHtml
```

The [contact simulation](mission.md) tracks the real geometry of an individual modeled NOTE through the scoring sensors. Its outcomes answer a separate engineering question.

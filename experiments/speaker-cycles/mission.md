# The six-floor-NOTE experiment

Start a blue robot empty, with six individually identified NOTES and a 120-second clock. Use three blue wing marks and three center-line marks. The field model owns each piece after release, including observation at the Speaker and clearance for the next arrival. [Field registration](../../sim/pressure/wing_field_frame.py) · [Speaker counter](../../sim/pressure/SPEAKER_COUNTER.md)

| Supply | Nominal field X (in) | Nominal field Y (in) |
| --- | ---: | --- |
| Inner, middle and outer wing | 114 | 161.625, 218.625, 275.625 |
| Three center-line marks | 325.625 | 161.625, 227.625, 293.625 |

The origin in plan lies at the blue station end wall and scoring-table-side field border. Positive X runs toward red and positive Y toward the AMP side. The practice marks define the available supply. The controller and route determine the order of acquisition. [Field layout sheets 3–4](https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf) · [Season rules](../../kickoff/season-rules.md)

A development attempt should continue to six counter events, the 120-second deadline, or the first observed failure that prevents continuation. Preserve the location, NOTE identity, elapsed time, controller settings and first failure. Count one result per predeclared start after the controller is frozen. The complete protocol includes the 15 setup dimensions and the repeatability threshold. [Protocol](../../sim/pressure/REPEATABILITY.md) · [Twenty evaluation starts](../../sim/pressure/evidence/speaker-repeatability-starts.csv)

| Run | Last demonstrated outcome | Evidence |
| --- | --- | --- |
| R7 nominal, corrected D1 | Middle and outer NOTES reached the shared counter at 13.639 and 28.064 s. Both robot departures and released NOTES passed their swept screens through the second field segment. | [Two-NOTE record](../../sim/pressure/evidence/r7-straight-first-two.json) · [Seek the first pickup and score](https://ehartye.github.io/frc-2024-poc/?view=r7-budget#model) |
| R7 nominal, earlier turning approach | All six isolated replays reached the detector by 117.621 s. At 8.2115 s the registered front bumper intersected a subwoofer panel. | [Wall witness and six observations](../../sim/pressure/evidence/r7-route-controls-D1.json) |
| Six-wheel nominal | Sixth counter event at 118.7945 s | [Six-score record](../../sim/pressure/evidence/center-low-sixth-score.json) |
| Swerve D1, six releases | Five of six NOTES blocked both sensor pairs in separate field replays. The outer-wing NOTE missed. Sixth arrival at 116.625 s. | [Six-NOTE record with plant revision](../../sim/pressure/evidence/maxswerve-six-note-D1.json) |
| Swerve D1 and D2, first two pickups | First counter events at 12.915 s and 14.214 s. Both returned with a different NOTE at 23.686 s and 23.864 s. | [First shots](../../sim/pressure/evidence/maxswerve-development-shot.json) · [Second-piece carries](../../sim/pressure/evidence/maxswerve-second-wing-development.json) |

These are simulation results. Full six-score reliability for the swerve robot remains open.

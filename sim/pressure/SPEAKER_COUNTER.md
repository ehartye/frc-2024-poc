# First floor NOTE through the Speaker counter

The [drive-over wing cycle](WING_CYCLE.md) records the whole-field route check and the subsequent middle-wing experiment.

The inner blue wing NOTE from the [powered release](RAISED_FEED.md) reaches both modeled counter sensor pairs at **19.504 simulated seconds** in the refined field run. The whole NOTE clears the nominal opening by 17.193 s. The other runs repeat that same pickup. A subsequent whole-field check found that the front bumper intersects the blue Stage upright at 1.1345 s on this approach. The first verified full-field score uses the middle-wing route and occurs at 19.2235 s.

| Trial | First simultaneous full shadow on the mission clock |
| --- | ---: |
| Baseline field integration | 19.899 s |
| Finer angular contact sampling | 20.575 s |
| Finer angular sampling and time step | 19.504 s |
| Half conveyor speed | 20.381 s |
| Stopped conveyor | No event |
| Coarser spatial launcher release | No event |
| Intermediate launcher release | 20.054 s |

All seven runs passed the opening. The coarser launcher release rebounded toward the field. The stopped conveyor left the NOTE upstream of the sensors. Bounce sequence and counter time depend on release velocity, impact parameters and integration resolution. The trials establish a simulated scoring path. They provide no measured scoring probability. [Numerical comparison](evidence/speaker-counter-20261118.json)

## Field and score model

The initial NOTE position, quaternion, velocity and spin come from the final powered-launcher integration state. One rigid transform places the entire pickup history on the selected wing mark and preserves the original clock. The free body retains the launch model's mass and inertia. Every step checks nearby surfaces across all 1,143 blue Speaker leaf occurrences from FIRST's FE-2024 STEP. The private geometry packet combines 120 installed component meshes with another 1,023 installed occurrences sharing 92 local meshes. The load uses ring circumference quadrature and a compliant normal law with capped sliding friction. The two lower belt surfaces move rearward at an assumed 1.5 m/s in the nominal run. Other surfaces stay at their installed poses. No measured conveyor drive dynamics, foam impact calibration, airflow or field control firmware is included.

Opening passage bounds the full torus between integration states against the registered opening edges. For each opposed sensor pair, the observer searches for a horizontal slice where the NOTE fills the entire rectangle between the two case ends. A bound on distance within each small tile covers intermediate ray positions. The counter records its first instant of simultaneous coverage of both pairs by the same ring after opening passage. A held ring earns one event. An independent empty-field check found unobstructed center rays and 63 of 81 sampled parallel rays clear at each pair. Lens locations and the actual timing logic are unmeasured. The simultaneous full-shadow requirement is a conservative simulated observation under the assumed through-beam arrangement. [2024 Game Details §6.5.1](https://firstfrc.blob.core.windows.net/frc2024/Manual/Sections/2024GameManual-06GameDetails.pdf) · [FIRST field assembly](https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/CRESCENDO2024Field-STEP.zip) · [FIRST NOTE counter assessment](https://firstfrc.blob.core.windows.net/frc2024/Manual/NOTECounterAssessment.pdf)

## Reproduce

Use the existing MuJoCo 4.0.1 environment, NumPy, libigl 2.6.3 and its SciPy dependency. The private registered field packet contains `panels.json`, `named-panels.npz`, `extras.json`, `extras.npz` and `opening-and-counter.json`. Each run takes a complete attained release packet and creates a new output directory. The stopped-belt control uses the identical release with `--belt-speed 0`.

```powershell
python -B sim/pressure/run_speaker_field.py $Pickup $Field $NewRun --dt .0001 --nodes 192
python -B sim/pressure/speaker_score_observer.py $NewRun "$Field/opening-and-counter.json" "$NewRun/observation.json"
python -B sim/pressure/check_speaker_sightlines.py $Field
```

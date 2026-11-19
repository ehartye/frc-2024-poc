# Six-NOTE repeatability campaign

The 2024 mission starts empty. Each successful attempt collects six distinct floor NOTES and registers all six at the close-range Speaker on one 120-second simulation clock. The target is at least 19 successes in the 20 predeclared setups. A completed [nominal six-score run](WING_CYCLE.md) and its [films](https://ehartye.github.io/frc-2024-poc/) provide the baseline.

## Physical setup

The [evaluation roster](evidence/speaker-repeatability-starts.csv) varies 15 dimensions. These are field X and Y offsets for the starting robot and six parked pieces, plus robot heading. The design envelope is ±10 mm and ±1 degree. Each coordinate uses the center of each of 20 equal strata once, with independently shuffled order using seed 17802024. Rotation offsets use the nominal blue field registration at the middle mark, yaw 245°. These bounds are chosen sensitivity inputs. Their hardware distribution is unmeasured.

The robot always targets the nominal marks. A setup offset initializes the corresponding physical body. Controller and observer assumptions must be identical in every evaluation run. Record the controller revision before evaluation. Keep the nominal run and development trials separate from the denominator. A controller change starts a new 20-attempt evaluation.

## Development observations

Three separate [development setups](evidence/speaker-pickup-development-starts.csv) exercised the first pickup on the 3-D native-bound plant with the empty mechanism lowered. The controller sampled the difference between the two modeled intake belt loads at 100 Hz. It applied 0.12 differential drive demand per newton, capped at 0.14. Approach speed was limited to 0.16 m/s. The physical instrumentation needed to observe those forces is unverified.

| Development case | Load feedback | First bilateral contact | Outcome |
| --- | --- | ---: | --- |
| Off-center | Disabled | — | Reached the tread substrate at 1.695 s |
| Same off-center setup | Enabled | 1.930 s | Seated at 4.830 s |
| Mirrored setup | Enabled | 1.916 s | Seated at 4.815 s |
| Centered setup | Enabled | 1.894 s | Seated at 4.816 s |

The same off-center and mirrored starts also seated using the difference in the two intake motor speeds. The controller sampled whole rpm every 10 ms and applied 0.008 differential drive demand per rpm, capped at 0.14. Seating occurred at 4.8395 s and 4.8250 s respectively. Use `--steering encoder` for this comparison. Sensor latency, noise and calibration on a physical robot remain unmeasured.

The same off-center setup and approach speed isolated the feedback change. Seating requires bilateral load, position and relative-velocity limits, and a 150 ms dwell. The [recorded inputs and trace digests](evidence/speaker-repeatability-pickup-development.json) retain the rejected control. These pickup observations count zero six-score attempts.

To replay the off-center development case, use the private native Fusion archive identified by the study and an unused output path.

```powershell
python sim/pressure/varied_floor_start.py --archive $Archive --output $Output --roster sim/pressure/evidence/speaker-pickup-development-starts.csv --attempt D1 --steering load
```

Use `--steering none` for the paired comparison. The terminal position, velocity, rotor, tread, bus, energy and clock state can continue to `wing_note_delivery.py --acquisition` with the registered pickup template and field station. A score still requires the field sensor event, clearance evidence and uninterrupted piece identity.

The off-center pickup continued to the station at 10.809 s. A fine release solve at the baseline first-shot duties produced free flight at 11.12775 s. The field observer found no whole-opening passage or beam registration. A sampled NOTE surface intersected a registered rigid wheel shell by 0.7233 mm. This shot fails both checks. The 16,384-segment ring needed a larger solver constraint arena to finish the off-center contact step.

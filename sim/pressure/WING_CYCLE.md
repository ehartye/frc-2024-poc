# Drive-over wing cycle

The 2024 practice study starts empty, drives over a marked floor NOTE, brakes, and carries that same piece to a declared close-range station. Simulation time, actuator positions, drivetrain motion, rotor speeds, stored tread deformation and electrical state carry across the transitions. Each collected mark has a distinct identity. The return controller can reverse away from the Speaker before turning toward the next mark.

**One of six full-field scores is verified.** At 0.80 launcher duty and 0.22 intake feed duty the middle-wing NOTE passed the opening by 16.6239 s and covered both pairs of sensors at 19.2235 s. The native and field checks cover the same release. The chassis was 1.4774 m from the end wall at release. The robot collected the outer NOTE at 23.0167 s. [Verified score](evidence/middle-wing-score.json) and [rejected comparison](evidence/wing-cycle-study.json)

## What the experiments establish

| Experiment | Result |
| --- | --- |
| Earlier inner-wing route | The counter detected two distinct NOTES by 35.692 s. At 1.1345 s the front bumper backing intersected the installed blue Stage upright and its panel. This route contributes zero mission scores. |
| Middle-wing start, 245° | The ground trajectory and braking finish at 9.45 s. All 18,900 intervals clear installed field structure and the five waiting NOTES under the flat-carpet model. |
| Station at (1.475, 5.553075) m facing 180° | The retained NOTE reached the raised shooting pose at 16.1725 s. Native Fusion swept-ring checks cleared hard parts during delivery. The minimum bilateral contact load was 3.657 N per side. |
| Powered shot, 0.72 launcher and 0.30 intake duty | The NOTE left contact at 16.4747 s, passed the whole opening by 16.6283 s and covered both counter sensors at 19.7879 s. The robot seated the outer wing NOTE at 23.0117 s. The released NOTE cleared the moving robot by at least 17.30 mm on the independent flight and return trajectories. |
| Rigid wheel shell at that shot setting | Native Fusion left four shell bodies for inspection. An independent material-point distance test found 0.454 mm penetration into one shell. The nominal tread contact model permits this impossible path. Zero scores from this trial qualify toward six. |

A lower-resolution handoff hit the modeled tread backing before release. The finer solve produced a free release and exposed the separate rigid-shell interference. The model assumes motor current limits, local dissipation of generating power by the drive electronics, a rigid NOTE, modeled tread compliance, and specified conveyor speed. The figures describe simulation of one declared supply layout.

## Reproduction

Use the pinned native F3D, the ground and stop packet from the refined turning-lift study, the private native body inventory, and the complete 2024 field STEP. The geometry stays in the private study directory. Run the geometry export with Python and OCCT, and the simulation with Python, NumPy and MuJoCo. The field contact solver also needs libigl. Define the file and output directory variables below before running. Each output path must be new.

```powershell
python sim/pressure/export_field_bounds.py $FieldStep $FieldBounds
python sim/pressure/wing_note_delivery.py --pickup $Pickup --archive $Archive --first-mark middle --field-yaw-deg 245 --field-station 1.475 5.553075 180 --grid 512 --dt .0005 --horizon 18 --output $Carry
python sim/pressure/wing_note_release.py --pickup $Pickup --archive $Archive --delivery $Carry --first-mark middle --field-yaw-deg 245 --grid 1024 --segments 16384 --dt .00005 --duty .8 --pinch-duty .22 --output $Shot
python sim/pressure/run_speaker_field.py $Shot $Speaker $Flight --first-mark middle --field-yaw-deg 245 --nodes 192 --dt .0001 --horizon 5 --trace-every-step
python sim/pressure/speaker_score_observer.py $Flight "$Speaker/opening-and-counter.json" $Observation
python sim/pressure/second_wing_pickup.py --pickup $Pickup --archive $Archive --release $Shot --first-mark middle --field-yaw-deg 245 --next-mark outer --retreat .4 --grid 512 --dt .0005 --horizon 13 --output $Return
python sim/pressure/field_route_clearance.py --pickup $Pickup --first-delivery $Carry --release $Shot --next-phase $Return --inventory $Inventory --field $FieldBounds --first-mark middle --field-yaw-deg 245 --output $RouteClearance
python sim/pressure/released_note_clearance.py $Flight $Return $Inventory $FlightClearance --installed-field $FieldBounds
python sim/pressure/refine_shell_clearance.py $Shot $Inventory $NativeShotSweep $ShellRefinement
python sim/pressure/verify_wing_attempt.py --delivery $Carry --shot $Shot --flight $Flight --acquisition $Return --sensor "$Speaker/opening-and-counter.json" --route-clearance $RouteClearance --flight-clearance $FlightClearance --delivery-clearance $NativeDeliverySweep --shot-clearance $NativeShotSweep --shell-refinement $ShellRefinement --output $Verified
```

The route audit bounds the chassis, carriage, cradle and active floor piece between integration poses against placed field leaves and the dormant supply. For sloping subwoofer members it uses bounds around 25 mm slices of each installed CAD solid. An overlapping envelope calls for refinement or a solid intersection test. The field solver and observer establish Speaker passage and beam coverage. Native part clearance requires swept checks against the pinned robot assembly, including rigid material underneath compliant treads. Each of these checks addresses a different physical failure.

The named Fusion draft supplies the two native sweep receipts in the last commands. In the passing shot, conservative swept solids left 293 integration intervals requiring a narrower test. For each of the four original bored wheel shells the verifier uses its registered finite annulus, a spatial sampling bound and a continuous time bound. The smallest remaining separation was 0.087 mm. The released NOTE cleared the moving robot by at least 26.92 mm. The same method rejects the 0.72/0.30 counter example. The small native margin warrants a tolerance study before making a hardware claim.

[Powered transfer](RAISED_FEED.md) · [Speaker counter model](SPEAKER_COUNTER.md)

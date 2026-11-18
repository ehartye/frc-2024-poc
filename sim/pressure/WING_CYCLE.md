# Drive-over wing cycle

The 2024 practice study starts empty, drives over a marked floor NOTE, brakes, and carries that same piece to a declared close-range station. Simulation time, actuator positions, drivetrain motion, rotor speeds, stored tread deformation and electrical state carry across the transitions. Each collected mark has a distinct identity. The return controller can reverse away from the Speaker before turning toward the next mark.

**Two of six full-field scores are verified.** The middle-wing NOTE covered both sensor pairs at 19.2235 s. The robot collected the outer NOTE at 23.0167 s and scored it at 38.2709 s, with the chassis 1.4845 m from the end wall at release. It seated the inner NOTE at 45.9859 s. [First score](evidence/middle-wing-score.json) · [Second score and preceding trajectory](evidence/outer-wing-score.json)

For the second shot the controller targets 3,950 rpm, changes the loaded cradle pitch by −0.20° and raises the lift by 1.06 mm relative to the attained station state. It waits for three ready control samples before feeding at 0.22 duty. The whole NOTE cleared the opening at 35.5336 s. At least 25.68 mm separated the released NOTE from the retreating robot. The first NOTE cleared the counter by 22.7340 s and remained at least 1.00 m from the second during its five-second field flight, assuming a passive collection floor behind the end wall.

The native wheel shells and one intake belt needed finer clearance checks. The belt calculation uses the filled exterior of its native extrusion. Fusion found zero belt volume outside this envelope. The conservative minimum across all refined intervals was 0.00092 mm. Nine sampled lift and cradle poses showed no new structural intersections. Actual material properties, field contacts and manufacturing clearances remain unmeasured.

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

To reproduce the second shot, continue the outer pickup with a 0.65 m final straight at the same shooting station. Use `--target-rpm 3950 --pitch-trim-deg -.20 --height-trim-mm 1.06` on the raised-feed runner. Approach the inner NOTE at field heading 0° with a 0.32 m capture standoff after backing 0.4 m from the Speaker. Refine the native release with `--samples 98304 --subdivisions 32`. Run `counter_reset.py` with the first score, its consecutive field segments, the second flight and the registered sensor geometry. Pass that receipt, the first score and the native outer pickup sweep to `verify_wing_attempt.py` using `--counter-reset`, `--previous`, `--entry-acquisition` and `--pickup-clearance`.

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

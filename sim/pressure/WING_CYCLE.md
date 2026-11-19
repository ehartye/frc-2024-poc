# Drive-over wing cycle

The 2024 practice study starts empty, drives over a marked floor NOTE, brakes, and carries that same piece to a declared close-range station. Simulation time, actuator positions, drivetrain motion, rotor speeds, stored tread deformation and electrical state carry across the transitions. Each collected mark has a distinct identity. The return controller can reverse away from the Speaker before turning toward the next mark.

**All six floor NOTES scored in 118.7945 simulated seconds**, leaving 1.2055 s on the 120 s clock. The robot started empty and collected a distinct piece from each of the six floor marks.

| Floor mark | Speaker counter (s) | Receipt |
| --- | ---: | --- |
| Blue wing, middle | 19.22350 | [First score](evidence/middle-wing-score.json) |
| Blue wing, outer | 38.27090 | [Second score](evidence/outer-wing-score.json) |
| Blue wing, inner | 60.78015 | [Third score](evidence/inner-wing-third-to-center-high.json) |
| Center line, high | 81.16660 | [Fourth score](evidence/center-high-fourth-to-center-mid.json) |
| Center line, middle | 100.50735 | [Fifth score](evidence/center-mid-fifth-to-center-low.json) |
| Center line, low | 118.79450 | [Sixth score](evidence/center-low-sixth-score.json) |

The robot seated the sixth NOTE at 106.29905 s, reached the shooting station at 115.64755 s and released at 116.14800 s. The whole NOTE cleared the opening at 116.29820 s. All five earlier NOTES cleared the counter and stayed at least 0.923 m from the final NOTE through 118.808 s. The released NOTE stayed at least 14.824 mm from the unloaded chassis. The native sweep and refinement cleared all 263 flagged launcher intervals with a minimum nominal separation of 0.0142 mm. Eight attained mechanism poses showed no new structural intersections. The model uses assumed field-contact properties and nominal CAD dimensions. Manufacturing and hardware margins remain unmeasured.

An earlier route verified five scores by 101.9423 s. Its last pickup completed at 107.7574 s. [Earlier fifth score](evidence/center-mid-fifth-score.json)

The center-line route carries forward the three wing scores. The inner-wing receipt above binds the next pickup on this route. The [earlier fourth-score route](evidence/center-high-fourth-score.json) reached 84.1170 s. Receipt hashes identify the private runner outputs, which used CRLF line endings. The repository stores their JSON text with LF line endings.

On the fourth shot, a 0.65 s move adjusted pitch by −0.279° and lift height by +1.238 mm. With a 3,950 rpm target and 0.23 feed duty, the ring cleared all 307 flagged native intervals by at least 0.0511 mm. Both counter paths were blocked after the full opening crossing at 81.6889 s. A control run at 0.22 feed duty also triggered the counter but overlapped a rigid wheel shell by 0.106 mm. The full replay at 0.23 duty reproduced the entire trajectory byte for byte.

For the second shot the controller targets 3,950 rpm, changes the loaded cradle pitch by −0.20° and raises the lift by 1.06 mm relative to the attained station state. It waits for three ready control samples before feeding at 0.22 duty. The whole NOTE cleared the opening at 35.5336 s. At least 25.68 mm separated the released NOTE from the retreating robot. The first NOTE cleared the counter by 22.7340 s and remained at least 1.00 m from the second during its five-second field flight, assuming a passive collection floor behind the end wall.

The native wheel shells and one intake belt needed finer clearance checks. The belt calculation uses the filled exterior of its native extrusion. Fusion found zero belt volume outside this envelope. The conservative minimum across all refined intervals was 0.00092 mm. Nine sampled lift and cradle poses showed no new structural intersections. Actual material properties, field contacts and manufacturing clearances remain unmeasured.

The inner NOTE arrived tilted about 1.19° from the nominal feed plane. An abrupt adjustment let it roll backward out of the intake. Over 0.65 s the robot instead kept its travel pinch controller engaged while moving the pitch reference by +1.12° and lowering the lift 5 mm. It released the NOTE at 56.81395 s with its chassis 1.4210 m from the end wall. All 310 native wheel intervals cleared by at least 0.06504 mm. Six sampled structural poses had no new intersections. The field observer verified whole-ring passage at 56.95935 s, both optical paths at 60.78015 s and at least 28.54 mm of separation from the returning robot.

The middle and outer NOTES touched in the collection area at 42.7313 s, after clearing both sensors. A shared simulation starts from their recorded field states at 40.3874 s. The contact applies equal and opposite forces at each common contact point. Both paths reproduce the independent positions and velocities through 42.61 s. The assumed field-contact coefficients also govern ring-to-ring contact, with peak modeled indentation of 3.16 mm. Through the third shot's field simulation the two older NOTES each stayed at least 0.969 m from the newly released NOTE.

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

For the inner shot, continue the inner pickup with a 1.45 m/s loaded travel reference to (1.415, 5.553075) m facing 180°, using a 0.65 m final straight. Keep the outer shot's speed and duty settings and use `--align-seconds .65 --pitch-trim-deg 1.12 --height-trim-mm -5`. Refine the release with 65,536 ring samples, 32 temporal subdivisions and `--refine-below-mm .065`. To collect the high center-line NOTE, use `--retreat .4 --via 2.75 6.80 --via 6.85 7.455 --transit-speed 2.3 --next-mark center-high`. Each coordinate is in the registered blue field frame.

To reproduce the shared collection run, terminate the first NOTE's passive-floor segment at the end of the outer NOTE's five-second field flight. Supply those two dense segment folders to `coupled_collection.py --inputs` with the registered Speaker assembly, a new output directory and `--duration 26`. For each older NOTE, supply its independent prefix and individual folder from the shared run to `counter_reset.py`. On the outer NOTE check, provide the middle score and its same segment list via `--earlier-collection`. Pass both reset receipts to the third-score verifier.

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

## Center-line travel and the final release

The [center-line control settings](evidence/centerline-route-controls.json) give the pickup waypoints, loaded approach, launcher commands and integration settings for each of the three remaining floor NOTES after the wing sequence. Field waypoints are in metres. The shooting station contains field x, field y and heading in degrees. Each field prefix ends on a recorded integration row after the counter event. Continue the earlier pieces in the shared collection solve from their recorded positions and velocities.

For a loaded return, `wing_note_delivery.py --brake-into-station` applies a stopping-distance speed limit over the last 2 m of a lookahead path. The command uses 1.3 m/s² for braking, caps the speed at 1.90 m/s and tapers the speed near the station. The existing heading, speed, mechanism pose, bilateral load and 0.15 s settling checks determine when the robot can hand off to the launcher. On the final center-low return the controller reached the station at 115.64755 s, 0.4155 s earlier than the default controller on the same pickup and waypoints.

The pickup and loaded-delivery runners accept a lookahead distance in metres for following connected field waypoints. Use the recorded robot poses to check bumper and stage clearance through each turn. During pickup, the capture speed controls the chassis closure rate and the intake speed reference after bilateral contact. During delivery, the lifting drive acceleration limits forward acceleration until the cradle reaches 45°. Compare the attained ring position relative to the chassis when checking clearance to the fixed drive motors.

To add a scored NOTE to the shared collection area, run `coupled_collection.py --until` with the other flight's recorded endpoint. The solver physically integrates the remaining fraction of a step and records its duration. Pass every earlier participant into the subsequent shared solve. The counter verification checks where independent paths remain separate and where their contact forces came from the same solve.

After the sixth release, use `post_release_hold.py` with the release, original pickup, native archive and field frame. Its dynamics model carries the chassis, lift, cradle and four spinning wheels. The independently simulated NOTE continues from the same release state. Cover the entire field flight with the hold, include it as the route audit's next phase, and run the moving robot separation check against it. Pass the resulting hold to `verify_wing_attempt.py --final-hold`. The verifier requires the six distinct marked pieces in score order, a cleared counter for each earlier piece, and a sixth counter event within 120 s.

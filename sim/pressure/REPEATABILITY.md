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

## Reproducing the first score

Use Python 3.13.12 with the [project dynamics lock](../mujoco/requirements-lock.txt) for the baseline comparison. The archived first delivery ends at 16.1725 s. On MuJoCo 4.0.1 and NumPy 2.5.5, the launcher reproduced all 6,144 rows of the archived release trace byte for byte. The field observer again saw the whole NOTE through the opening at 16.6239 s and both beams covered at 19.2235 s. The original native sweep and its interval refinement give a minimum rigid-wheel clearance of 0.08676 mm for this trace.

An alternate environment with MuJoCo 3.14.0 and NumPy 2.5.3 repeated its own trace byte for byte. Starting with the same delivery, controller settings and 50 µs timestep, the two environments agreed on NOTE center position within 0.000001 mm through first wheel contact. Their positions differed by 0.1 mm only 3.45 ms later. Peak wheel load reached 5.44 kN in the alternate environment against 1.83 kN on the locked stack. The alternate shot intersected a shell by 0.63 mm and missed the opening. Both package versions changed between these runs.

To compare two releases, run the following from the checkout with the two private output directories. The comparison verifies the trace digests and common starting state, matches rows at the same simulation time, and reports the first separation and both peak loads.

```powershell
python sim/pressure/compare_shot_replays.py $ArchivedRelease $ReplayedRelease
```

Use `wing_note_release.py --dt .000025` to halve the 50 µs integration step. At 16,384 segments, `--ring-phase-deg .010986328125` shifts only the discretization by half a segment. Each check starts with the original physical NOTE pose. The new pickup, delivery, release, field flight and robot return receipts record Python, MuJoCo and NumPy versions. The official roster verifier checks those stages against the dynamics lock and carries the runtime into the next score receipt. The field query used libigl 2.6.3 and SciPy 1.18.1.

## First off-center score

On the locked stack, the untrimmed first shots from both mirrored development starts reached both Speaker sensors. Both paths intersected a rigid wheel shell, by 0.124 mm on D1 and 0.087 mm on D2. Both received zero credit.

The next controller used the same settings on both starts. It moved the loaded cradle by −0.117° and the lift by +0.84 mm over 0.30 s, held a 3,950 rpm wheel reference and fed at 0.23 pinch duty. The robot aimed at the nominal floor marks and the same shooting station. Each flight and robot return began with the actual release of that case.

| Development result | D1 | Mirrored D2 |
| --- | ---: | ---: |
| First NOTE seated using encoder steering | 4.8395 s | 4.8250 s |
| Minimum clearance in refined pickup intervals | 0.0153 mm | 0.0104 mm |
| Minimum clearance in refined release intervals | 0.0671 mm | 0.0222 mm |
| Whole NOTE through Speaker opening | 11.6235 s | 11.6350 s |
| Simultaneous counter beams | 15.1758 s | No event |
| Unresolved flight intervals against the moving robot | 0 | 1,823 |
| Next outer NOTE seated | 18.0319 s | 18.0453 s |
| Verified first scores | **1** | **0** |

Fusion swept the actual trajectories against the registered assembly. The interval refinement bounded the remaining rigid shell and belt candidates. The smallest certified gap between the discharged D1 NOTE and returning robot was 21.35 mm. The D2 NOTE passed the opening, encountered field hardware and returned to the field side. An unresolved robot interval is an open clearance question. D1 earns one development score. All twenty six-score evaluation slots remain pending. [Runtime, scorer and native clearance evidence](evidence/speaker-first-release-runtime.json)

## Differential-drive baseline for the swerve conversion

Moving the common station 7.5 mm closer to the wall gave a verified first score on both mirrored starts. For each trial the robot drove to field position (1.4675, 5.553075) m at 180° and performed the same loaded alignment. Each case carried its own attained pickup, delivery, release, field flight and robot return through verification.

| Six-wheel differential-drive measurement | D1 | Mirrored D2 |
| --- | ---: | ---: |
| First counter event | 14.9905 s | 14.88765 s |
| Second floor NOTE seated | 18.0424 s | 18.05595 s |
| Minimum clearance in refined release intervals | 0.0692 mm | 0.0805 mm |
| Minimum discharged NOTE to moving robot clearance | 21.39 mm | 22.89 mm |

The 2024 robot is converting to four independently steered modules. Compare pickup and score times at the same floor placements with the same sensor, native-clearance and mission-clock checks. The six-score campaign will use the integrated swerve robot. [Verified differential-drive comparison records](evidence/speaker-first-release-runtime.json)

### Four-module pickup and carry

Both mirrored development starts use the same 520 mm square module layout and the same four independently steered wheels as the articulated Fusion revision. Each run starts empty. The simulator integrates the free NOTE and all eight module coordinates alongside the original carriage, pitch, belts and launcher wheels. Once the NOTE loads a belt, the robot can move sideways to balance the two simulated belt loads. The controller uses ideal simulated load and pose observations. The same plant continues from seated pickup to the first close-range station at field XY (1.475, 5.553075) m and heading 180°.

| Simulated measurement | D1 | D2 |
| --- | ---: | ---: |
| Six-wheel baseline, NOTE seated (s) | 4.8300 | 4.8150 |
| Swerve, NOTE seated (s) | 4.4945 | 4.6545 |
| Swerve, NOTE retained at shooting pose (s) | 9.8945 | 10.0545 |
| Simultaneous strafe, turn and lift or pitch (s) | 2.6535 | 2.6360 |
| Minimum bus voltage (V) | 11.365 | 11.364 |

The pickup improvement was 0.336 s on D1 and 0.161 s on D2 against the six-wheel tactile controller. Halving the integration step to 0.25 ms changed D1 station arrival by 0.03125 s. The steering motors reached their 20 A limit. The minimum modeled load on either belt during the two nominal carries exceeded 3.58 N. Scoring still requires a separate launch, field passage and native clearance check. [Development receipts](evidence/maxswerve-development-carry.json)

The drive model uses the 12-tooth pinion and 5.5 drive reduction. Its inferred carrier coupling gives drive shaft rate equal to 5.5 times wheel rate plus steer rate. The same transmission maps torque and reflected inertia. The electrical model uses the published motor endpoints, bounds motoring torque by the back-EMF constant, and dissipates generated terminal power locally. Each tire applies one friction-limited force at the carpet. The planar chassis assumes equal tire normal loads, friction coefficient 0.9 and 0.1 m/s slip regularization. Rotating masses and inertias are estimates within the original robot mass and yaw inertia. Hardware traction, sensor response and the carrier coupling sign need measurement. [WPILib motor data](https://github.com/wpilibsuite/allwpilib/blob/v2024.3.2/wpimath/src/main/java/edu/wpi/first/math/system/plant/DCMotor.java) · [REV gearing and current settings](https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/e87ac7aa00edc12efecf49089a904388aa896a84/src/main/java/frc/robot/Constants.java)

Run with the pinned simulator and the private articulated F3D. Each output argument points to a new directory.

```powershell
python -B sim/pressure/varied_floor_start.py --archive $ArticulatedRobot --drivetrain maxswerve --roster sim/pressure/evidence/speaker-pickup-development-starts.csv --attempt D1 --output $Pickup --carry-output $Carry
python -B sim/pressure/verify_swerve_carry.py $Pickup $Carry
```

### First Speaker shot with the four-module drivetrain

The release runner restores the carry endpoint on its original clock, including chassis and module positions and velocities, the free NOTE, external rotors, belt shear, bus voltage and accumulated source energy. All eight module motors share the battery while holding the shooting station. The launcher uses the six-wheel control settings for both starts. The same registered field solver follows the free NOTE into the Speaker and checks both opposing counter sensor pairs.

| Mirrored start | Arrival | Free release | Whole NOTE through opening | Both sensors | Six-wheel baseline |
| --- | ---: | ---: | ---: | ---: | ---: |
| D1 | 9.895 s | 10.566 s | 10.705 s | 12.915 s | 14.991 s |
| D2 | 10.055 s | 10.726 s | 10.864 s | 14.214 s | 14.888 s |

These development results use 16,384 launcher contact samples at 50 μs. With 8,192 samples, D1 reached the unsupported tread-depth limit. On D2 the ring passed through the opening and bounced back into the field without triggering either counter. Rotating the 16,384 samples by half a segment also reached the tread-depth limit on D1. The contact calculation needs to converge under refinement, and a native rigid-clearance check is still due on these trajectories. The twenty six-NOTE evaluation slots remain open. [Paired observations and controls](evidence/maxswerve-development-shot.json)

Pass the original swerve floor-pickup run as `--pickup` and the matching loaded carry as `--delivery`. Use the installed articulated robot archive that produced the input trajectories. The field command needs the local field meshes and the registered sensor file.

```powershell
python -B sim/pressure/wing_note_release.py --pickup $Pickup --archive $ArticulatedRobot --delivery $Carry --output $Shot --dt .00005 --grid 512 --segments 16384 --duty .8 --pinch-duty .23 --target-rpm 3950 --pitch-trim-deg -.117 --height-trim-mm .84 --align-seconds .3 --first-mark middle --field-yaw-deg 245
python -B sim/pressure/run_speaker_field.py $Shot $Field $Flight --dt .0001 --nodes 192 --first-mark middle --field-yaw-deg 245 --trace-every-step --horizon 5
python -B sim/pressure/swerve_shot_observation.py $Pickup $Carry $Shot $Flight $Sensor
```

## Second floor NOTE and shot timestep check

From the 50 µs first release, the swerve chassis drove to the outer wing while lowering the mechanism. It turned toward the nominal mark, staged 0.85 m short, then used the intake loads to correct its final approach. Both development starts used the same outer NOTE position. The robot returned to the original shooting pose carrying this second, distinct NOTE.

| Simulated time (s) | D1 | D2 |
| --- | ---: | ---: |
| First release starting the return | 10.5661 | 10.7259 |
| Outer NOTE seated | 17.3031 | 17.4724 |
| Outer NOTE at shooting pose | 23.6855 | 23.8643 |
| First NOTE paired-sensor event with a 25 µs shot step | 14.0969 | 13.7570 |

The last row is a separate refinement of the **first** shot. The field step stayed at 100 µs. The two sensor times moved by +1.182 s and −0.457 s relative to the 50 µs shot runs. The second NOTE has no release or counter event in these runs. The formal six-NOTE campaign remains at zero starts. [Development measurements and trace digests](evidence/maxswerve-second-wing-development.json)

Use the first pickup and release from the same attempt, its original archive and the development roster. The return verifier checks the robot, rotor, shear, battery and source-energy handoff, the clock, the distinct floor NOTE and its resting placement. Continue the field simulation of the first released NOTE independently on that clock.

```powershell
python -B sim/pressure/swerve_next_note.py --pickup $Pickup --release $Shot --archive $ArticulatedRobot --roster $Roster --next-mark outer --output $OuterPickup --carry-output $OuterCarry
python -B sim/pressure/verify_swerve_carry.py $OuterPickup $OuterCarry --release $Shot --roster $Roster
```

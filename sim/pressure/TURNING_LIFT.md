# Floor pickup through a turning lift

The driver can start turning toward the next scoring location while the mechanism raises the NOTE. The simulation covers a drive-over pickup, 225.56 mm of floor transport, an active stop, acceleration into an arc, and the full pitch, lift and hold while following that arc. The NOTE is free to slip, roll or fall. This run uses differential wheel commands with feedback and the native relieved side-pinch mechanism.

## Verified maneuver

The floor NOTE indexed at 4.45 s. The robot transported it 225.56 mm and braked at 9.45 s. It then accelerated into the requested path. The table covers the 5.8 seconds from the first lift command through the end of the 0.8-second hold. In each trial the NOTE had positive pressure on both sides and stayed clear of the floor after lift-off.

| Maneuver | Heading change | Distance driven | Net lift | Minimum forward speed | Minimum gap in hold |
| --- | ---: | ---: | ---: | ---: | ---: |
| Left | 72.61° | 1.845 m | 449.43 mm | 0.3176 m/s | 317.06 mm |
| Right | -72.61° | 1.845 m | 449.43 mm | 0.3176 m/s | 317.06 mm |
| Straight | 0.00° | 1.846 m | 449.43 mm | 0.3177 m/s | 317.01 mm |

The independent one-command run reproduced every recorded floor and stop state and the byte-identical left-turn lift trajectory. The two turning cases cleared the full 60-degree handoff with the robot still translating and steering. [Physical reproduction](evidence/turning-lift-20261118/refined-end-to-end.json) · [Measured maneuvers](evidence/turning-lift-20261118/refined-study.json)

At the selected 0.25 ms timestep, the 1024- and 2048-square contact grids differed by at most 0.0149 mm in translational position and 0.199 mm/s in linear velocity over all shared samples. Both meet the declared path bounds. Agent FRC separately compared final height, minimum hold clearance and heading change. Fusion cleared every interpolated NOTE interval on the tested refinements. The nominal fine left turn had 73 sampled mechanism poses with zero new or increased structural overlaps. [Agent FRC measurements](evidence/turning-lift-20261118/agent-frc-refined-result.json) · [Native coverage](evidence/turning-lift-20261118/refined-native-coverage.json)

## Run the maneuver

Use Python with NumPy and MuJoCo, the private relieved-tread Fusion archive, and a new output directory. The runner checks the archive digest against the native pressure registration before starting. The reference run used Python 3.13.12, NumPy 2.5.5 and MuJoCo 4.0.1.

```powershell
$env:OPENBLAS_NUM_THREADS = '1'
python -B sim/pressure/run_floor_pickup.py --archive $archive --output "$packet/left" --turn left
python -B sim/pressure/verify_turning_lift.py "$packet/left/lift"
```

Use `--turn right` to mirror the drive command and `--turn straight` for the translating control. Omitting the option runs the existing braked experiment. Each invocation recomputes the ground approach and active stop, then writes the per-step lift trajectory and an independent measurement in `retention.json`. The default turning lift uses a 0.25 ms timestep and a 1024-square contact grid. To refine just the lift, use the ground and stop receipts from that run.

```powershell
python -B sim/pressure/run_turning_lift.py --archive $archive --ground "$packet/left/ground/receipt.json" --stop "$packet/left/stop/receipt.json" --case left --dt .000125 --grid 1024 --output "$packet/left-finer"
```

## What moves

Six tire patches produce the wheel forces and turning moments. The 45 kg robot includes the moving 4 kg carriage and 3 kg cradle. The wheel and pinch rotors and reflected lift and pitch inertia draw from a common 12 V supply with 0.02 ohm internal resistance. The finite-force controller runs at 100 Hz. After 1.5 seconds of drive lead-in it centers the NOTE, pitches toward 60 degrees, raises the pivot to 474.83 mm, then holds for 0.8 seconds. It continues driving throughout.

The free NOTE and cradle exchange full three-dimensional forces and moments. Four finite elastic shear regions divide the two treads fore and aft so the grip can transmit a roll moment. Each region uses 1250 N/m, preserving the earlier symmetric stiffness. The solver checks the friction cone, reciprocal work and electrical power at each integration stage. Infeasible contact or motor states end the run. The nominal handover releases coordinates that were flat during ground transport. Mechanical energy agrees across that handover to about 1e-15 J. [Handover measurements](evidence/turning-lift-20261118/junction.json)

## What the evidence covers

The overlap test uses net attained lift, signed yaw rate and forward velocity on a common physical clock. Every sample from the first lift command to the end of the hold must meet the speed and steering gates. The verifier reconstructs the ring orientation and lowest point, checks continued bilateral pressure, and checks chassis velocities against successive poses. It also checks for renewed floor contact after the whole ring becomes airborne. [Acceptance contract](evidence/turning-lift-20261118/contract.json)

At the upper hold, a 100 ms replay with the recorded motor commands changed the supported NOTE height by 0.017 mm. Removing the tread forces and moments from the same state produced a 49.01 mm drop that matched free fall. Run `verify_turning_support.py` with the lift directory and stop receipt to reproduce both cases. [Support control](evidence/turning-lift-20261118/refined-support-control.json)

For native clearance, the verifier expresses the measured torus in the chassis, carriage and cradle frames. It bounds translation and rotation between integration samples, then asks Fusion to intersect a conservative torus envelope with rigid solids in each frame. Intervals that need tighter bounds are subdivided. A separate sampled test compares cross-group mechanism interference with the floor-pose baseline. The native archive, component geometry, raw Fusion outputs and colored replay are private.

The chassis is planar and uses fixed tire normal loads. Grip stiffness, tire scrub, floor contact, actuator parameters and moving inertias are modeling hypotheses. The result describes possession during the specified low-speed maneuver. Driving limits at higher lateral acceleration, material calibration, overhead loading, handoff to another mechanism and scoring remain open.

## Selecting the integration timestep

The original 0.5 ms runs retained the NOTE. Agent FRC compared final center height, minimum hold clearance and heading change. All three measures met their finest-pair tolerances. Yet the 1024- and 2048-square grids at this timestep differed by up to 0.299 mm and 4.84 mm/s along the path, exceeding the declared 0.05 mm and 1 mm/s bounds. This distinction matters when a maneuver passes close to the bumper. At 1024-square resolution the 0.25 and 0.125 ms runs agreed within 0.00018 mm and 0.070 mm/s throughout. [Whole-path comparison](evidence/turning-lift-20261118/study.json) · [Agent FRC comparison](evidence/turning-lift-20261118/agent-frc-result.json)

One extra braked control at 512-square resolution encountered a tied signed-distance gradient before completing the maneuver. The verifier retained that stop reason. The separate planar braked-lift experiment supplies the completed stationary baseline. [Rejected control](evidence/turning-lift-20261118/rejected-braked-control.json) · [Braked baseline](RETAINED_FLOOR_PICKUP.md)

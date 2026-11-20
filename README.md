# 2024 CRESCENDO: component-based robot proof of concept

<!-- cad-still -->
[![1780 MAXSwerve · R9 physical assembly](docs/media/r9-assembly.png)](https://ehartye.github.io/frc-2024-poc/?view=r9-assembly#model)

**1780 MAXSwerve · R9 physical assembly.** Parked robot with its inboard lift drive, frame, removable bumper and electrical supports. Purchased mechanisms use dimensioned external models. This view records mechanical fit in CAD. [Native model record](docs/media/r9-assembly.json) · [Inspect the parked assembly](https://ehartye.github.io/frc-2024-poc/?view=r9-assembly#model)
<!-- /cad-still -->

[Download the editable R9 robot](cad/student/r9-physical-assembly.f3d) · [Assembly sequence and fit results](cad/assembly-readiness/physical-assembly.md) · [Shop drawings](cad/assembly-readiness/drawings/r9-shop-drawings.pdf)

[Blue 3 role comparison, service availability and whole-robot support review](experiments/alliance-service/study.md)

[R13 ground-supported frame and four rail service checks](cad/service-readiness/ground-rails/study.md)

[R11 four-controller bench plate, disconnected battery gauge and mast spacer](cad/service-readiness/r11-service.md)

[R10 battery and unloaded mast service fixture](cad/service-readiness/passive-service.md) · [Download the editable service fixture](cad/student/r10-passive-service.f3d)

[Explore the R8 Stage climb and missed-lock control](https://ehartye.github.io/frc-2024-poc/?view=r8-stage#model) · [Climber design, load transfer and clearance](experiments/stage-climb/study.md) · [Both-alliance STAGE and qualification results](experiments/speaker-cycles/match-rehearsal.md)

The nominal R7 collected six distinct floor NOTES and reached the shared counter six times by **117.756 s** on development start D1. All twenty formal evaluation starts remain available.

[Download the student R7 Fusion assembly](https://raw.githubusercontent.com/ehartye/frc-2024-poc/main/cad/student/r7-maxswerve.f3d) · [Fusion joints and native handoff](cad/native-handoff.md) · [Play the R7 nominal-mass pickup and shot](https://ehartye.github.io/frc-2024-poc/?view=r7-budget#model) · [Six-NOTE mission results](experiments/speaker-cycles/mission.md)

[Follow the 2024 Agent FRC workflow](docs/workflow.md) from the season rules and alliance roles to the native assembly, scoring trials and published replay. [Open the field lab and play three matches at 10×](https://ehartye.github.io/frc-2024-poc/field-lab.html#rehearsal). [Replay the historical A2 and pressure campaigns](experiments/a2-campaign/CAMPAIGN.md).

The [historical case input manifest](experiments/a2-campaign/historical-fixtures/agent-frc/manifest.json) identifies source-pinned simulation parameters, original geometry and prior analysis from the earlier plugin case. Inputs labeled as excerpts have a new schema and retain their source hash. The manifest records identical existing files and the original revision of the shared producers.

**Strategy-first editable CAD, with retained simulation experiments.** The current component assembly is a design candidate. The runnable legacy A2 cycle below uses earlier geometry and does not validate this newer robot.

## Experimental checkpoint: choose the entry point

This repository is a **reviewed research checkpoint, not a completed robot simulator or fabrication release**. The implementations below are alternatives and historical experiments, not one coupled mission and not a commitment to maintain three simulators. Synthesis evaluation remains open; no installation or compatibility result is implied.

| Entry point | What it currently demonstrates | Not accepted |
| --- | --- | --- |
| [Maple 2024 arena](sim/maple/README.md) | Headless force-driven differential robot, actual upstream wall/stage collisions and a shared ground NOTE | Intake collection, corrected season scoring, airborne obstacle collision, driver UI or full robot controls |
| [MuJoCo drivetrain](sim/mujoco/README.md) | Six-wheel reduced dynamics with original Fusion-exported visual meshes; portable headless checks | Whole-mechanism dynamics, calibrated tires, capture or field scoring |
| [Pressure/contact research verification](sim/pressure/VERIFYING.md) | Current and exact-source historical numerical experiments, separately registered | Automatic transfer between contact laws or successful acquisition-to-lift |
| [Native floor pickup and lift](sim/pressure/RETAINED_FLOOR_PICKUP.md) | Loose NOTE, driven pickup, transport, active stop, 60° pitch and full-height hold in a planar contact simulation | Installed hardware, driving while raising, feeder intake and scoring |
| [Native pickup through a turning lift](sim/pressure/TURNING_LIFT.md) | Floor pickup, active stop, then differential drive through the full pitch, raise and hold with a free NOTE | Higher-speed operating envelope, hardware calibration and scoring |
| [Legacy A2 slice](#run-the-retained-legacy-a2-slice) | Earlier geometry and prescribed cycle experiment | Validation of the current component robot |

Actual CAD delivery requires the separately preserved Fusion archive. Public STEP/meshes are source-sized references, not a substitute for that archive. Supplier native CAD is excluded from this public repository.

See [checkpoint qualification and reproducible smoke matrix](experiments/a2-campaign/MERGE_READINESS.md) for tested scope and remaining limitations. A merge would accept this experimental checkpoint only, not the open physical or mission outcomes.

## Current design highlights

- Close-range Speaker cycle narrative, with Amp and feeder alternatives retained: [strategy](experiments/a2-campaign/STRATEGY.md).
- AM14U4-based six-wheel drivetrain with source-sized components, an original front-throat modification and fixed bumper candidate. Structural and impact qualification remain open.
- Reversible asymmetric side-pinch intake, coordinated rotation/elevator and four-wheel launcher. Original drilled lower brackets, portal gussets, carriage yokes and recessed slide-pad attachments connect previously reserved interfaces.
- 711 original/source-sized parts per exported pose. Full-NOTE intake feeding is screened at49 positions; moving mechanism clearance at125 poses. These are finite nominal geometry checks, not capture, tolerance, force or durability acceptance.
- Actual chain attachments, shaft torque/retention couplings, motor mounts and remaining shooter interfaces are still completion gates. No previous launch result automatically transfers.

## Open and edit the current candidate

- [Current original floor-pose STEP](cad/assembly/connected-floor.step), [design and limits](cad/assembly/INTAKE-CANDIDATE.md), [parameter contract](cad/assembly/intake_components.json).
- [Connected generator](cad/assembly/build_connected_candidate.py), [source-bound evidence](cad/assembly/evidence/connected-report.json), [render provenance](cad/assembly/connected-visual.json).
- With existing OCP8 and Matplotlib: `python cad/assembly/build_connected_candidate.py`. This produces floor, rotated and handoff STEP files, renders and exact-file verification receipts in `artifacts/connected-intake-candidate/`.
- Vendor native CAD is not distributed. Original authored stock/fasteners and source-sized purchased-component envelopes are distinguished in the part manifest.

## Retained pre-engine modeled integration

The earlier [integrated plant work](sim/integrated/README.md) couples a force-driven four-CIM drivetrain with two intake motors and a loose NOTE on one electrical bus. Chassis movement comes from motor/tire forces, and NOTE contact reacts back onto the robot. This is **not yet a complete drivable robot simulator**: lift/launcher phase coupling, physical parameter validation and several mechanical interfaces remain open.

With the existing Python/OCP dependencies, run a fresh bounded experiment:

```text
python sim/integrated/run_coupled_acquisition.py --output artifacts/my-coupled-run.json
```

The current open-loop experiment intentionally stops at its supported geometry boundary rather than inventing capture. [Observed braking controls](sim/integrated/README.md#threshold-braking-is-a-retained-negative-control) preserve overrun and stale-observation cases. Recorded mechanism experiments, numerical refinement and unresolved conditions are included for reproducibility; they are not scoring or hardware acceptance.

## New bounded checkpoint continuation

The [eight-motor phase pilot](sim/integrated/coupled_lift_README.md) continues the exact recorded indexed state for 10 ms with four drivetrain motors, two intake motors, one lift motor and one pitch motor on one bus. It preserves momentum, time and cumulative energy, and replaces the prior 45 kg robot hypothesis with 38 kg base plus 7 kg moving mechanism rather than adding that mass twice.

```text
python sim/integrated/run_regulated_acquisition.py --index --output artifacts/my-index.json
python sim/integrated/coupled_lift_run.py artifacts/my-lift-pilot.json
```

For one fresh, strictly registered chain, run `python sim/integrated/run_current_checkpoint_chain.py artifacts/my-fresh-chain`. It executes acquisition first and refuses continuation unless that complete fresh receipt matches the registered checkpoint bytes; it then executes the pilot and verifies state/energy continuity. A mismatched state requires new registration, not a snap to the old state. The final `chain.json` binds both newly produced outputs and is absent if a stage fails.

These are separate reproducible commands: the pilot validates and uses the committed indexed checkpoint, not the output path of the preceding command. A single continuous mission runner is still unfinished. With finite gravity-feedforward commands the cradle sags slightly and the free NOTE drops; it is not attached or successfully lifted. The retained coast control stops on an unsupported tiny-negative-pitch guide-domain boundary, not a confirmed collision. Contact stiffness, mechanism mass/inertia and current normal-force budgets are explicit hypotheses. This pilot does not adopt the new spring-cassette candidate or transfer historical launcher results.

The separate [finite-command envelope study](sim/integrated/coupled_lift_sweep_README.md) extends seven predeclared cases to 25 ms using a reviewed separation-only guide gate. It preserves unsupported contact boundaries and shows the cradle and NOTE moving independently, not successful retention. The one-command chain above intentionally retains its original 10 ms pilot.

## Run the retained legacy A2 slice

Requires existing Python 3 with NumPy, PowerShell, JDK 17 and a populated Gradle/WPILib cache. Toolchain is pinned to Gradle 8.11, GradleRIO/WPILib 2026.2.1 and JUnit 5.10.1. This models the 2024 game using the 2026 desktop toolchain, not a historical robot deployment.

```powershell
./experiments/a2-campaign/run-a2.ps1 -JavaHome '<JDK 17 directory>' -GradleHome '<Gradle cache directory>'
```

Open `NORMAL.html` in the printed results folder. `NORMAL_9V.html` and fault replays preserve comparisons; `SUMMARY.md`, `results.json`, `native/receipt.json` and `presentation-receipt.json` expose what ran and which inputs produced it. Each invocation requires fresh output and executes actual native tests. No robot address, installation or hardware deployment is performed.

The cache must already contain the pinned public dependencies. On a newly configured development computer, set JAVA_HOME and GRADLE_USER_HOME, then run the included `shared/examples/java-2026-sim/gradlew.bat test` from that project directory with network access to populate its cache before using the offline verified runner. Dependency notices are [here](experiments/a2-campaign/shared/examples/java-2026-sim/THIRD_PARTY.md).

## What the legacy run proves

The nominal 12 V and permissive 9 V cases each record one prescribed acquisition and two powered exits in about 27.3 seconds. Missing piece, jam, stale pose and strict 9 V RPM controls fault without invented completion. Actual exit states feed sampled full-NOTE aperture checks, not a prescribed launch speed. These are sampled thin-plane results, not scores or complete target clearance.

Acquisition, lift, telescope, level retention and transfer are prescribed kinematics with virtual observations. Their actuation, contact, energy and physical capture are not validated. Shooter energy accounting covers rollers/guide only. Body attitude is a fixed 60-degree hypothesis; spin, aerodynamics, roof/walls, tire slip, realistic opponents and impact durability remain unresolved. The full serialized cycle does not fit 15-second autonomous.

## Retained earlier CAD and planning tools

- [Stowed STEP](cad/a2/stowed.step), [deployed STEP](cad/a2/deployed.step), [stowed picture](cad/a2/stowed.svg).
- [Geometry contract](cad/cycle-contract-A2.json), [original generator](cad/build_a2_layout.py), [shared kinematics](cad/a2_kinematics.py).
- 54 original solids per endpoint are exported and reopened. Green regions reserve linkage, transfer or service space; they are not engineered hardware. Narrow nominal envelope allowances are not tolerance/strength approval.
- With existing OCP 8 and Matplotlib: `python cad/build_a2_layout.py --output artifacts/my-cad`.
- The [portable editable pre-vis](kickoff/blockout/README.md) lives in this project. It saves/loads the plugin design artifact and exports editable OpenSCAD; it is a generic planning tool, separate from the A2 mechanism contract.

## Project ownership and retained dependencies

This repository owns the 2024 narrative, CAD, code, replay and reliability plan. The self-contained `shared/` snapshot retains cross-season regression fixtures, including CORAL/FUEL cases. Their presence does not make them 2024 mechanisms or require private repositories. `PROVENANCE.json` distinguishes the inherited base from A2 modifications and pins this project snapshot.

The old `run.ps1` remains a legacy preloaded all-season regression entry; it is not the new full-cycle runner. Earlier rejected geometry and failed lift experiments are retained as clearly labeled evidence, not silently promoted into the active model.

Before publication: Python tests, native tests, fresh receipts, public-content guard and original CAD metadata review. No private hosting configuration or third-party team/vendor CAD is included.

Current launcher spacing and original mounting correction: [revision and remaining gates](cad/assembly/launcher-mount-revision.md). The selected49.2mm gap requires fresh force/contact simulation; old launch speeds do not transfer.

An additive [spring-biased cassette study](cad/assembly/spring-cassette-candidate.md) preserves failed layouts and sampled feed conflicts. It is not adopted by the 711-part assembly or current simulation: actual springs, opening actuator, clamps and moving mass remain unresolved.

The [100 ms extension](sim/integrated/coupled_lift_duration_README.md) records partial fixed-patch domain loss: 28 of 30 nominal witnesses remain at the rejected trials. This is not proof of full physical release or a collision. Exact accepted state and rejected trial diagnostics stay separate.

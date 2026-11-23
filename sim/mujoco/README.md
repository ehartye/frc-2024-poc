# Bounded MuJoCo drivetrain with Fusion robot visuals

<!-- cad-still -->
[![Preliminary robot with the scoring cassette raised above the blue bumper](../../cad/preliminary-design/media/002-preliminary-robot.png)](https://ehartye.github.io/frc-2024-poc/preliminary-design/)

**Guided cassette at the start of a cycle, with a floor NOTE ahead. Preliminary design release 004, in progress.** The angled cassette picks up, retains and shoots a NOTE. The parallel guides support the climb. This still uses the same robot geometry as the interactive preliminary review. [Still and geometry record](../../cad/preliminary-design/media/002-preliminary-robot.json)
<!-- /cad-still -->

This is the current **controllable drivetrain slice**, not a complete robot mission. It uses the actual 711-part Fusion-exported robot geometry for display and the separately tested six-wheel native-ground dynamics for movement. The old pressure/acquisition simulator and all its evidence remain unchanged.

## Scope and current result

- Four-CIM electrical/drag law, six source-sized wheel cylinders, gravity, XYZ/yaw chassis and four soft wheel-coupling constraints. Chassis roll/pitch are locked. Total 45 kg and its mass/inertia partition, tire friction/compliance and fixed 12 V supply are hypotheses, not measurements.
- Source wheel centers preserve the 1.778 mm center drop. At rest the two center wheels support the modeled robot. Do not assign six artificial equal normal forces.
- Reflected motor inertia is on each central driven hinge once; the physical side inertia budget is preserved. Contact chatter remains: roughly 11% zero-support samples in the turning diagnostic, despite consistent integrated vertical impulse. No instantaneous tire-force calibration or full load-transfer acceptance.
- The bundled legacy three-mesh export contains 711 source roots / 714 bodies and 63,864 triangles. The loader also accepts version 2 native exports with multiple appearance meshes per motion group, explicit body coverage and documented excluded bodies. The privately supplied colored export was checked with its matching F3D: eight meshes, 57,988 triangles, continuous red bumper and white numbering. Its archive, supplier-derived meshes and images are not distributed here.
- A separate zero-DOF display model means these meshes add **no collision, mass, inertia or actuator behavior**. The mechanism stays in its floor pose, and CAD wheel visuals do not rotate individually.
- Actual full-mesh UI test observed W/S/A/D/C/B/Q callbacks and drive/turn/coast/brake commands, 0.09728 J motor work and graceful quit. It ran 18.62 simulated seconds in 31.04 wall seconds (0.600 cumulative real-time factor) on that test. This is a bounded observed run, not a general performance guarantee or physical validation.

## Setup and run

Use Python 3.13 and a project-local virtual environment. Installation is an explicit user action; no script installs software automatically. Versions tested are in `requirements-lock.txt`. No engine binaries or third-party native CAD are bundled.

```powershell
python -m venv .venv-mujoco
.venv-mujoco\Scripts\python -m pip install -r sim/mujoco/requirements-lock.txt
```

From the repository root, open the six-wheel physics-only viewer:

```powershell
.venv-mujoco\Scripts\python -B sim/mujoco/native_drive_interactive_v3.py --view --trace native-drive-ui.jsonl
```

For actual robot visuals, supply the separate original Fusion archive whose SHA-256 is recorded in `visual/manifest.json`. The archive is deliberately not bundled in this change; an arbitrary F3D or STEP cannot pass the identity gate.

```powershell
.venv-mujoco\Scripts\python -B sim/mujoco/fusion_visual_drive.py --manifest sim/mujoco/visual/manifest.json --fusion-archive PATH_TO_MATCHING_SOURCE.f3d --trace fusion-drive-ui.jsonl
```

For a private version 2 export, pass its external `manifest.json` and matching native F3D to those same arguments. Every exported mesh is checked by name, hash and bounds; mesh names are distinct from motion groups. Excluded bodies remain in the source inventory and require an exclusion reason. Do not copy private exports into the repository.

WASD drives, C coasts, B or space brakes, Q quits. A short tap latches for 100 ms wall time; held input extends it. Losing focus clears input and commands braking without resetting momentum. Arrow held polling exists but is not UI-verified; arrow callbacks were not recognized and are not advertised as working taps. Held-key/focus detection is currently Windows-only. Headless model/tests do not require the Windows input API.

Output paths must be fresh. The UI records actual callbacks, focus, commands, state, motor work and wall/sim timing. Viewer reset/perturbation controls cannot modify physical state. Avoid minimizing the viewer during automated desktop tests because the current automation surface cannot restore a minimized target reliably.

Drive keys also reach MuJoCo's built-in display shortcuts. The Fusion runner restores only the conflicting display flags, keeping the floor, shadows and solid meshes visible while driving. Unrelated view options and camera controls remain available.

## Verification and lineage

Run the 28 focused tests from this directory:

```powershell
python -B -m unittest test_native_drive test_native_drive_interactive test_native_drive_interactive_v3 test_fusion_visual_display test_fusion_visual_provenance
```

`evidence/portable-visual-verification.json` is a fresh no-render run of this portable code: 30 forward/turn/brake ticks exactly equal a no-display physical reference, and source/config/geometry inputs are checked before and after. Nine internal mesh-frame bound checks share the same pose helper; they are **not independent Fusion mechanism-pose validation**. Synthetic tetrahedra in unit tests are parser/registration/isolation fixtures, never robot assets.

`migration.json` maps preserved audit bytes to this portable copy. Only repository-relative source-root defaults changed. Native physics and geometry code remain byte-identical. Receipts prefixed `historical-` keep executed audit-source hashes and are not relabeled as portable UI tests. The compact historical UI summary removes machine-uptime callback timestamps; it retains the private raw trace hash and documents this transformation. The raw trace and large native-contact diagnostic traces are retained outside this repository. Their hashes in compact summaries support lineage but are not independently replayable from the summaries alone.

The historical UI test used the original audit implementation. The new `evidence/native-color-viewer-checkpoint.json` separately records verification of the colored native export: 24 internal mesh-frame checks, exact physical parity across 30 ticks, and a real interactive run with WASD/C/B/Q and focus-loss braking. After fixing display-shortcut conflicts, a user-run trace records WASD/Q and the user confirmed the requested controls and stable visuals. Both traces are source-bound; the post-fix trace matches the final runtime inputs. The legacy export also passed its nine checks with the updated loader. These are narrow, uncalibrated checks. No capture, retention, lift, launcher, field collision/scoring, tolerance, structural or installed electrical acceptance follows from this viewer.

The broader `tests/` run completed 515 tests: 502 passed, four failed and nine errored. All 13 failing cases reproduced on unchanged reconciliation baseline `ca7f2e8`; they concern older component/packaging and pressure receipt/source mismatches. They are listed in the checkpoint summary and remain unresolved. Historical hashes were not rewritten, and this viewer checkpoint does not claim a full-repository passing suite.

## Assets and licensing

Repository-original code and original/source-sized robot representations retain the repository MIT license. Visual meshes come from the original project CAD already represented by `cad/assembly/connected-floor.step`, through the hash-identified Fusion working archive. Manufacturer native test CAD, private drawings, private wiki material and host-specific paths are excluded. Public availability of a vendor CAD file is not a redistribution license. MuJoCo and Python dependencies retain their own upstream licenses and are installed separately.

# Bounded driving with a physically retained NOTE

Status: **eight verified simulation experiments; no complete driving/shooting robot or hardware acceptance**.

The [static-contact fixture](NATIVE_STATIC_CONTACT.md) could retain a staged NOTE but had world-fixed belt surfaces. This additive runner puts the existing six-wheel drivetrain, two finite-inertia belt patches and a free six-DOF NOTE into one MuJoCo model. Contact reactions reach the chassis through the same dynamics solve. No NOTE pose or velocity is assigned after initialization; the four existing equality constraints connect drivetrain wheels only.

## What changed

The two native-backed straight belt faces become X-axis sliding material patches relative to the chassis. Each has a declared 0.05 kg material mass and the existing 3e-5 kg m2 motor rotor hypothesis reflected once through the source 12-tooth pitch radius. The 0.1 kg material mass is redistributed from the existing 45 kg robot hypothesis, preserving its initial center of mass and yaw inertia. The NOTE adds 0.235 kg. These are exploratory inertias, not a measured native mass ledger; chassis roll and pitch remain constrained.

An ideal reciprocal DC motor law applies finite forces to each belt coordinate, with 12 V and 40 A limits. Position/velocity feedback requests force; it cannot assign belt motion. Motor-off coasting removes that force. Drive and pinch use one ideal fixed voltage, not a solved resistive battery or validated regenerative power system. Motor electrical power equals winding heat plus mechanical power in the recorded samples; this does not validate every contact-energy term.

The final relieved F3D hash remains mandatory at launch. The source faces and other geometry/configuration files are pinned. The staged carry height is a fixed 0.3 m hypothesis, not a demonstrated lift or cleared native assembly pose. The rotor/slider reduction reproduces tangential inertia and force transfer but omits actual shaft/stator angular reaction, curved returns, belt tension dynamics, native hard parts and rotor gyroscopic coupling. It stops when either belt travels more than 1 mm, NOTE XY motion exceeds 1 mm, NOTE tilt exceeds 0.01 rad, or a solved contact approaches within 2 mm of a patch endpoint. These local guards do not prove continuous hard-part clearance.

The earlier stationary-fixture stiffness fit is reused without tuning against retention outcomes. Adding the chassis changes the solver's effective contact response: initial normal force is approximately 2.865 N per side, not the stationary fit's 3.765 N target. At initial ground settling, resultant NOTE force peaks around 15.2-15.4 N. This transient and the normal-force difference are retained as model limitations, not measured loads. Friction, compliance, material behavior, controller gains and installed pose remain uncalibrated.

## Executed results

MuJoCo 4.0.1 alone advances all state. Eight complete runs pass `verify_native_drive_retention.py`, including trace/model hashes and the current executable input files. No engine warnings occur. All measured friction-cone excesses are below 1e-8 N; independent contact-force projection agrees with the NOTE's generalized constraint forces below 1e-10 N.

| Run | Result |
| --- | --- |
| Forward then brake, 1.5 s | Chassis advances 0.425705 m, peaks at 0.638393 m/s and ends at 0.032600 m/s; NOTE remains supported |
| Differential turn then brake | Chassis advances about 0.361 m and reaches 0.041511 rad yaw; NOTE remains supported |
| Belt motion/current during forward run | Maximum 0.529828 mm travel and 0.378873 A; not a locked or infinite-force belt |
| Half timestep, 0.1 ms | Retention remains; final chassis X differs by 0.137327 mm |
| Finer NOTE, 256 capsules | Retention remains; maximum vertical relative motion is 0.087747 mm versus 0.135719 mm at 128 capsules |
| Belt motors coast | Belt/NOTE motion crosses the 1 mm domain boundary; execution stops rather than extrapolating into an unmodeled belt return |
| No-contact / low friction | Both drop 50 mm; no-contact motion matches analytic discrete free fall within 2.23e-16 m |
| Free-floating internal motor pulse | NOTE moves forward and chassis backward; total mass-weighted center of mass changes less than 2.37e-18 m |

The internal-pulse control disables gravity and floor contact and applies a finite current pulse for 0.05 s. It tests action/reaction rather than driving. The force/power checks, limited timestep comparison and geometry refinement do not establish full numerical convergence or hardware accuracy. The three existing drivetrain tests also pass; no full-repository passing claim is made.

A development version's arbitrary 10 mm endpoint buffer stopped the finer discretization immediately. Final runs use the more restrictive 1 mm material/NOTE travel domain and a documented 2 mm contact-point margin. The earlier stopped trace remains preserved, not relabeled as a pass. The final limits are local modeling boundaries, not fabricated hardware tolerances.

## Next integration boundary

This demonstrates bounded driving/braking/turning with physical retention under explicit assumptions. It does **not** acquire a floor NOTE, raise or rotate the cradle, circulate the complete belts, transfer to the launcher, or shoot. The earlier launcher-only and pressure-model evidence remains separate. A complete native belt/transfer geometry check, finite lift/pitch mechanics, shared electrical supply, contact calibration and launcher coupling are still required before a complete demo can be claimed. The canonical driver and earlier experiments are unchanged.

Fusion transport discovery recovered, but both a fresh read-only document inventory and the advertised active-command read timed out. The preceding audit import has an uncertain execution outcome; it was not replayed. Native integration is blocked until existing document state can be inspected. No desktop operation, Fusion restart, or native design mutation occurred in this continuation.

## Run headlessly

Use the existing MuJoCo/NumPy environment and a fresh private output directory:

```powershell
<mujoco-python> -B sim/mujoco/native_drive_retention.py --project . --native-archive <private-final-relieved.f3d> --output <fresh-output>
```

Other cases use `--scenario turn`, `--dt .0001`, `--segments 256`, `--belt-mode coast`, `--no-contact`, `--mu .1`, or `--scenario reaction --duration .05`. Verification expects the eight named case directories documented in the verifier:

```powershell
python -B sim/mujoco/verify_native_drive_retention.py --project . --packet <private-packet> --output <fresh-verification.json>
```

Full XML/traces remain private. The [portable evidence](evidence/native-drive-retention-20261118.json) contains the verified scalar receipts and source pins. No native/vendor CAD is published. MuJoCo's [joint armature](https://mujoco.readthedocs.io/en/latest/XMLreference.html#body-joint) and [force projection API](https://mujoco.readthedocs.io/en/latest/APIreference/APIfunctions.html#mj-applyft) document the mechanisms used; their availability does not qualify this model.

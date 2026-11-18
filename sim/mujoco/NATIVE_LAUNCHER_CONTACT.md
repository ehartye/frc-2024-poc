# Native-backed launcher and drivetrain experiment

This experiment connects four rotating launcher treads, a free six-degree-of-freedom NOTE, the existing six-wheel drivetrain and a shared finite electrical supply in one MuJoCo solve. Contact forces cause release and react on the robot. **It is not an accepted shooting model or a complete driving/intake/lift/shoot demo:** collision refinement changes both speed and whether the NOTE releases within the experiment. No hardware prediction or scoring claim is supported.

The source of truth remains the preserved, reopened **734-body native Fusion archive**, SHA-256 `5f2a1a4cb98931f31575444e95323c9cb78bab8df6d74f226c34dc53bd62bf96`. This work made no CAD changes and did not interact with the uncertain live Fusion session. Native CAD and supplier source geometry remain private. The earlier drive/retention experiment remains separate; its state is not teleported into this launcher.

## Native binding and assumptions

`evidence/native-launcher-binding-20261118.json` pins the final native receipt, archive and configuration. It checks four tread centers, bounds and annular volumes against the final receipt: maximum bounds difference is below 5.8e-12 mm. The intermediate 735-body inventory is explicitly rejected as final assembly authority. This is an offline preserved-receipt check, not a fresh live-document verification.

The launcher uses 48.26 mm tread radius, 25.4 mm width, 49.2 mm nominal gap, two lanes at Y +/-152.4 mm, and the existing 60-degree pitch. The NOTE starts already inside the launcher, 30 mm upstream of its wheel station, at rest. It has no attachment, prescribed feed velocity, or post-initialization pose assignment. Four model equalities connect drivetrain wheels only.

Each rotating wheel has hypothetical mass 0.2 kg and spin inertia 0.0003 kg m2. The 0.8 kg is redistributed from the 45 kg robot hypothesis, preserving initial COM and yaw inertia; the NOTE adds 0.235 kg. Rotor torque reactions exist through hinge joints, but chassis pitch and roll are constrained. Cylinder tread exteriors omit annular cavities; a contact inside the native seat radius stops the experiment. Other hard parts are not dynamic collision bodies.

The existing reciprocal DC law uses finite 40 A phase limits and 0.8 duty. The existing pure shared-supply solver couples launcher and drive power with a 12 V open-circuit source and 0.02 ohm resistance. Mass, inertia, friction 0.8, preload, compliance and supply/controller behavior are hypotheses. There is no battery transient, aerodynamic, field-target or thermal model.

## Verified evidence and rejected prediction

The offline verifier checks **27 complete runs** in MuJoCo 4.0.1 against their exact archived executables, input/model/trace hashes, free NOTE constraints, finite current, contact-force projection, friction cones and electrical ledgers. No engine warnings occurred. Every receipt has `acceptance: false`.

The original 128-capsule experiment physically releases the NOTE from rest; a simultaneous drive run moves the chassis 20.38 mm while shooting in the same solve and reduces minimum bus voltage from 9.44145 to 9.32486 V. Those are model observations, not validated robot performance. Zero launcher drive and zero friction do not produce forward release. The no-contact run matches discrete analytic free fall within 1.00e-15 m. Gravity/floor-disabled internal actuation conserves system COM within 1.28e-16 m. Post-contact trajectories match discrete free flight within 4.98e-14 m.

Maximum motor power identity error is 8.53e-14 W, bus-port power error 1.71e-13 W and supply voltage balance error 7.25e-12 V. Halving the timestep changes original baseline last-contact speed by 0.001852 m/s and reduces the NOTE work residual from 0.003011 to 0.001505 J. These checks do not establish complete compliant-contact energy conservation.

Geometry sensitivity rejects a speed prediction:

| Contact experiment | Result |
|---|---|
| Original normal-force fit, 128 / 256 / 512 / 1024 capsules | Last-contact speed 3.221 / 2.760 / 2.349 / 2.002 m/s |
| Match initial normal force **and** local damping, same resolutions | 3.221 / 2.560 / 1.944 / 1.616 m/s |
| Disable optional NoSlip postprocessing | Same strong resolution trend; no resolution of the defect |
| Analytic torus, 40 search points / 20 iterations | Released by 0.228 s, last-contact speed 1.661 m/s |
| Analytic torus, 80/40 and 160/80 | Still in contact at 0.4 s; no supported release verdict |

The initial normal-force target is 25 N per wheel. The matched series preserves the baseline local normal damping response of 8.084465 N s/m, measured at +/-1 mm/s with gravity and friction disabled. It does not fit ejection speed or measured hardware. Matching that local response does not preserve the entire contact law as the contact manifold changes. Ring-phase controls, an alternative 50 N s/m damping hypothesis, and the initial analytic-torus pilot are also retained; none is silently promoted to a successful model. A separate pre-spun case starts with **21.3183 J** of rotor kinetic energy and is explicitly not a continuous spin-up/capture demonstration.

The analytic torus uses the already-installed official `mujoco.sdf.torus` plugin. Its [official example](https://github.com/google-deepmind/mujoco/blob/main/model/plugin/sdf/torus.xml) and [SDF collision documentation](https://mujoco.readthedocs.io/en/stable/programming/extension.html#sdf) describe the geometry and numerical search. The search controls here refute acceptance; using an analytic shape does not by itself prove converged contact forces.

`HORIZON` means the 0.4 s horizon was reached. It does not imply retention: the fine capsule case releases but never reaches the runner's 0.3 m downstream boundary. `PASSED_LOCAL_LAUNCHER_DOMAIN` means only that local position boundary was crossed without remaining tread contact.

## Clearance evidence and next decision

`evidence/native-launcher-clearance-20261118.json` records offline OCP bounds of the hash-verified bare-wheel source. Sixty-four support directions give a 47.046670 mm containing radius valid for every source clocking, conditional on the configured installed transform. The baseline rest-launch trajectory has 278 finite saved poses and 6,120 containing-envelope distance queries; the minimum gap is 0.353247 mm after torus sagitta inflation. This is neither continuous clearance nor a clearance result for every numerical variant.

Conservative annular broadphase against the preserved native scan covers 712 physical bodies, including hidden material. Eleven other hard-part bounds survive: bearing carriers, motor sliders, carrier bridges, belt-pitch representations and one motor envelope. These are **unknown**, not proven collisions. Full material checks, actual installed registration/tolerances and compliance remain unresolved.

Do not size a shooter or change the native design based on a favorable coarse trace. The next numerical task is a displacement/velocity contact-manifold sweep with a defined resolution-independent compliant law, followed by independently specified convergence criteria and measured NOTE/tread calibration. The next CAD task is to reconcile the uncertain live document state read-only with the owner's permission, then check those eleven native parts against the supported trajectory. Acquisition, retention, moving lift/pitch and launcher transfer must eventually share one uninterrupted state before a complete robot demo is claimed.

## Run and verify

Use Python with the installed MuJoCo 4.0.1 and NumPy environment. From the repository, provide the private native archive and a new output directory:

```powershell
python -B sim/mujoco/native_launcher_contact.py --project . --binding sim/mujoco/evidence/native-launcher-binding-20261118.json --native-archive $NativeArchive --output $NewRunDirectory --segments 128
```

Add `--drive-duty 0.15` for simultaneous drive demand. For the analytic torus comparison add `--surface sdf --noslip-iterations 0 --sdf-points 80 --sdf-iterations 40`. These are offline experiments, not an interactive robot launcher. The current runner matches initial force and damping; the original control series is reproduced using its archived executed source in the private packet.

```powershell
python -B sim/mujoco/verify_native_launcher_contact.py --packet $PrivatePacket --project . --manifest "$PrivatePacket/launcher-run-manifest.json" --binding sim/mujoco/evidence/native-launcher-binding-20261118.json --output $NewVerificationFile
```

The packet contains all run XML, traces, receipts, executed source versions, unsuccessful exploratory variants and native-audit helpers. The public repository contains sanitized scalar evidence only. Passing the verifier means integrity and stated controls passed; `speedPredictionAccepted` and `acceptance` remain false. GUI operation, hardware validation and a complete native robot cycle are not claimed.

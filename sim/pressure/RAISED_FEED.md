# From the turning lift to a free NOTE

The 2024 close-range Speaker study continues the verified floor-pickup and turning-lift trajectory at 16.75 seconds. The NOTE, chassis, carriage, pinch rotors and stored tread deformation retain their attained states. Four current-limited launcher wheels start at rest. The model preserves chassis mass, center of mass and planar yaw inertia when it allocates the wheel bodies. Wheel and NOTE contact uses the [native quadrature geometry and law](../mujoco/LAUNCHER_QUADRATURE.md). The intake retains its finite-area pressure law. As the normal load falls, the spring force is projected onto the available friction limit and the released spring energy is recorded. The material and electrical parameters are simulation hypotheses.

The shared supply powers the drive, lift, pinch and launcher. Once all four wheels exceed the design's 1,800 rpm readiness threshold, the pinch ramps over 100 ms while the launcher continues to accelerate. The ring is free to move throughout. The refined run feeds at 16.83 s, has simultaneous positive intake and launcher forces for 6.55 ms, and loses its last launcher contact at approximately 17.022 s. A further 30 ms of upward free motion establishes release. Holding the feed prevents launcher contact. Replaying the nominal feed clock with the launcher unpowered produces simultaneous contact and no upward release in the observation window. [Five-run comparison](evidence/raised-feed-20261118/comparison.json)

The spatial and timestep refinements agree on release. Maximum shared-time position differences are 0.846 and 0.321 mm. The corresponding velocity differences of 0.0435 and 0.1300 m/s exceed the earlier staged study's 0.01 m/s criterion. The simulations support a qualitative handoff. Quantitative contact convergence and hardware calibration remain open. An earlier passive-wheel trial stopped at a tiny negative calculated dissipation. The current per-region summation matches the friction solver's summation and rounds the maximum spring force inward by sixteen machine epsilons. The original rejection remains in the private packet.

A native Fusion check covered all 6,037 integrated steps of the refined run and subdivided the 284 initially ambiguous wheel-shell intervals. The final 6,025 Boolean pair checks left zero unresolved intervals above the 0.0001 mm³ reporting threshold. This applies to the registered rigid geometry and the interpolated trajectory. The coarse pilot produced one wheel-shell overlap of roughly 0.010 mm³. The native check retained that counterexample. [Refined native clearance](evidence/raised-feed-20261118/native-clearance.json)

## First wing pickup and opening

A retrospective field registration places the original floor NOTE on the nominal blue inner wing mark (114, 161.625 inches) and rotates the entire robot history through 90 degrees. With gravity-only flight, the smallest certified rim clearance in the refined calculation is approximately 102 mm at the nominal opening. The calculation bounds the whole torus over space and time, including precession. A 750 mm lateral shift loses that clearance. The shooting location was selected after observing the route. Wall and ramp contacts, competition variability and a downstream sensor event require separate evidence. The verified score count is zero. [Opening calculation](evidence/raised-feed-20261118/opening.json)

The opening uses the low wall, upper hood and inner side-panel dimensions from FIRST's 2024 field assembly. The low-edge longitudinal coordinate is inferred from the drawing's horizontal run. [2024 field drawings, GE-24300 Rev A, sheet 2](https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024FieldDrawings.pdf)

## Reproduce

Use the MuJoCo 4.0.1 environment and the verified private native archive and pickup packet. Supply a fresh output directory for every run. Raw states and supplier geometry belong in private storage.

```powershell
python -B sim/pressure/run_launcher_handoff.py --pickup $TurningPickup --archive $NativeArchive --output $NewRun --segments 16384 --grid 1024 --dt .00005 --horizon .34
python -B sim/pressure/wing_speaker_passage.py $NewRun $NewOpeningReceipt
```

The spatial trial uses 8,192 segments and 0.1 ms, the temporal comparison uses 16,384 segments and 0.1 ms, and the refined run uses 16,384 segments and 0.05 ms. The hold and unpowered controls use 8,192 segments, 0.1 ms and a 0.34 s horizon. Use `verify_launcher_handoff.py --help` to compare their complete traces. The native pose exporter writes the fixed, carriage and cradle reference frames for saved samples.

# R7 mass properties in the floor pickup pose

The nominal operating model weighs **48.015 kg** including battery and bumpers. Its center of mass is **(7.7, −29.8, 282.5) mm** from the robot-floor origin, with +X forward, +Y left and +Z up. Its yaw inertia at that center is **5.846 kg·m²**. These are engineering estimates for the registered-front-crossmember assembly in its lowered pose. [Editable archive handoff](../native-handoff.md) · [All three mass cases](r7-floor-mass.json)

| Estimate | Mass (kg) | CG X, Y, Z (mm) | Yaw inertia at CG (kg·m²) |
| --- | ---: | --- | ---: |
| All light | 43.640 | 7.6, −29.8, 286.2 | 5.252 |
| Nominal | 48.015 | 7.7, −29.8, 282.5 | 5.846 |
| All heavy | 54.294 | 8.4, −29.5, 279.2 | 6.672 |

The CG can move further if different components take opposite ends of their assigned ranges. Varying each parcel independently puts X between −9.4 and 25.4 mm, Y between −47.5 and −13.0 mm, and Z between 265.2 and 301.5 mm. The individual extremes need different mass distributions. The static central yaw inertia increases monotonically with each positive parcel mass when the positions and normalized inertias are fixed. [Independent intervals](r7-independent-bounds.json)

### Assigning mass

- Fusion's assigned steel would give a single side noodle about 19.7 kg. Use all eight native foam volumes at 20–55 kg/m³, giving 0.391–1.074 kg total. Include four separate wood parts at 450–750 kg/m³. Count the physical cushions and backing even though they are hidden beneath the displayed continuous cover. Allow 0.25–0.80 kg for the covers.
- Use the catalog mass for each of the 18 installed electronic devices with a ±5% screening range. Count the four REV modules at 1.720 kg each nominal including motors. The eight other NEOs have separate parcels. Use one parcel per module within its installed envelope. [Electrical BOM](../electrical/manifest.json) · [Module selection and dimensions](maxswerve-layout.json) · [REV module mass](https://docs.revrobotics.com/ion-build/motion/maxswerve)
- For machined material use the surveyed solid volume and an assigned density range. For chain, pulley and bearing envelopes use effective densities. Use separate assumed masses for the two gearbox envelopes. Add explicit distributed allowances for harnesses, retention, welds and installation hardware. All material classes, quantities, individual case masses and allowances are in the budget. Exact purchased-part masses, installation hardware and wiring still need reconciliation against the finished robot.
- Keep the estimated material distribution of each native solid for yaw. Estimate its other two central moments from its footprint aspect and vertical extent. Place purchased components at their modeled envelope centers. The off-diagonal terms include the parallel-axis contribution from the surveyed locations. Internal mass distributions, within-part products of inertia, actual corner loads and dynamic weight transfer are unmeasured.
- Integrate the fixed chassis, carriage and pivoting cradle separately. The nominal carriage and cradle are 1.870 and 1.828 kg. The MuJoCo partition deducts the explicitly articulated forks, wheels and launcher rotors from the fixed parcel while conserving mass, first moment and the tensor. The controller uses nominal lift and pitch feedforward for all three cases. The planar drivetrain retains its equal-wheel-load traction assumption. [Model binding](../../sim/pressure/r7_dynamics.py) · [Partition regression](../../tests/test_maxswerve.py)

### Reproduce the estimate

Use the authorized native survey and display manifest from the handoff. The survey enumerates installed occurrence paths, physical centers, bounds, volumes and the native polar moment divided by mass. The exporter excludes study geometry. The budget matches physical solids to the R7 motion groups, replaces supplier bodies with module parcels and excludes motor-withdrawal reserves and display graphics.

```powershell
python cad/drivetrain/r7_mass_budget.py artifacts/native/r7/physical-mass-r7.json artifacts/native/r7/display-manifest.json artifacts/r7-budget.json
python cad/drivetrain/r7_mass_budget.py artifacts/native/r7/physical-mass-r7.json artifacts/native/r7/display-manifest.json artifacts/r7-bounds.json --independent-bounds
```

The mass cases screen sensitivity to the assumed BOM. The native starting-size survey, electrical packaging, physical material calibration and six-score repeatability each require their own evidence. [Starting-size survey and frame build](MAXSWERVE_FRAME.md) · [Mission results](../../experiments/speaker-cycles/mission.md)

# Floor pickup and retained lift

The native side-pinch model collects a loose 2024 NOTE, carries it 225.56 mm on the floor, brakes, and raises it. With the commanded 8 mm lift, the whole ring cleared the floor throughout the 0.30 s hold in five simulations. In the 1024-square, 0.5 ms run, the minimum hold clearance was 7.38 mm. The earlier regularized-friction model returned to the floor with the same motor duties. [Recorded measurements](evidence/retained-pickup-20261118/short-lift-verified.json)

## What carries the load

The native finite-area pressure law supplies the normal force. The two fore/aft contact regions each add an axial elastic shear coordinate, initially relaxed, with a hypothesized stiffness of 2500 N/m per region. Each region distributes its elastic force in proportion to local normal load. Sliding dissipation uses the remaining circular friction budget at each sample. The generalized rate of elastic displacement is the corresponding weighted relative surface velocity. This makes stored energy and reciprocal NOTE, cradle and rotor work explicit. A region that reaches its friction bound or loses support while loaded ends the sticking experiment. No contact attaches the NOTE to the carriage.

The model preserves the original finite-effort motors, shared supply, compliant floor and free NOTE pitch dynamics. Its current fore/aft shear distribution is a reduced material hypothesis. Bond stiffness, contact pressure, tread friction, transmission losses and installed mass properties need measurement. A successful finite hold establishes a simulated load case. It does not qualify the hardware.

## Reproduce the short-lift comparison

From the repository root, use the existing Python environment with NumPy and OCP. Supply the private relieved native Fusion archive, the exact acquired active-stop receipt, and the previous half-step lift receipt. The latter supplies the 80 recorded controller events. The producer uses their motor demands at the original absolute clock ticks. All new body states come from integrating forces.

```powershell
python -B sim/pressure/run_axial_shear_lift.py --archive $archive --stop $stop --commands $oldLift --dt .0005 --output "$packet/shear-0.0005"
python -B sim/pressure/run_axial_shear_lift.py --archive $archive --stop $stop --commands $oldLift --dt .00025 --output "$packet/shear-0.00025"
python -B sim/pressure/run_axial_shear_lift.py --archive $archive --stop $stop --commands $oldLift --dt .000125 --output "$packet/shear-0.000125"
python -B sim/pressure/run_axial_shear_lift.py --archive $archive --stop $stop --commands $oldLift --grid 512 --output "$packet/grid-512"
python -B sim/pressure/run_axial_shear_lift.py --archive $archive --stop $stop --commands $oldLift --grid 2048 --output "$packet/grid-2048"
python -B sim/pressure/verify_retained_pickup.py $packet "$packet/verification.json"
```

The verifier checks all integrated clock samples, bilateral contact, source and contact work, energy drift and torus clearance. The finest timestep pair differed by 0.071 mm/s at most across translational velocities. The 1024 and 2048 grids differed by 0.046 mm/s. Both meet the previously declared 1 mm/s bound. Grid refinements reuse the attained stop and report the change in initial potential. They cover the lift interval only. The Agent FRC contact checker separately compared minimum hold clearance and final rise. [Agent FRC result](evidence/retained-pickup-20261118/agent-frc-result.json)

## Native geometry

The registration source is the private relieved-tread F3D. A live Fusion MCP reopen matched the 734 saved native bodies. Boolean intersections of the NOTE with the physical bodies at 81 attained short-lift poses found only intended tread contact. A separate scan checked 63 prescribed poses along the 60° pitch and full-height route. Six moving-assembly poses were checked for new or increased cross-group overlap. Sampled clearance leaves the spaces between samples and assembly tolerances unresolved. [Native geometry checks](evidence/retained-pickup-20261118/native-clearance.json)

The original [ground acquisition](NATIVE_SIGNED_PRESSURE.md) and [floor-return experiment](NATIVE_PRESSURE_LIFT.md) remain reproducible. The drivable viewer and these offline contact experiments have separate entry points.

# Drive-over loading acceptance

Start with an unrestrained NOTE lying flat, the robot behind it and the lift down. Use the native intake and continuous bumper in their installed positions. Drive forward so the NOTE enters under the bumper, engages the two powered intake surfaces and seats in the cradle. Close the loop on the resulting contact and motion. After braking, lift and pitch while the chassis drives and turns. Track the same NOTE through the hold and any later handoff. [Native geometry and acquisition path](../../cad/assembly/floor-acquisition.md)

| Stage | Observation required | Record |
| --- | --- | --- |
| Approach | Initialized floor position and heading, bumper clearance along the actual route, first bilateral intake contact | [Bumper fit and pickup clearance](../../cad/drivetrain/maxswerve-bumper-fit.json) |
| Capture | Both contacts under load, supported compression, bounded relative motion and seating dwell | [Retained floor pickup experiment](../../sim/pressure/RETAINED_FLOOR_PICKUP.md) |
| Carry | Continuous robot and NOTE states through braking, travel, lift and pitch. Record position, heading, bus voltage and side load at the station. | [Turning lift](../../sim/pressure/TURNING_LIFT.md) · [Swerve carry results](../../sim/pressure/evidence/maxswerve-development-carry.json) |
| Feed | Attained launcher speed and a free upward release from the measured terminal state. Count the field outcome separately. | [Swerve release and contact-sampling results](../../sim/pressure/evidence/maxswerve-development-shot.json) |

Report the first lost contact, unsupported substrate penetration, missed opening or missed sensor when it occurs. Repeat at a smaller timestep and vary contact sampling. A coarse visual pass cannot establish repeatability. All material response remains theoretical. The proposed overhead entry needs its own entry geometry and receiving test.

# Sliding-pivot cassette · P2.2

**Disposition: revise the pickup contact path in Preliminary design.** One cassette on a translating pivot handles both intake and launch. The two outboard supports react its shaft into the chassis. The mission is the Blue 1 close-speaker role. This study starts with the nominal NOTE and official field-opening dimensions. It consumes no Fusion archive, station placement or prior score.

| Check | Observation |
| --- | --- |
| Opening pose | 92 closed display meshes from named solids. 4,186 distinct parked pairs, 214 requiring the exact common-volume calculation; zero measured above 0.001 mm³, zero unresolved. |
| Fixed pivot control | The actual nose bracket exceeds the permitted forward reach by 177.2 mm at horizontal. |
| Moving pivot | X travel up to 197.5 mm. The bounding rectangle of the actual rotating stock stays inside X634 throughout the programmed swing; 10.8 mm nominal clearance to the R105 front limit at X644.8. Actual installation error, tracking error, rear and side compliance and a stuck-axis interlock remain separate. |
| Two frame supports | At least 70.2 mm lateral clearance to the centered NOTE on both sides. A displaced NOTE enters the left plate by 18,549 mm³. |
| Bumper and carpet | P2.1 entered the modeled bumper by 106.72 mm³ per side and reached 2.90 mm below carpet. Raising the pivot 14 mm cleared these failures in the 25-pose operating screens. The loaded swing has 11.10 mm minimum sampled clearance to the carpet. Finished bumper variation and inter-sample clearance remain open. |
| Floor contact | In a level approach, the nose roller is the first of the tested contact articles reached, at 16.13 mm of additional approach. Along the proposed tip-and-draw trajectory, all three powered surfaces simultaneously clear the NOTE by 37.88 mm at 18.75% stroke. The animation cannot establish that the ring will follow that path. |
| Powered center belt trial | A 45 mm-wide upper belt run reaches the ring where the 37.88 mm gap occurs. It still leaves 16.02 mm at entry and cuts the existing nose roller by 405.14 mm³ at its wrap. This trial is excluded from the assembly. Recess or replace the nose roller and test free-piece capture before allocating more slide hardware. |
| Battery service | Continuous 370 mm swept case, empty mechanism and leads disconnected. At the aiming angle the pull intersects the right belt by 8,645 mm³ and the right runner by 366 mm³. At 80° the swept case clears by 5.0 mm excluding its support deck. Specify the mechanical service prop and hand clearance before accepting the procedure. |
| Inspection weight | 31.0–49.5 kg using preliminary subsystem allocations and integration reserves. Upper planning margin 7.20 kg against the historical 125 lb limit. The purchased-article roster and measured load are open. |
| Weak pack | 11.8 V open circuit, 27 mΩ, 8 V planning floor. Staged aim/spin-up is 8.56 V worst case; simultaneous acceleration and spin-up violates the floor. These are allocated battery-side loads. |

The motion cases are geometric experiments with a fixed chassis datum. Pickup force, tire and belt slip, gearbox sizing, dynamic mass, feed speed, release and scoring require independent evidence. The initial slide, screw, gearcase and bumper are package envelopes. The public plugin contains general review methods. The robot data and LAN model are private.

## Reproduce

Run `check.py`, `travel.py`, `engineering.py`, `drive_contact.py`, then `present.py` with the Python/OpenCascade geometry environment. `layout.py` constructs the design. The STEP, actual pose matrices and numerical receipts are written beside these scripts. The fixed-pivot comparison is in the sibling greenfield directory. The frontend reads one atomically replaced data file. The source mesh and the selection renderer are independent of the Fusion station.


## Loaded retention

At −50° a 0.25 kg NOTE has 1.88 N of gravity toward the open mouth. Beyond vertical the deck cannot supply the opposing normal force. At 125° the outward component is 1.41 N. The full torus swept 80 mm along the open-face normal intersects none of the 91 neighbors; the center descends 45.89 mm. A separate keeper coupon has a 4.60 mm admission gap and blocks that sweep by 2400 mm³. Negative shown in the LAN player. The actual keeper, attachment, release, friction and loaded acceleration remain open. See [receipt](gravity-hold.json).

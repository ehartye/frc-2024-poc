# Ground-supported frame and rail service

The ground-supported configuration retires the robot-weight mast and its removable rail fasteners. Four welded aluminum sleeves stay in the parked frame. An 8 mm stepped bench gauge lets a second person check that the original bolts have cleared their bores. Remove the gauge before moving the chassis. [Architecture decision](../../../experiments/alliance-service/study.md) · [Source assembly and mast attachment measurements](../../assembly-readiness/mast-fit.json)

## Identify the parts

| Item | Parked R9 | Ground-supported selection |
| --- | --- | --- |
| Side rails and four flush rail sleeves | Four 6061 aluminum sleeves, 13 mm outside diameter, 8.5 mm bore, 50.8 mm long. Two stations per rail. | Retain each welded sleeve. Leave its bore open. |
| Crossmember and upper sleeves | Upper crush sleeves belong to the mast crossmember. | Retire the crossmember with its upper sleeves and the remaining mast assembly. |
| Four fastener stacks | MISUMI CB8-100 cap screw, M8 plain washer and Accu HPN-M8-C8-Z nut at each station. | Remove and account for all four sets. |
| Inspection gauge | One project-authored, loose bench tool in 6061 aluminum. | Use only during the stationary check. |

The gauge has a nominal 8 mm diameter stem, 54 mm from tip to shoulder, and a 12 mm diameter grip, 14 mm long. Measure its actual diameter, straightness and tip condition before using it. This gauge checks for gross obstruction. Its dimensions do not establish a precision bore tolerance. [Editable gauge STEP](rail-service-gauge.step) · [Gauge and register calculations](bench.py)

## Assembly and removal

1. Identify the original parked frame and both rails. Use the frame datum to locate X = 23 and 53 mm on each Y = −329 and +329 mm rail centerline. Label the four stations before starting. The frame work assumes a stationary chassis and an independently supported, unloaded mast.
2. For a new ground-supported build, retain the four rail sleeves and omit the retired assembly. For an existing assembly, keep its weight on an independent support while accounting for the four lower nuts, washers and cap screws. Verify the complete mast and upper crossmember are clear before inspecting the rails. The lift-out path for the assembled mast and any as-built attachments requires its own shop review.
3. At each exposed station, inspect the upper and lower openings and the welded sleeve. With access to both ends, offer the gauge straight down until its shoulder reaches the top face. Its tip should project 3.2 mm below the sleeve. Binding or an obstructed underside is a failed check.
4. Withdraw the gauge on the same axis. Have the second person record the station, whether the stop seated, whether the tip was visible below, and whether the tool came back out freely. Count the four retired fastener sets and the loose gauge before closing the job.

## Measured interface and mass

The STEP reread gives a gauge volume of 4,297.70 mm³. At one nominal station, a retained original bolt occupies 2,553.49 mm³ of its insertion volume through the sleeve. With the bolt removed the seated gauge has zero interference and 0.25 mm radial clearance. A 0.40 mm lateral offset produces 36.99 mm³ of interference. The same dimensions apply at all four stations. [Offline fit receipt](bench-fit.json)

The selection retires 31 of the 923 parked bodies. Four welded lower sleeves contribute 41.69 g in the native material model and stay installed. The retired solids sum to 4.186 kg of native material proxies. The sourced and estimated portion of the installed mass register drops by 2.536–2.679 kg, which leaves a known installed subtotal of **41.176–44.770 kg**. Seven retired accounting entries had unknown mass. The complete robot still needs an all-up measurement. [Source-bound list of retired articles](bench-fit.json) · [R9 installed mass limits](../../assembly-readiness/mass/result.json)

Removing a transverse member changes the frame load path. The original parked hull and nominal rail fit do not establish torsional stiffness with that member absent. Retain the ground-supported configuration as a CAD service study pending a structural check and physical inspection.

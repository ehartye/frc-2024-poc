# Finite guide geometry API and finite actuator coordinates

`finite_retention_guide.load_contract()` validates the reviewed guide input inventory and returns its provenance. No native CAD import is required for runtime queries.

`query_round_xz([x,z], pitch, contract, y_m=0)` takes SI metres/radians in the robot-longitudinal XZ frame of the robot-fixed guide (NOT the inertial world frame). Positive pitch is front-up (rotation about negative Y). It returns the guide point, outward normal, tangent, NOTE-center lever, penetration and the penetration Jacobian with respect to[x,z,pitch]. It supplies NO stiffness, friction or force. The support point is not an asserted pressure centroid.

In the bounded domain, the torus is a union of tube-radius spheres along its major ring. For front-up pitch and the frontmost ring center below the guide upper cap, all other ring centers are left/below it. The frontmost center minimizes distance to the vertical capsule representing the guide face/lower round. Finite side edges, top edge and mounting holes are conservatively screened using an enclosing plane-section patch. Deep contact, unsupported attitudes and uncertain finite boundaries return UNKNOWN. Separation is returned only from a conservative solid-distance bound.

At the planar face, the normal is -X, so contact can push the free piece rearward and produces positive pitch torque. At the lower round, the normal has a downward component: it can INCREASE vertical retention demand. This is not guaranteed self-indexing or lift assistance. The normal potential derivative matches the returned generalized moment arm; finite-difference tests cover both regions. A force integrator must retain stored energy when validity ends, not silently switch an active contact off.

The archived20-degree Boolean/material-witness disagreement returns UNKNOWN at that exact state. The entire API carries `nativeMaterialStatus=UNKNOWN_BOOLEAN_WITNESS_DISAGREEMENT`; an analytical candidate query is not physical/native qualification. Do not normalize that status away. Canonical robot geometry is unchanged.

`actuator-geometry-contract.json` separately binds current lift/pitch coordinates and source modules. The two generalized coordinates are pivot Z and front-up pitch, not NOTE pose. One12:1 NEO drives the nominal16.4mm lift sprocket; a carriage-mounted12:1 NEO drives the equal16T pitch chain. Moving mass/COM/inertia are explicitly null pending a proper assembly ledger. Couplings, retention, chain compliance, efficiencies and limits are not supplied by geometry.

## Moving-base frame contract

API points/normals/lever fields explicitly say RobotXZ. If the chassis translates in an inertial world, add its position to guide points and rotate vectors with its heading; use the actual guide-point velocity from that chassis state. A robot-fixed guide is not a world-fixed obstacle. Contact force reaction/work must be applied to the chassis, including nonzero residual chassis velocity. Do not set guide velocity to zero simply because this geometric API has no time dependence. The present planar query excludes chassis pitch/roll; yaw/world transforms belong to the coupled model.

# Original spring-biased complete cassette candidate

The bounded prototype moves one positive-Y belt cassette inward0..1.5 mm while preserving its belt, pulley, shaft, motor and transfer-drive registration. It does not change foam stiffness. It is an original editable interface layout, not an installed spring assembly or canonical robot revision.

## Load path and hardware
A rebuilt cassette bridge carries two bored sliding blocks on6 mm transverse rods at cradle X25/70,Z85 mm. Two compression-spring envelopes sit between the moving blocks and fixed outer seats. Inner collars and separate outer stop pins bound travel. The pivot-linked backbone is structurally separate from the moving cassette. Its shaft clamp/torque interface is deliberately unresolved; the clearance hole is not claimed to transmit torque. Actual bushings, spring wire/rate/free length, fasteners, rod retention and opening actuator still need selection and validation.

The installed spring length is7+u mm before seat adjustment. Spring force must be calculated from selected/calibrated installed springs and cassette dynamics, not equated automatically to NOTE normal force. Opening against the springs requires finite actuator force/work. Complete moving mass/inertia are unknown. This is only the positive-Y side; the other side has different pitch-chain packaging and must not be blindly mirrored.

## Actual checks and artifacts
The first two layouts are preserved as rejected studies. Their bridge/backbone and source-frame/drop-bracket contacts prompted an inboard bridge and revised guide stations. The final3poses x3travel-state audit retains only four inherited belt/pulley proxy-engagement pairs; no new mount or moving-part intersections occur in those samples. This is not a continuous swept-volume or load proof.

Original STEP: `artifacts/spring-cassette/original-one-side-candidate.step`, independently reopened28 roots. Source code and scalar evidence are tracked; no supplier native CAD is distributed.

## Full feed is a separate constraint
Exact private native material at nominal clocking is clear on the centered sampled path at zero travel. At0.75 mm inward stroke, pulley contacts appear at localNOTE X140/150 mm. At1.5 mm, contacts appear atX0 and130..160 mm, including small inherited-bearing contacts. The held10.567440 mm probe is clear at these sampled states, but that does NOT clear feed. These are rigid foam/metal contact witnesses, not safety verdicts.

Therefore the mechanism needs an explicit staged-open/force-handoff architecture, or a deliberately modeled compliant metal-contact path. It cannot remain closed through feed just because retention geometry fits. The opening actuator and support during release are NOT implemented. Existing±5 mm entry counterexamples and rotor-phase/tolerance uncertainty remain.

`spring-cassette-travel-contract.json` defines the u sign, spring length/energy relation, state/mass ownership, stop limits and unselected opening actuator. The runtime must evolve u/velocity and account for spring/actuator work; no instant gap reset or supplied preload is authorized by this geometry.

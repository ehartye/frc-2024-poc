# Ground custody at the R14 treads

The R15 external dock leaves 25.4 mm between the NOTE and the foremost parked body. Waiting there cannot establish contact with either tread. The revised Blue 3 sequence drives over the floor NOTE, seats it between the two registered treads, lifts it 8 mm for transport, lowers it at the pad and backs out with the treads free to turn. The second deposit in the clear case moves from 104.05 to **118.70 s**. The revised model completes both floor deposits in all five replays. [Inspect the contact section and per-NOTE history](https://ehartye.github.io/frc-2024-poc/ground-contact.html)

## What touches the NOTE?

The source is the parked 923-body R9 mesh. The 909-body R14 selection retains both treads, the cradle, the front cover and its fasteners. The native survey matches the source archive hash and every body name, identity and volume against the published bill. The two tread bodies are R7 534 and 535. The analysis separates the 34 connected shells in the adjoining cradle mesh and maps each back to a unique native body using its six extents. The full parent mesh provides a conservative obstacle set around the retained articles. [Source configuration and tessellation](../../docs/media/r9-assembly.json) · [R14 selection](../ground-relay/footprint.json) · [Registration check](register.py)

In robot coordinates, the original dock puts the ring center at X = 741.812 mm. The revised seat is at X = 241.5 mm, Y = 0 and Z = 25.4 mm before lifting. Blue 3 advances 500.312 mm along its own X axis while the NOTE rests on the carpet. At the two exchange pads it makes the reverse stroke only after restoring floor support. A 14-inch torus with 25.4 mm tube radius models the nominal NOTE. The contact screen bounds signed distance between each CAD triangle and the torus throughout the **continuous** axial stroke. Negative distance puts a mesh surface inside nominal foam. The contact distances below do not measure physical foam compression. [Contact bounds](contact.json) · [Distance calculation](contact.py)

| Pose or motion | Registered result |
|---|---|
| Centered floor entry and seat | Both tread surfaces enter the nominal torus by 1.378–1.393 mm. The closest underlying belt has 0.107–0.122 mm modeled clearance. The front pulley has at least 1.062 mm. |
| 3 mm lateral error | One tread clears by at least 1.607 mm. The opposite pulley enters the ring by approximately 1.8 mm. The control rejects the stroke. |
| Raise while fully seated | The carriage, cradle and NOTE translate together. Fixed mesh surfaces clear the ring by at least 3.75 mm over the 8 mm rise. |
| Withdraw while raised | The front cover enters the ring by at least 0.8 mm and its attachment group by at least 1.4 mm. The contact-aware control rejects the motion before release. |

The exported display mesh has 0.2 mm tessellation tolerance. The 0.107 mm lower margin at the belt backing is smaller. The nominal result therefore supports a control sequence, while clearance under manufacturing variation, actual tracking and assembly deflection remains open. The stroke check covers NOTE-to-robot geometry. Moving-joint clearances, motor tracking, unintended changes of pitch and interactions among the remaining field objects need their own evidence.

## Why the withdrawal mode matters

The force calculation integrates the archived penalty density of 10⁷ N/m³ over the torus surface against both registered tread profiles. At the 1024 × 1024 resolution, the total modeled seated normal load is 7.531 N. With the historical 0.6 belt friction assumption, sidewall friction supplies 4.407 N against a 2.366 N upper-mass NOTE weight. Across the loaded field steps, peak planar acceleration is 1.024 m/s². The smallest modeled margin is 0.632 N, governed by lateral restoring force evaluated at 0.1 mm displacement. The 8 mm seated lift allows 0.8 s in each direction. [Force calculation](loads.py) · [Resolution and step checks](loads.json) · [Earlier low-lift evidence](../../sim/pressure/evidence/retained-pickup-20261118/short-lift-verified.json)

On the floor, the model assumes a static carpet coefficient of 0.35. That gives **0.828 N** available to keep the ring still. Braked treads can demand **4.519 N** during withdrawal, so the receiver never gets credit for that NOTE. With unpowered rolling drag modeled at 0.035 times total normal load, the largest sampled demand falls to **0.324 N**, including the axial component of the tread contact normals. Halving the axial sample spacing from 0.25 to 0.125 mm gives the same result at the reported precision. The rolling demand is 0.325 N at the 512 × 512 grid. Real backdrive drag, carpet friction, foam stiffness, lift travel and the ability to center the NOTE remain unmeasured. Braking shows a loss of guaranteed floor restraint. It does not establish the subsequent slip distance.

## Full match result

The two tracked IDs are `center-2/1` and `center-1/1`. A Blue 3 deposit requires the same NOTE to seat, survive the loaded trip, regain floor support, stay inside the pad during withdrawal and clear the robot. Blue 2 can receive each ID once after the deposit. Its earlier fixture model supplies the receiver behavior. All ground exchanges earn zero match points. Each column counts events before the 150 s horn.

| Receiver delay / second NOTE offset | R15 B3 deposits | Revised B3 deposits | Revised second deposit, match s | B2 pickups | Downstream stores |
|---|---:|---:|---:|---:|---:|
| Ready / centered | 2 | 2 | 118.70 | 2 | 2 |
| 110 s / centered | 2 | 2 | 118.70 | 2 | 1 |
| 20 s / 3.5 in | 2 | 2 | 119.95 | 2 | 2 |
| 112 s / 3.0 in | 2 | 2 | 119.80 | 2 | 1 |
| 120 s / 0.8 in | 2 | 2 | 119.25 | 1 | 1 |

Blue 3 resets in all five. Both robots complete storage and reset in two. The minimum planar clearance is 0.239 in. The five cases were already evaluated in R15, so this comparison is a matched rerun. The original R15 frozen throughput receipts remain byte-identical. In the weakest case, the second NOTE remains on pad two. Repeating the earlier receiver staging and direct-storage change recovers that same NOTE at 148.20 s, still aboard Blue 2 at the horn. [Per-case event histories](results.json) · [Original frozen results](../ground-relay/buffer-results.json)

The revised reservation follows Blue 3's timed insertion, seating and retreat through the four archived traffic tracks. Reserving their enclosing rectangle for the whole interval leaves time for one pickup and zero deposits. Omitting the reservation encounters Blue 1. Empty stock and zero holding force earn zero deposits. Three peers cross the first source NOTE's projected location. Removing it after traffic passes leaves only the second identity deliverable. The successful cases assume local detection and exact lateral correction within the four-inch search band. These contacts and forces are offline model results.

## Reproduce

From the season checkout with Python and NumPy installed, run:

```sh
python -B experiments/registered-custody/contact.py
python -B experiments/registered-custody/loads.py
python -B experiments/registered-custody/rehearse.py
python -B experiments/registered-custody/lab.py
python -B tests/test_registered_custody.py
```

CI repeats the geometry, force screens, per-actor replay and published lab data. With the original native survey and mesh manifest, `register.py <survey-folder>` independently checks the source body mapping. The public path uses the pinned GLB, R14 selection and compact cradle bounds. [Control implementation](rehearse.py)

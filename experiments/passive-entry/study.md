# Can the ground relay accept an off-center NOTE?

R17 advances Blue 3's floor seat to X = 410 mm and adds two passive, tapered urethane rails to the existing cradle. A loose NOTE entering 6 mm off center stays supported through the modeled carry. Across five previously evaluated six-robot conditions, Blue 3 sets both named pieces down and resets five times. Blue 2 stores both and completes its last reset twice. This selects a candidate for a passive bench trial. [Inspect the cassette and replay](https://ehartye.github.io/frc-2024-poc/passive-entry.html) · [Public robot attachment audit](showcase-attachments.md)

| Compared on the original field and receiver clocks | R16 centered pinch | R17 passive pair |
|---|---|---|
| Entry | A 3 mm error loses the opposite tread and enters the front pulley by about 1.8 mm. Centered backing clearance is 0.107 mm against a 0.2 mm display tessellation. | A flared 14 mm OD pair takes the load ahead of the old pulleys. Nominal ±3 and ±6 mm residuals enter on the carpet with no centering constraint. |
| Hard interference during the matched role | Center must reach X = 241.5 mm. | At least 9.057 mm to the old backing, 7.544 mm to the old treads, 3.162 mm to the bare rear return and 6.7 mm to the bare front return. The tightest old group is a bumper attachment at 0.400 mm. |
| Handling and cost | 8 mm lift, 42 in/s² loaded acceleration limit. Clear second deposit at 118.70 s. | 6.6 mm lift, 18 in/s² loaded limit and a 4 in retreat before the southern lane. Clear second deposit at 126.30 s, 7.60 s later. Two removable cassettes add an estimated 215.58 g. |

## Made parts and retention

Mirror the following across the robot centerline. Coordinates are millimeters in the parked R9 frame used for the retained 909-body R14 selection. Six tapped holes and 16 added articles yield a proposed 925-body installed selection. The separately reopened STEP contains the 16 added articles. [Geometry parameters](part.json) · [STEP assembly](paired-entry.step) · [Article volumes and identities](part-receipt.json)

| Per side | Material and design | Where the load goes |
|---|---|---|
| One carrier | Machine a single 6061-T6 billet. The forward section spans X 275–505, Y 211.4–217.75, Z 4–15. A 36 mm long web joins the 3.175 mm top tongue. | Vertical and lateral loads go from the low beam through the web and top tongue to the existing side weldment 536 or 538. |
| One spine and one tread | Fixture-weld a 4 mm diameter 304 stainless spine with two transverse locating tangs. Centerline passes (350,181), (453,181), (500,193) at Z 8.5. Order a finished 70A PMC-770 urethane overmold with 14 mm OD and rounded ends. | The sleeve surrounds the spine and its end shoulders. Tang loads bear in two 4.10 mm carrier bores. Clamp the flats with two M3 × 6 A2 flat-point set screws, centered at Y 214.6. The tread and spine are serviced together. |
| Three cap screws | M3 × 8 A2 full-thread socket heads at X 280, 293 and 306, Y 207.6. Machine Ø3.4 clearance in the tongue. Register and tap the existing weldment M3 × 0.5. Nominal engagement is 4.825 mm. | The clamped face and three fasteners react the cantilever moment. Screw envelopes omit thread form and socket detail. Check actual host stock, engagement, end clearance and pullout before machining. |

The design uses the manufacturer's published screw dimensions, polyurethane density and hardness, and nominal stainless stiffness. Contact friction, bending of the joined tangs, foam compliance, sleeve adhesion and abrasion need measurement on the assembled part. [Accu M3 × 8](https://www.accu.co.uk/metric-cap-head-screws/3818-SSCF-M3-8-A2) · [Accu flat-point M3 × 6](https://www.accu.co.uk/flat-point-set-screws/4514-SSF-M3-6-A2) · [Smooth-On PMC-770 data](https://www.smooth-on.com/tb/files/PMC_770_TB.pdf) · [Outokumpu Core range](https://otke-cdn.outokumpu.com/-/media/files/products/core/outokumpu-core-range-datasheet.pdf)

The largest two-sided contact load was 3.893 N. Assigning it all to one rail gives 0.109 mm idealized wire and 0.197 mm carrier deflection. A 20 N comparison gives 0.56 mm and 1.012 mm respectively, with 119.37 MPa wire stress and 159.23 N incremental tension on the outer screw. The free-piece simulation uses a combined contact stiffness and a rigid carrier. These beam estimates show why the 0.400 mm hard margin needs a loaded measurement. They exclude impacts, joint compliance and preload loss. [Load path calculation](public.py) · [Load cases](load-path.json)

For Fusion handoff, insert the STEP in millimeters at the parked robot origin into a saved derivative of the R14 selection. Preserve all 16 roots. Register the mounting tongues to the two original weldment faces at Z 38.1, mark and model the six tapped holes in that derivative, then reopen it and inspect the complete assembly. The authored dimensions remain editable in the [shape generator](parts.py). Each imported STEP body is a solid. The integrated native edit and reopened assembly check are pending. The [visual audit](showcase-attachments.md) identifies what the student actually sees in each existing robot view.

## Loose-piece replay

The model integrates a free 0.2413 kg torus on carpet, with translation and rotation driven by contact and gravity. It samples the major ring at 96 angles and integrates every millisecond between the 50 ms robot frames. A compliant normal law and tangential springs limited by Coulomb friction act on the two finite rails and the carpet. The nominal combined stiffness is 0.12 N/mm², tread friction 0.60 and carpet friction 0.35. The original coarse sensing gate remains. The specified small entry residuals then evolve freely. On withdrawal Blue 3 lowers the NOTE onto carpet and moves away. Blue 2 plans from each measured resting center in a second pass. Its pickup and store remain timed receiver fixtures. [Contact dynamics](physics.mjs) · [Field clocks](clocks.py) · [Full piece histories](field.json)

| Condition | Initial errors, first/second (mm) | Second settled (s) | Custody at 150 s, first/second | Blue 2 reset |
|---|---:|---:|---|---|
| Ready | −3 / +3 | 126.30 | Stored / stored | Yes |
| Late | +3 / −3 | 126.30 | Stored / aboard Blue 2 | No |
| Offset source | −6 / +6 | 127.55 | Stored / stored | Yes |
| Late and offset | +6 / −6 | 127.40 | Stored / aboard Blue 2 | No |
| Slow receiver | +3 / −6 | 126.75 | Stored / second pad | No |

First NOTE is always `center-2/1`, second is `center-1/1`. The first settles at 84.80 s in all five. In the slow case direct dispatch recovers the second at 148.20 s and ends with it still aboard. The 0, 110, 20, 112 and 120 s receiver gates and original coarse source offsets are preserved. Removing the 4 in route setback stops the loaded carrier at 64.05 s against the other floor NOTE, producing a 10.276 m/s² discontinuity. This rejected control is retained. [Matched outcomes](field.json) · [Lane control](lane-control.json)

## Service and limits

Park and unload. Remove the three carrier screws with a 6.5 mm diameter tool envelope and 80 mm access stroke. Raise one cassette 17 mm, shift it outboard 2 mm, move it forward 70 mm, lower 5 mm, then slide forward another 210 mm. The installed, lifted, tool and extraction envelopes clear other bodies in the registered parked mesh. The six pilot-hole rays each meet solid host stock from Z 12.7 to 38.1. A simple 3 mm rise and forward pull catches the host, and a straight upward exit intersects the front frame and cover. Full wheel rotation gives a 4.641 mm installed bound and a 2.761 mm extraction bound. A 35 mm outboard shift intersects an actual tire vertex at zero steering angle. [Service and wheel witnesses](service.json)

Across the nominal field cases the lowest loaded point clears carpet by 0.693 mm and the highest point reaches 56.870 mm. The hard screen encloses center X 405–752, |Y| ≤ 6.5 and top ≤ 57.0 mm. The ready-case second NOTE's minimum floor gap is 0.693, 0.679, 1.172 and 1.648 mm as time step and angular sampling refine. The nominal carry outcome repeats, but the submillimeter clearance has not converged. With 0.5 mm wider and softer treads the NOTE touches carpet. Narrower, stiffer treads exceed the overhead screen at 57.881 mm. Raising carpet friction to 0.55 also returns it to the floor. At tread friction 0.02 it misses the second pad by about 5 m. [Material and numerical controls](sensitivity.json)

Three bench observations can discriminate these cases: measure paired normal load against compression and lateral displacement with the actual NOTE; measure carpet drag and wet/worn tread slip through pickup and release; then measure loaded rail sag, NOTE floor gap and upper clearance at both width limits. Capture the original piece ID, floor settlement and receiver reset on the same clock. Physical retention and fit remain open.

# Detailed-design handoff from Guided cassette 1.0

This is what the Fusion work starts from. Guided cassette 1.0 is the effective preliminary design.

Every number names its source file. Paths are relative to the repository root. `EV/` means `2024/preliminary-review-lan/greenfield-retained/`. `REL/` means `cad/preliminary-design/releases/1.0/`. `BUILDER/` means `cad/preliminary-design/builder/`.

Detailed design may publish as in-progress when three things hold: it links the effective preliminary release, the STEP exported from the registered Fusion design passes the layout check on this release's motion cases, and a mission animation plays on the detailed geometry.

## 1. What to link

| Item | Value | Source |
| --- | --- | --- |
| Preliminary release | Guided cassette `1.0`, issue `2024-guided-cassette-1.0` | `REL/layout.json` |
| Layout solids | `REL/robot.step`, 232 solids, revision A | `REL/step-revision.json` |
| SHA-256 of the layout solids | `2914b6f0b94eda5c0fa18f411c58f933ce0502527997c9e2baf06476a0af9343` | `REL/step-revision.json`, `REL/layout.json` |
| Parent solids (draft layout) | `EV/guided-hoist-01/service-locked-robot.step`, SHA-256 `a29a146d8897f64e8f056fcb46a431ba234c7aecccbc3233b6834c43c703bc88` | `BUILDER/revise_step.py` |
| Receipt | `REL/receipt.json`, pending the whole-cycle check | — |

Confirm the SHA-256 against `REL/layout.json` on the day you link. The layout was still being rebuilt when this page was written.

Guided cassette 1.0 is a revised layout. The draft layout is the 232-solid layout the design was drawn on while it was in progress. Revision A is the one geometry change from it: the launch belt motors moved (section 6). The 144 double-encoded component names were also repaired.

## 2. Datums and robot frame

Robot frame: millimetres, +X forward, +Y left, +Z up from the carpet. The STEP is drawn parked. The layout datum is the identity transform (`REL/layout.json`, `datum`).

| Datum | Value | Source |
| --- | --- | --- |
| Pivot axis | Parallel to Y through (185, 0, 501) at park | `BUILDER/kin.py` |
| Cassette angle, parked | 125° | `BUILDER/kin.py` |
| Cassette angle, launch, released and checked | 122° | `BUILDER/timeline.py`, `REL/operating-point.json` |
| Cassette angle, chain capture | 75.386° | `EV/endgame-route-01/guided-stroke.py` |
| Cassette angle, service index | −40° | `BUILDER/build.py` |
| Cassette angle, floor pickup | −50° layout pose | `EV/pickup-lip/layout.json` |
| Pivot truck | Slides along X. Scheduled pivot X runs from −30.7 mm to 185.0 mm | `EV/follower-01/stroke.py`, `EV/pickup-lip/layout.json` (`minSlideX`) |
| Pivot X at pickup | 169.36 mm | `EV/pickup-lip/layout.json` (`pickPivot`) |
| Pivot X at the service index | 98.37 mm | `EV/follower-01/stroke.py` evaluated at −40° |
| NOTE seat | (215, 0, 50.4) in the cassette frame | `BUILDER/kin.py` |
| Cover leaf hinge | Parallel to Y through (457.5, 0, 96) in the cassette frame | `BUILDER/kin.py` |
| Cover leaf cam | Opens the leaf from 102° to 120° of cassette angle | `BUILDER/build.py` |
| Hook carriage | Draws along cassette −X. 229.3 mm reached in the forward climb | `EV/climb-brackets-04/results.json` (`peakDraw`) |
| Cable drums | 29.5 mm radius on the pivot axis | `EV/guided-hoist-01/budget.py` |

The cassette frame is the robot frame rotated by minus the cassette angle about Y and placed at the pivot.

## 3. Envelopes

| Envelope | Value | Source |
| --- | --- | --- |
| Frame | 680 mm long, 660 mm wide, 510 mm track | `REL/design.json` |
| Bumpers | 82 mm deep, 124 mm tall, bottom edge 52 mm above the carpet | `REL/design.json` |
| Parked height | 1,040.3 mm | `REL/design.json` |
| Furthest forward point, pickup deployed | 640.0 mm sampled, 642.3 mm bounded between samples, against a 644.8 mm project limit | `EV/pickup-lip/layout.json` |
| Blade tip, pickup deployed | X = 619.05 mm, 3 mm above the carpet in the layout pose | `EV/pickup-lip/layout.json` |
| Inspection mass | 56.270 kg upper estimate against 56.699 kg | `REL/gross-power.json`, `REL/limits.json` |

The front reach has 2.5 mm to its limit. Do not add anything forward of the blade.

Not recorded as one number in this release: the largest height and the largest reach during chain capture and hoist. Measure both on the first Fusion assembly. Verify the size rules and their edition in the season project before relying on a limit.

## 4. Interfaces and allowances

The layout check uses zero clearance between unrelated solids. Three kinds of contact are allowed.

**NOTE contact.** The NOTE may indent these surfaces one-sided, up to the limit. They are planning assumptions. No measured force or allowable is claimed. Source: `cad/preliminary-design/compression-limits-002.json`.

| Surface | Limit |
| --- | --- |
| Centre intake belt | 8 mm |
| Feed tread, left and right | 7 mm |
| Launch tread, left and right | 7 mm |
| Rear stop pad | 3 mm |
| NOTE runner, left and right | 2 mm |
| Windowed keeper panel | 1 mm |
| Replaceable pickup lip | 1 mm |

**Designed joints.** Fourteen pairs touch by design. Each is qualified one case at a time. Source: `BUILDER/build.py` (`JOINTS`).

- Pivot torque shaft in the left bearing, the right bearing and the reducer bore.
- Cover leaf on its axle.
- Slide screw in its travel nut.
- Pivot clutch on its drive socket.
- Declutched pivot shaft in the left cheek, the right cheek and the service index disk.
- Keeper rack on its return pinion.
- Hoist line leaving each cable drum, left and right.
- Hoist line over each rear redirect sheave, left and right.

**Hoist line take-up.** Each 3 mm line is drawn at its parked length. As the carriage draws in, the drawn span runs into the splice allowance. The allowed overlap is the line section times the take-up, plus 0.5 mm³. Source: `BUILDER/build.py` (`LINES`).

**Mounts.** `REL/layout.json` lists one mount for each mechanism member under `mounts`: the host solid, a point on the host in the parked frame and the measured separation. The check compares a point only with its host's bounding box. Seating, hole alignment and strength need their own checks.

Eleven members touch no other solid in the parked layout, so their connection is not drawn. Source: `REL/layout.json` (`mounts`, `basis`).

| Member | Host | Gap |
| --- | --- | --- |
| Battery service pin | Service pin guide | 0.175 mm |
| Hoist thimble pin, left and right | Hoist eye thimble | 0.050 mm |
| Capture-angle dog bushing | Catch sector | 0.010 mm |
| Hoist rear redirect axle, left and right | Hoist redirect ear | 0.050 mm |
| Carpet follower, left and right | Cassette side beam extension | 0.300 mm |
| Catch dog | Catch receiver | 0.200 mm |
| Slide stop pin, left and right | Slide stop guide | 0.150 mm |

**Names.** Cases name solids by their STEP product names. The detailed STEP must keep those names or supply a mapping.

**Not in the layout solids, so not checked:** the removable slide-motor cartridge (`EV/mounted-slide-02/`), the battery-service slide link and its two shoulder screws (`EV/guided-hoist-01/removable-truck-lock.step`), the working stage chain and the field. The battery lift at the service index is not a checked case.

## 5. Operating point

All figures in this section come from `EV/scoring-window-04/summary.json` unless another file is named. `REL/operating-point.json` carries the released point. "Perturbed" means the 34 carried NOTES of starts R01 to R07. "Nominal" means the 12 carried NOTES of the nominal route. "Spanning" means six perturbed NOTES chosen to span the population (`EV/scoring-window-04/later.py`). "Delivered" means the whole NOTE passed through the opening. "Counted" means both speaker beams were shadowed within 7 s.

| Setting | Released | Cycle trials |
| --- | --- | --- |
| Robot centre from the alliance wall | 1.39 m | 1.39 m |
| Firing lane | y = 5.821 m | y = 5.871 m |
| Heading | 0° | 0° |
| Cassette launch angle | 122° | 122° |
| Launcher command | 6.88 m/s | 6.88 m/s |

Only the lane moved. The single-shot campaign re-poses the robot and launcher for other lanes and angles. It does not re-run the lift.

At the released point all 34 perturbed and all 12 nominal NOTES are delivered. The smallest opening margin is 56.7 mm. At the trial lane it was 17.6 mm.

### Launch angle at the released lane

| Angle | Delivered | Smallest margin | Perturbed counted | Nominal counted |
| --- | --- | --- | --- | --- |
| 120° | 13 of 34 | 1.6 mm | 13 | — |
| 121° | 34 of 34 | 3.4 mm | 32 | — |
| **122°, released** | 34 of 34 | 56.7 mm | 31 | 12 of 12 |
| 123° | 34 of 34 | 67.7 mm | 33 | 9 of 12 |
| 124° | 34 of 34 | 39.8 mm | 33 | — |
| 125° | 6 of 6 spanning | 26.0 mm | 6 | — |
| 126° | 4 of 6 spanning | — | 4 | — |

The window is 121° to 125°. It fails at 120° and at 126°. The released 122° is 1° above a thin low edge: 121° delivers with 3.4 mm. Draw the launch hard stop at 122° and hold it within bench test B5.

**Alternative, not the setpoint: 123°.** It is the centre of the window and has the most opening margin. It was not released because counting is the mission outcome, and the nominal NOTES count 9 of 12 at 123° against 12 of 12 at 122°. If a real speaker shows no such difference, 123° is the better stop. Its window is recorded below.

### Window about the released point, one input changed at a time

All 34 perturbed NOTES at each value.

| Input | Delivers every shot | Fails | Smallest margin |
| --- | --- | --- | --- |
| Heading | −3° to +4° | −4° (3 of 34 delivered) | 11.3 mm at −3°, 46.8 mm at +4° |
| Lane y | 5.796 m to 5.871 m | 5.771 m and 5.721 m (33 of 34); 5.921 m and beyond (0 of 34) | 17.6 mm at 5.871 m |
| Station x | 1.39 m to 1.45 m | 1.49 m (18 of 34) | 32.2 mm at 1.42 m, 7.6 mm at 1.45 m |
| Launch angle | 121° to 125° | 120° and 126° | See the table above |

So from the released point the robot may err 25 mm toward the centre of the opening, 50 mm toward the side wall, 60 mm away from the alliance wall and 3° in heading, one at a time.

Launcher command, fired at the trial lane only: every perturbed NOTE is delivered from 6.56 to 7.68 m/s. Nominal NOTES are delivered down to 6.08 m/s (8.4 mm margin); 5.92 m/s loses 1 of 6; 5.60, 5.20 and 3.44 m/s lose all 6.

Limits of this window:

- At 1.39 m the rear bumper is 35 mm from the subwoofer face (`EV/scoring-window-04/campaign.py`). Stations nearer the wall were not fired.
- Errors do not simply add. 20 mm toward the side wall with −3° heading let all 34 NOTES through the opening with no margin, and one was counted. With −2° and the same 20 mm, all 12 nominal NOTES were delivered and counted. Source: `EV/scoring-window-04/results/aim-search.jsonl`.

### Window about the 123° alternative

Six spanning NOTES at each value. Source rows: `EV/scoring-window-04/results/angle-123-window.jsonl`.

| Input | Delivers | Fails | Smallest margin |
| --- | --- | --- | --- |
| Heading | −3° to +4° | −4° (1 of 6) | 10.5 mm at −3° |
| Lane y | 5.771 m to 5.871 m | 5.921 m (0 of 6) | 17.7 mm at 5.871 m |
| Station x | 1.39 m to 1.49 m | Not found | 28.5 mm at 1.49 m |
| Launcher command | 6.56 and 7.20 m/s | Not found | 67.4 mm |

### Failing brackets at the trial lane

These are why the lane moved. At y = 5.871 m, +50 mm of lateral error loses every shot. So does −2° of heading. −1° of heading delivers 31 of 34 with 0.4 mm.

### Counting, and the NOTES that bounce out

At the released point 31 of 34 perturbed NOTES are counted within 7 s. The median count is 4.09 s after the shot starts and the latest is 5.20 s. All 12 nominal NOTES are counted.

The other three perturbed NOTES bounce out. They were watched for 20 s. Each enters 0.54 to 0.63 m past the opening, returns through the opening within 3 s without shadowing either beam and comes to rest on the carpet in front of the speaker. None counts late. None stays in the speaker. None leaves by the counter's exit. The one uncounted perturbed NOTE at 123° does the same, 0.71 m in. Sources: `EV/scoring-window-04/results/released-20s.jsonl`, `EV/scoring-window-04/results/angle-123-20s.jsonl`, full records in `EV/scoring-window-04/shots/`.

**The release criterion, wholly through the opening and eventually counted, is not met for those three NOTES.** This is an open finding for the owner, carried with bench tests B1 and B2.

What separates the bounce-outs:

- Nothing recorded at release. Launch speed, elevation, lateral velocity, peak height and opening margin all lie inside the range of the counted NOTES.
- The carry. The three are R02-inner, R03-center-mid and R03-outer. Across the campaign those carries bounce out in 15 to 18 percent of their deliveries. Five other carries never do. So the cause travels with the incoming NOTE state.
- The contacts. A NOTE that bounces out touches none of the counter parts that every counted NOTE touches.
- Not examined: NOTE attitude and spin at the opening. The campaign rows do not hold them.

Across the campaign the count rate did not move with launcher speed or with the speaker's counter drive speed (0.55 to 1.0 of full). Speaker surface friction was not varied. What a NOTE does inside the speaker is a property of the modelled speaker interior. A returning NOTE also lands on or beside the robot (open risk 5).

## 6. Motor relocation, revision A

Each launch belt motor can is 36 mm in diameter on a centreline 195.8 mm from the cassette centre. Its inner face is 177.8 mm out, which is the NOTE outer radius. The floor NOTE riding over the pickup lip passed the cans with 0.0004 mm. Sources: `BUILDER/revise_step.py`, `BUILDER/risks.py`.

The revision:

- Raises both motors 60 mm along the cassette normal, away from the deck. In the parked frame that is 49.1 mm rearward and 34.4 mm down. Source: `REL/step-revision.json` (bounds before and after).
- Lengthens each drive shaft by the same 60 mm. Shaft volume goes from 4,398 mm³ to 9,111 mm³ each. Source: `REL/step-revision.json`.
- Leaves the motor inner faces at Y = ±177.8 mm. The clearance comes from height above the deck, not from width.
- Changes no other solid.

After the revision the checked cycle clears the motors by more than 2.8 mm (15 mm sampled). Source: `BUILDER/risks.py`.

Still to do in Fusion:

- Draw the taller motor mount. It is not in the layout.
- Check the lengthened shaft for whip and for bearing support.
- The pickup trial does not model the motors. A NOTE picked up off centre meeting the shaft is untested (bench test B4).
- The raise adds 0.074 kg of shaft, which the mass ledger includes, and about 1 percent to the lifted moment, which the voltage budget does not include (section 7). Source: `BUILDER/risks.py`, `BUILDER/ledgers.py`.

## 7. Budgets to stay inside

| Budget | Figure | Margin | Source |
| --- | --- | --- | --- |
| Inspection mass, upper estimate | 56.270 kg against 56.699 kg | 0.429 kg, with a 3 to 6 kg unallocated reserve inside the estimate | `REL/gross-power.json` |
| Loaded lift, weak pack, other loads staged to 25 A | 8.705 V bus | 0.448 V at the pivot motor | `REL/gross-power.json` (`loaded25A`) |
| Loaded lift, weak pack, 39 A of other loads | 8.168 V bus | −0.089 V: fails | `REL/gross-power.json` (`negative39A`) |
| Feed and launch, weak pack | 8.344 V against 8.0 V | 0.344 V | `EV/scoring-window-04/summary.json` (`lowestBusV`) |

Weak pack: 11.8 V open circuit, 27 mΩ (`BUILDER/risks.py`). Reopen preliminary design if detailing consumes a margin.

The builder regenerates `REL/budgets.json` and `REL/limits.json` for Guided cassette 1.0. `BUILDER/ledgers.py` starts from the mass lines of the draft layout budget, which sum to the 56.270 kg of `EV/mounted-slide-02/gross-power.json`, a study made before revision A. It then adds the volume the revision added to each changed solid, at steel density, to the line that holds it. The lengthened shafts add 0.074 kg (9,425 mm³ from `REL/step-revision.json` at 7.85 g/cm³), so the upper inspection estimate is 56.344 kg and the headroom 0.355 kg. The taller motor mounts are not drawn and not estimated. Also, the lift voltage margin does not include the roughly 1 percent larger lifted moment. Charge the mass to the unallocated reserve until the study is rerun.

## 8. Brackets found by trial

| Mechanism | Passes | Fails | Source |
| --- | --- | --- | --- |
| Pickup, blade tip height above carpet | 1.5 to 10 mm (5 mm nominal) | 12 mm and above | `EV/pickup-brackets-04/results.json` |
| Pickup, NOTE lateral offset | 0 and ±50 mm | ±100 mm and beyond | `EV/pickup-brackets-04/results.json` |
| Pickup, nose roller and pad grip | 0.30 to 0.75 (0.45 assumed) | 0.20 and below | `EV/pickup-brackets-04/results.json` |
| Pickup, nose motor phase current limit | 1 to 8 A (8 A nominal) | 0.5 A and below | `EV/pickup-brackets-04/results.json` |
| Retention, lift duration | 0.3 to 4.8 s: NOTE stays seated | None found | `EV/retention-brackets-04/results.json` |
| Retention, chassis acceleration | 2.6 to 90 m/s²: NOTE stays seated | None found | `EV/retention-brackets-04/results.json` |
| Cover leaf overtravel past its cam stop | 0.13° at 4.8 s; 0.75° at 1.8 s | Above 1° for lifts of 1.4 s and shorter | `EV/retention-brackets-04/results.json` |
| Climb, centre of gravity ahead of nominal | +40 mm (11.5 mm floor clearance), +50 mm (10.9 mm) | +60 mm (8.4 mm against a 10 mm allowance) | `EV/climb-brackets-04/results.json` |
| Climb, centre of gravity behind nominal | −40, −80, −120 mm | None found | `EV/climb-brackets-04/results.json` |
| Climb, hoist ramp | 6.5 s | 5 s (6.9 mm clearance); 4 s and shorter do not latch | `EV/climb-brackets-04/results.json` |
| Climb, catch disabled | — | Robot returns to the carpet | `EV/climb-brackets-04/results.json` |
| Shot, launcher at half command (3.44 m/s) | — | Does not deliver | `EV/scoring-window-04/results/low-speed.jsonl` |

The climb passes at +50 mm with 0.9 mm to spare. Hold the centre of gravity. The ramp brackets were run with the centre of gravity 40 mm ahead only.

## 9. Bench tests carried forward

`REL/missions.json` lists each open risk with its measure, pass value and fail value. The register source is `BUILDER/risks.py`. Results are not needed to start detailing. They are needed before any repeatability or strength claim. `REL/RISK-REVIEW.md` records how the register was reviewed.

### Scoring and aim

| # | Measure | Pass | Fail |
| --- | --- | --- | --- |
| B1 | Starts out of 20 on a full field speaker that score six floor NOTES within 120 s | 19 | 18 |
| B2 | Delivered NOTES out of 20 counted by a real speaker within 7 s of the shot | 20 | 18 |
| B3 | Largest heading error at the firing station over 20 autonomous arrivals | 2.0° | 3.0° |
| B3a | Largest lateral error at the firing station over 20 arrivals | 20 mm | 50 mm |
| B3b | Largest fore-aft error at the firing station over 20 arrivals | 30 mm | 100 mm |
| B4 | Smallest gap between a NOTE picked up 50 mm off centre and either launch motor or shaft | 3.0 mm | 0.0 mm |
| B5 | Largest departure of the cassette launch angle from 122° over 20 lifts to the stop | 1.0° | 2.0° |
| B6 | Launch belt surface speed at feed on a weak pack, for a 6.88 m/s command | 6.56 m/s | 5.92 m/s |
| B7 | Lowest bus voltage during a shot on a pack at 11.8 V open circuit | 8.0 V | 7.9 V |

B2 in simulation: 31 of 34 perturbed NOTES at the released point; the other three bounce out. Six in a row at that rate is about 57 percent, so B1 is not supported by simulation and a real speaker decides it. B3 fails at 3° because errors combine (section 5); alone, 3° delivers. B5 fails at 2° because 120° loses 21 of 34 NOTES; at 1° low the margin is already 3.4 mm.

### Pickup and carry

| # | Measure | Pass | Fail |
| --- | --- | --- | --- |
| B8 | Blade tip height above competition carpet, cassette deployed, robot loaded | 10 mm | 12 mm |
| B9 | Floor pickups out of 20 that seat within 2.82 s with the NOTE 50 mm off centre | 19 | 18 |
| B10 | Grip coefficient of the nose tread on a competition NOTE | 0.3 | 0.2 |
| B11 | Seated NOTES out of 20 confirmed by the seat sensor within 0.2 s | 20 | 19 |
| B12 | Cover leaf overtravel past its cam stop during a 4.8 s loaded lift | 1.0° | 2.0° |
| B13 | One-sided NOTE indentation under the centre intake belt at the seat | 8.0 mm | 8.1 mm |
| B14 | One-sided NOTE indentation on either runner, NOTE seated | 2.0 mm | 2.1 mm |
| B15 | Smallest gap, nominal NOTE to either entry bearing bracket through discharge | 1.0 mm | 0.0 mm |
| B16 | Smallest gap, NOTE to the slide motor package through pickup and lift | 1.0 mm | 0.0 mm |
| B17 | Deepest entry into the control-board envelope of a NOTE dropped from the speaker opening, guard fitted | 0 mm | 1 mm |

### Power and mass

| # | Measure | Pass | Fail |
| --- | --- | --- | --- |
| B18 | Battery current of all loads other than drive, pivot and slide during the loaded lift | 25 A | 39 A |
| B19 | Weighed inspection mass without battery and bumpers | 56.69 kg | 56.70 kg |

### Climb, brake and hold

The climb does not rely on motor power to hold. With the catch disabled the robot returns to the carpet (`EV/climb-brackets-04/results.json`, `no-catch`). So the catch, the slide stop pins and the pitch lock are the brake. Trial loads to size them against, from `EV/climb-brackets-04/results.json`: pitch lock peak up to 276 N·m; combined line tension up to 735 N in the passing cases; modelled robot mass 71.76 kg.

| # | Measure | Pass | Fail |
| --- | --- | --- | --- |
| B20 | Centre of gravity ahead of nominal, robot at gross mass | 50 mm | 60 mm |
| B21 | Lowest tire or bumper clearance during a 5 s disabled hold after a 6.5 s hoist ramp | 10 mm | 9 mm |
| B22 | Chassis drop during a 5 s disabled hold on the stops at 1.5 times robot weight | 0 mm | 1 mm |
| B23 | Proof load held by each terminated hoist line without slip or damage | 5.73 kN | 5.7 kN |
| B24 | Largest slack in either hoist line through full carriage travel, take-up fitted | 5 mm | 10 mm |

### Service

| # | Measure | Pass | Fail |
| --- | --- | --- | --- |
| B25 | Horizontal load held by the fitted slide link without movement | 1.29 kN | 1.0 kN |
| B26 | Smallest gap around the battery through a straight lift at the service index, leads disconnected | 5 mm | 0 mm |
| B27 | Smallest gap around the slide motor cartridge through its full withdrawal | 3 mm | 0 mm |
| B28 | Largest reach beyond the frame perimeter through capture and hoist, on the first Fusion assembly | 300 mm | 305 mm |

## 10. Open risks to carry into detailing

1. **Scoring and counting.** Every NOTE is delivered. Three of 34 perturbed NOTES bounce back out of the speaker uncounted. The literal criterion is not met for them: an open finding for the owner. Bench tests B1 and B2.
2. **Chassis tracking.** The firing window is about ±20 mm and ±2° together. Tracking is unmeasured. Bench tests B3, B3a, B3b.
3. **Launch angle.** The released 122° is 1° above a thin low edge. 123° has more margin but counts fewer nominal NOTES in simulation (section 5). Bench test B5.
4. **Launch motor mount and shaft.** Not drawn (section 6).
5. **Returning NOTE.** In start R02 a NOTE left the speaker and entered the roboRIO envelope by up to 25.0 mm (`EV/shared-counter-03/return-to-robot.json`). No guard is drawn.
6. **Thin clearances.** Entry bearing bracket 0.035 mm sampled; slide motor package 0.189 mm guaranteed. Sources: `cad/preliminary-design/motion-evidence-002/exit-bracket-clearance.json`, `EV/mounted-slide-02/free-piece-clearance.json`.
7. **The voltage budget does not include revision A.** Mass headroom is 0.355 kg with the lengthened shafts and without the undrawn motor mounts. Lift voltage margin is 0.448 V without the motor raise (section 7).
8. **Climb margin.** 0.9 mm at the forward centre-of-gravity limit. Catch, stop pins and pitch lock are ideal constraints in the trial.
9. **Battery lift.** Not a checked case.
10. **Assumed contact.** Every NOTE indentation limit, the grip coefficient and the speaker interior are assumptions.

## 11. First work items

1. **Launch motor mounts.** Draw the taller mounts and the lengthened shafts of revision A.
2. **Launch hard stop.** Draw it at 122° (section 5).
3. **Profile A blade seat.** Detail the pickup crossbar and the nose-roller bearing supports around the blade's four-screw seat. Keep the blade removable and the runner height adjustable.
4. **Entry bearing brackets.** Relieve both so a nominal NOTE clears by at least 1 mm through discharge.
5. **Control-board guard.** Add a guard over the electrical bay. Charge it to the unallocated reserve.
6. **Cover leaf.** Draw the leaf in its cam position at park.
7. **Undrawn connections.** Draw the eleven connections of section 4.
8. **Slide motor cartridge and service link.** Install both and check their withdrawal paths.
9. **Seat sensor.** Select and mount the seat beam.

## 12. The Fusion blade component

| Item | Value |
| --- | --- |
| Archive | `iterations/002/design/cad/guided-cassette/profile-a/profile-a.f3d` |
| Exported STEP | `profile-a.step` beside it |
| Stock | UHMW polyethylene, 434 mm wide, four M3 countersunk bores at Y = −165, −55, +55 and +165 mm |
| Coordinates | Drawn deployed. `geometry-check.json` beside it gives the transform from deployed blade to parked assembly |
| Fit | 2.335 mm to the nose roller (`EV/pickup-lip/layout.json`, `noseGapMm`) |

The blade is the first component of a guided-cassette Fusion design. No whole-robot Fusion design of this architecture exists yet. `iterations/002/design/cad/fusion-authority.json` still names the earlier side-finger archive. Reissue it for the guided cassette when the first assembly is saved.

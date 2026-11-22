# Detailed-design handoff from preliminary design 004

This package is what the Fusion work starts from once release 004 is marked effective. Until then it is a draft: sections marked *pending* fill in as the release firms up. The inputs it cites are in this folder and are pinned by `layout.json` and `receipt.json`.

Detailed design may publish as in-progress when three things hold: it links the effective preliminary release, the STEP exported from the registered Fusion design passes the layout check on this release's motion cases, and a mission animation plays on the detailed geometry.

## 1. What to link

| Item | Value |
| --- | --- |
| Preliminary release | `004`, issue `2024-004-guided-cassette` |
| Layout solids | `robot.step`, 232 solids, SHA-256 `91ac459ba6f4f3116fc5da3bf25d54c9e58ee19e214e0dba8cdc076960e50b74` |
| Same geometry, earlier names | `guided-hoist-01/service-locked-robot.step`, SHA-256 `a29a146d…bc88` (release 003) |
| Receipt and release digests | *pending* the issued receipt |

Release 004 changes no geometry. It repairs 144 component names in which the middle dot was encoded twice.

## 2. Datums

Robot frame: millimetres, +X forward, +Y left, +Z up from the carpet. The STEP is drawn parked.

| Datum | Value | Source |
| --- | --- | --- |
| Pivot axis | Parallel to Y through (185, 0, 501) at park | `build-004/kin.py` |
| Cassette angle | 125° parked, 122° launch, −49.745° floor pickup, 75.386° chain capture, −40° service index | Trial traces |
| Pivot truck | Slides along X. Pivot X runs from −30.7 to 185.8 mm; 169.36 mm at pickup, 98.37 mm at the service index | `follower-01/stroke.py` |
| NOTE seat | (215, 0, 50.4) in the cassette frame | `kin.py` |
| Cover leaf hinge | Parallel to Y through (457.5, 0, 96) in the cassette frame; cam opens the leaf from 102° to 120° | `empty_return.py` |
| Hook carriage | Draws along cassette −X, 230 mm stroke; 229.3 mm reached in the forward-CG climb | `climb-gross-front.json` |
| Cable drums | 29.5 mm radius on the pivot axis | `publish-review.py` |
| Firing station | Robot centre at field (1390, 5871) mm, heading 0 | `counted-reset-power-02/plan.json` |

The cassette frame is the robot frame rotated by minus the cassette angle about Y and placed at the pivot.

## 3. Mounting points

`layout.json` lists one mount for each mechanism member under `mounts`: the host solid, a point on the host in the parked frame and the measured separation. Validate each on the first Fusion issue. The check compares a point only with its host's bounding box; seating, hole alignment and strength need their own checks.

Eleven members touch no other solid in the parked layout, so their connection is not yet drawn: the battery service pin, both hoist thimble pins, the capture-angle dog bushing, both hoist rear redirect axles, both carpet followers, the catch dog and both slide stop pins. Each is registered against its nearest host with the clearance stated.

## 4. Cases the detailed model must pass

`motion.json` holds the cases in mission order. *Pending* the final count and the issued receipt.

| Stage | Cases | What moves | Path source |
| --- | --- | --- | --- |
| Reset | `reset-NN` | Cassette, truck, cover leaf | Commanded 4.8 s return; leaf from the empty-return trial |
| Acquire | `acquire-NN` | NOTE, cover leaf | Recorded free NOTE, fine nominal pickup |
| Retain | `lift-NN` | Cassette, truck, leaf, NOTE | Commanded 4.8 s lift; recorded NOTE and leaf |
| Transfer | `feed-NN` | NOTE | Recorded free NOTE, fine nominal shot |
| Deliver | `flight-NN` | NOTE | Recorded free flight to full passage of the opening |
| Park | `stow` | Cassette | 122° to 125° |
| Endgame | `approach-NN`, `capture-NN`, `hoist-NN`, `hold-NN` | Cassette, keeper, carriage, drums, catch parts | Commanded approach; recorded forward-CG climb |
| Service | `withdraw-service-pin` | Index pin | Straight 225 mm pull |

Every case states how far its straight path departs from the trace it follows; the limit is 1.0 mm. Cases name solids by their STEP product names, so the detailed STEP must keep those names or supply a mapping.

Five pairs touch by design and are qualified one case at a time (`joint-*.json`): the pivot shaft in both bearings and in the reducer, the cover leaf on its axle, and the slide screw in its travel nut.

Not in the layout solids, so not checked here: the removable slide-motor cartridge (`mounted-slide-02/`), the battery-service slide link and its two shoulder screws (`guided-hoist-01/removable-truck-lock.step`), the working stage chain, and the field. The battery lift at the service index is *pending*.

## 5. Budgets to stay inside

| Budget | Figure | Margin |
| --- | --- | --- |
| Inspection mass, upper estimate | 56.270 kg against 56.699 kg | 0.429 kg, with a 3 to 6 kg unallocated reserve inside the estimate |
| Loaded lift, weak pack | 8.705 V bus against an 8.257 V pivot demand | 0.448 V, with other loads staged to 25 A |
| Feed and launch, weak pack | 8.344 V against 8.0 V | 0.344 V |

Weak pack: 11.8 V open circuit, 27 mΩ. Reopen preliminary design if detailing consumes a margin.

## 6. Bench tests carried forward

`missions.json` lists each open risk with its measure, pass value and fail value. Results are not needed to start detailing. They are needed before a repeatability or strength claim.

## 7. First work items

1. **Profile A blade seat.** Detail the pickup crossbar and the nose-roller bearing supports around the blade's four-screw seat. Keep the blade removable and the runner height adjustable.
2. **Entry bearing brackets.** Relieve both so a nominal NOTE clears by at least 1 mm through discharge.
3. **Control-board guard.** Add a guard over the electrical bay against a NOTE returning from the speaker. Charge it to the unallocated reserve.
4. **Cover leaf.** Draw the leaf in its cam position at park.
5. **Undrawn connections.** Draw the eleven connections listed in section 3.
6. **Slide motor cartridge and service link.** Install both in the assembly and check their withdrawal paths there.
7. **Seat sensor.** Select and mount the seat beam.

## 8. The Fusion blade component

| Item | Value |
| --- | --- |
| Archive | `iterations/002/design/cad/guided-cassette/profile-a/profile-a.f3d`, SHA-256 `b2d9352d…8b1d` |
| Exported STEP | `profile-a.step` beside it, SHA-256 `7f55fe95…53de` |
| Stock | UHMW polyethylene, 434 mm wide, four M3 countersunk bores at Y = −165, −55, +55 and +165 mm |
| Coordinates | Drawn deployed. `geometry-check.json` gives the transform from deployed blade to parked assembly |
| Fit | Seats on the pickup crossbar with zero overlap; 2.335 mm to the nose roller; matches the layout blade both ways |

The blade is the first component of a guided-cassette Fusion design. No whole-robot Fusion design of this architecture exists yet. `iterations/002/design/cad/fusion-authority.json` still names the earlier side-finger archive and must be reissued for the guided cassette when the first assembly is saved.

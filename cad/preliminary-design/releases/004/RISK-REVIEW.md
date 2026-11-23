# Risk register review, release 004

Reviewed: the 27 risks that `cad/preliminary-design/build-004/risks.py` held on 2026-11-22, against the trial files they cite. `EV/` means `2024/preliminary-review-lan/greenfield-retained/`.

Status: the coordinator accepted the nine edits at the end of this file and they are applied. The register now has 32 risks. Findings 1 to 5 describe the register before the edits. Two follow-up runs settled findings 2 and 4: see "After the review" below.

## Disposition

"Trial" names the trial that sets the pass and fail values. Every risk with a bench test also stays open until the bench result exists.

| Risk | Disposition | Basis |
| --- | --- | --- |
| scoring-repeatability | Bench test | `EV/scoring-window-04/summary.json`. See finding 1 |
| firing-lane-tolerance | Design change (lane to y = 5.821 m), trial, bench test | `EV/scoring-window-04/` lateral, heading and station sweeps. See finding 3 |
| launch-angle-tolerance | Trial, bench test | `EV/scoring-window-04/` angle sweeps. See finding 4 |
| launch-motor-clearance | Design change (revision 004-A), bench test for the residual | `step-revision.json` |
| returning-note | Bench test. Guard not drawn | `EV/shared-counter-03/return-to-robot.json` |
| entry-bracket-clearance | Bench test | Detailing relieves the bracket |
| motor-package-clearance | Bench test | — |
| belt-indentation | Bench test | — |
| runner-indentation | Bench test | — |
| pickup-tip-height | Trial, bench test | `EV/pickup-brackets-04/results.json`: passes to 10 mm, fails at 12 mm |
| pickup-lateral-offset | Trial, bench test | Same file: passes at ±50 mm, fails at ±100 mm |
| pickup-grip | Trial, bench test | Same file: passes at 0.30, fails at 0.20 |
| seat-sensor | Bench test | — |
| cover-leaf-stop | Trial, bench test | `EV/retention-brackets-04/results.json` |
| auxiliary-load-during-lift | Bench test | See finding 6 |
| weak-battery-shot | Bench test | — |
| inspection-mass | Bench test | See finding 6 |
| endgame-clock | Closed by mission rule | Not a design change and not a new trial. The 24 s endgame case is the evidence |
| climb-centre-of-gravity | Trial, bench test | `EV/climb-brackets-04/results.json`: passes at +50 mm, fails at +60 mm |
| hoist-ramp | Trial, bench test | Same file: passes at 6.5 s, fails at 5 s |
| hoist-line-termination | Bench test | — |
| hoist-line-slack | Bench test | — |
| climb-catch-and-stops | Bench test | The no-catch control fails, as it should |
| battery-service-restraint | Bench test | — |
| battery-removal-path | Bench test. Not a checked case | See finding 7 |
| slide-motor-cartridge | Bench test | — |
| cover-leaf-park-pose | Closed by design decision (cam law) | Drawing action passes to detailed design |

No risk in the register has neither a closure nor a bench test. Five things that should be in the register are not (finding 5).

## Findings

1. **Two sentences in scoring-repeatability state results no trial holds.**
   - "The count rate did not move with ... its surface friction." None of the 1,774 result rows has a `fieldMu` other than 0.45. Friction was never varied. The speed sweep and the counter drive sweep (0.55 to 1.0) are real.
   - "Of 0 uncounted NOTES watched for 20 s, 0 counted late ..." No row has a duration other than 7 s. `extendedObservation` in `summary.json` is empty.
2. **"Eventually counted" is not shown.** The owner's pass criterion is delivered and eventually counted. No shot is watched beyond 7 s. At the released lane the three uncounted NOTES had all left the speaker by 7 s (`operating-point.json`, `uncountedLeftSpeaker`). Across the perturbed shots 66 uncounted NOTES left the speaker and 36 were still inside.
3. **firing-lane-tolerance bench numbers do not match the trial.**
   - The bench test fails at 3.0° of heading. At the released lane ±3° alone delivers all 34; −4° fails. The 3° number is right only for combined error: 20 mm toward the side wall with −3° counts 1 of 34 (`EV/scoring-window-04/results/aim-search.jsonl`). Say so in the text.
   - There is no position number. The window is −25 to +50 mm laterally and 0 to +60 mm fore-aft (7.6 mm margin at +60 mm; +100 mm loses 16 of 34).
   - The lane at y = 5.796 m was fired with 25 of 34 carries. Nothing closer to the wall than x = 1.39 m was fired; the bumper allows 35 mm that way.
4. **launch-angle-tolerance has no high edge.** The text says to centre the stop in the window, but no failing angle exists above the window. At the released lane the smallest margins are 3.4 mm at 121°, 56.7 mm at 122°, 67.7 mm at 123° and 39.8 mm at 124°. So 123° has the most margin, and the recommended 124° has less than 122°. 125° and above were not fired at the released lane. The bench values (121.5° pass, 121.0° fail) assume a 122° setpoint. `operating-point.json` still releases 122°.
5. **Missing from the register.**
   - Speaker count delay. The owner put the 7 s count rate on the bench list. Only the 19-of-20 test is there.
   - Launcher speed floor. Delivery fails at 5.60 m/s and below, and 5.92 m/s loses 1 of 6 (`EV/scoring-window-04/results/low-speed.jsonl`). The weak-battery test measures voltage, not belt speed.
   - Reach and height through chain capture and hoist. No file states them as numbers. The front reach at pickup has 2.5 mm to its limit (`EV/pickup-lip/layout.json`).
   - Launch shaft. The 60 mm longer shaft is checked for clearance only. Whip and bearing support are not assessed.
   - Combined aim error (finding 3).
6. **Budgets predate revision 004-A.** Two risks say so in passing. The 0.448 V lift margin and the 0.429 kg mass headroom were computed before the motor raise.
7. **battery-removal-path disagrees with an older record.** `EV/CURRENT-PRELIMINARY.md` reports 5 mm minimum clearance on a 370 mm disconnected lift. The register says the lift does not yet pass. One of them needs a note.
8. **Climb margin is thin.** At +50 mm the hold has 10.9 mm against 10 mm. The ramp brackets were run only with the centre of gravity 40 mm ahead.

## After the review

- **Finding 2, settled.** The three uncounted NOTES at 122° and the one at 123° were watched for 20 s. None counted late. Each bounced back out through the opening within 3 s without shadowing either beam (`EV/scoring-window-04/results/released-20s.jsonl`, `angle-123-20s.jsonl`). The literal criterion is not met for the three at the released point. It is an open finding for the owner.
- **Finding 4, settled.** Six spanning NOTES were fired at 125° and 126° at the released lane. 125° delivers 6 of 6 with 26.0 mm. 126° delivers 4 of 6. The window is 121° to 125°. The release keeps 122°. The launch-angle bench test is now departure from 122°: pass 1.0°, fail 2.0°.
- **Finding 3, partly settled.** The lane at y = 5.796 m now has all 34 carries, all delivered.
- **New.** At 123° the 12 nominal NOTES count 9 of 12, against 12 of 12 at 122°. That is why 123° is an alternative and not the setpoint. The three were not watched beyond 7 s.
- **Finding 6 is closed for the shafts.** The ledger now adds the mass of each solid the revision changed: 0.074 kg for the two lengthened shafts. The undrawn motor mounts remain outside it.

## Edits applied to `risks.py`

1. scoring-repeatability: delete the friction clause. Delete the 20 s sentence, or run the 20 s observation and keep it.
2. Add `speaker-count-delay`: "Delivered NOTES out of 20 counted by a real speaker within 7 s of the shot", pass 20, fail 18. Simulation gives 31 of 34.
3. firing-lane-tolerance: state that the heading numbers are for combined error. Add lateral error, pass 20 mm, fail 50 mm. Add fore-aft error, pass 30 mm, fail 100 mm.
4. launch-angle-tolerance: state the high edge (found after the review). The bench test becomes departure from the 122° setpoint: pass 1.0°, fail 2.0°.
5. Add `launcher-speed-floor`: "Launch belt surface speed at feed on a weak pack, 6.88 m/s command", pass 6.56 m/s, fail 5.92 m/s.
6. Add `endgame-envelope`: reach and height through capture and hoist, to be measured on the first Fusion assembly.
7. launch-motor-clearance: add shaft whip and bearing support to the text.
8. auxiliary-load-during-lift and inspection-mass: rerun `gross-power.json` with the 004-A masses, or mark both figures as computed before the motor raise.
9. battery-removal-path: cite the older 5 mm result and say why it no longer holds.

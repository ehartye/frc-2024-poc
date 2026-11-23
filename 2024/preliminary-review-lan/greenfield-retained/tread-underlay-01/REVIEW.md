# Profile A field review

**Updated disposition.** Two setup failures ended the frozen campaign, and the inward climb strikes fixed field steel. See the [whole-robot decision and updated viewer](../endgame-route-01/REVIEW.md) for the revised PARK route, sensor recovery and next climb layout.


**Decision.** Keep Profile A on the common cassette and hold detailed design. Test whether measured pickup and launch behavior leaves enough repeatability margin for six scored NOTES in 120 seconds. The rejection threshold is a second unsuccessful six-piece run on the frozen twenty-start roster. The campaign counts distinct pieces, requires entire opening passage and both counter shadows, and preserves each missed opportunity.

The LAN viewer opens on the fine scoring case. It also exposes the coarse case, two perturbed starts, the free-chassis climb and the earlier PARK route. Browser verification scrubbed the fine route, first failed start and disabled climb to their recorded end states. See [viewer check](viewer-check.json).

| Decision input | Current evidence |
|---|---|
| Complete smooth-ring cycle | Six scores at 116.68 s and 113.48 s on the two nominal runs. In the first setup variation the last NOTE crossed with 106.3 mm clearance and missed the counters. The second variation scored six at 119.32 s. See [frozen roster and model](../split-field-02/freeze.json). |
| Failure diagnosis | With exactly the same saved releases, halving the field timestep preserves the miss for R01 and the score for R02. R02 counter time varies from 3.69 to 4.07 s after the start of that firing cycle. For this comparison R01 settles inside the modeled field assembly without paired credit. It does not fall back to the carpet. See [miss](replay-R01-0.000125.json) and [score](replay-R02-0.000125.json). |
| Robot clearance | The changed robot passed all 29 tested route segments against the registered fixture bounds and dormant non-target pieces. The rigid belt backing now clears the recorded ring poses by at least 0.354 mm after angular discretization allowance. Intersample motion, wear and actual belt/load compliance remain open. See [route](route-review.json), [backing](backing-review.json) and [controlled STEP check](result.json). |
| Climb | Bilateral hook contact supports the floating chassis through a five-second motor cutoff. Minimum ground clearance is 85.82 mm on the 48-segment front-CG trial. Holding the gates open prevents lift. Added drive hardware clears the articulated chain at saved poses by at least 13.59 mm. See [contact](../chain-contact-02/front48.json), [control](../chain-contact-02/held-open.json) and [hardware clearance](loaded-chain-clearance.json). |
| Budgets | Estimated high inspection mass is 56.500 kg, with 0.199 kg available. With modeled 101.6 mm wheels and the 5.5 drive ratio, the loaded-drive estimate falls to 8.272 V against the 8.0 V floor. See [mass](review.json) and [drive](four-wheel-duty.json). |

## Next experiment

Run one adjustable cassette fixture with the selected Profile A blade, measured belt speed and a free NOTE. Start with the measured successful and failed setup conditions, retaining the same launch command and actual incoming orientation. Capture fresh and worn NOTE mass and diameter, carpet height, loaded battery voltage, belt speed on both sides, and synchronized side and overhead footage from first contact to departure. Use the footage to estimate exit speed, pitch and spin. Retain pushes, stalls, drops and rim rebounds.

On the bench first measure tread normal load versus deflection and maximum tangential pull before slip. Replace the assumed force law with that measured range. Replay the original development cases, then a separately reserved group of starts using fixed settings. A recovered development miss alone gives no new repeatability credit. Record whether the discrepancy comes before the opening or inside the receiver. The powered simulation currently has neither a calibrated NOTE/tread/friction curve nor a measured field rebound coefficient.

For climb, measure gate closure under load, overtravel and catch load transfer on a supported fixture before assigning a hardware rating. The physical drive, fastener load paths, compliance and full approach timing remain open. Detailed design can resume when the architecture has enough measured margin or when its operating requirement is explicitly revised.

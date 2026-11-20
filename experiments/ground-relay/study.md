# Blue 3 ground relay on the R14 chassis

Blue 3 can deposit two distinct NOTES within the modeled TELEOP window in all five frozen two-pad runs. Blue 2 collects both in four runs and stores both and finishes its reset in two. With a late receiver, adding a second floor pad lets Blue 3 finish its own job while Blue 2 still has work at the horn. Every transfer earns zero match points. [Replay the field lab](https://ehartye.github.io/frc-2024-poc/ground-relay.html)

The [R16 contact study](../registered-custody/study.md) replays these same five conditions against the surveyed R14 tread locations.

## Who owns each transfer?

The held-out six-robot field replay extends the [alliance role study](../alliance-service/study.md) to the archived `miss-and-stage` match. It records one Blue 3 pickup of `center-2/1` at 3.35 s into TELEOP. The new two-piece trial assigns Blue 3 to retrieval and Blue 2 to receipt. It preserves the six starting poses, four other robot tracks, field, clock and initial availability of the two tracked center NOTES. The relay adds its own second source and delivery criterion. [Role witness](role-demand.json) · [Ground deposit criterion](demand.json) · [Field tracks and opening stock](peer-tracks.json)

Blue 3 gets custody after a stationary seating interval and a local position check. A release counts only after the whole identified NOTE settles inside an empty, 10-inch-radius exchange pad. Blue 2 takes separate custody after docking and dwelling for 2.45 s. A downstream store requires Blue 2 to bring the same NOTE to rest at its storage location. Each robot carries one at a time. The trial supplies one NOTE at each source, with no replenishment. The horn closes the window at 150 s. The season manual describes NOTE dimensions in §6.3.4 and defines TELEOP limits on multiple-NOTE control in G409. The exchange pads are trial locations on the floor. [2024 CRESCENDO game manual](https://firstfrc.blob.core.windows.net/frc2024/Manual/HTML/2024GameManual.htm)

## Paired dispatches

Serial dispatch uses one pad and waits for Blue 2 to acknowledge pickup before assigning Blue 3 its next source. Buffered dispatch uses two distinct pads and sends Blue 3 onward after its deposit and retreat. Times in the delay column start at TELEOP. Event times in the replay use the full match clock.

| Receiver delay / second source offset | Serial B3 deposits | Buffered B3 deposits | Buffered B2 pickups | Buffered stores | Both reset after both stores |
|---|---:|---:|---:|---:|---|
| 0 s / 0 in | 2 | 2 | 2 | 2 | Yes |
| 110 s / 0 in | 1 | 2 | 2 | 1 | No |
| 20 s / 3.5 in | 2 | 2 | 2 | 2 | Yes |
| 112 s / 3.0 in, held out | 1 | 2 | 2 | 1 | No |
| 120 s / 0.8 in, held out | 1 | 2 | 1 | 1 | No |

The frozen baseline passes 2/3 cases. Its two supplemental held-out replays appear in the table. The frozen refinement passes 3/3 matched and 2/2 held-out cases. Worst and nearest-rank tenth-percentile rates both rise from 1/135 to 2/135 deposits per second. These fractions describe the specified cases. They do not estimate a match success probability. Axes and settings were committed before evaluation, and the policies were committed before opening the held-out results. Later witness instrumentation records release, peak acceleration and peer overlap with floor pieces. [Freeze receipts](freeze-lock.json) · [Serial evaluation](serial-results.json) · [Buffered evaluation](buffer-results.json)

At 120 s delay, Blue 3 deposits at match times 74.75 and 104.05 s and resets at 109.40 s. Blue 2 collects the first NOTE at 142.00 s, stores it at 146.70 s and resets from that store at 147.95 s. It starts toward the second NOTE, which remains on the pad at the horn. In the 112 s offset case, Blue 3 misses the initial contact, searches locally and completes the second deposit at 109.50 s. Blue 2 collects that NOTE at 148.25 s. [Event history and field samples](../../docs/ground-relay.json)

## Where the physical evidence ends

The parked selection comprises 909 bodies. Its bounds run from (−437.8125, −439.75, 0) to (538.612, 439.75, 968.315) mm. The source survey reconciles all 923 R9 rows with the published bill before applying the R13 removals and R14 restorations. Its six limiting bodies and both native archive digests are recorded. Blue 3 keeps a fixed heading. Its carried envelope includes the 7-inch NOTE radius and a proposed ground-transfer location one inch forward of the body-to-ring contact limit. Blue 2 and the four traffic robots use the field fixture's 34-inch square footprints. [R14 native result](../../cad/service-readiness/stiffness/native-fit.json) · [Survey](footprint.json)

Archived simulated acquisitions D1 and D2 seated at 4.4945 and 4.6545 s, with minimum carry side loads of 3.592 and 3.589 N respectively. R15 uses those seating times with a 1.5-inch alignment tolerance and a 4-inch local search band. The cubic routes assume 42 in/s and 42 in/s² limits. Peak loaded acceleration was 40.316 in/s². At 241.3 g NOTE mass and assumed friction coefficient 0.25, the modeled minimum carry-force surplus was 0.6502 N. For floor transfer the model holds still for 0.35 s and assumes a passive 7.38 mm vertical release, using the lowest archived short-lift clearance, zero initial horizontal speed and restitution 0.2. Physical R14 pickup, release geometry, receiver behavior, friction and localization remain unmeasured. [Archived carry traces](../../sim/pressure/evidence/maxswerve-development-carry.json) · [Short-lift checks](../../sim/pressure/evidence/retained-pickup-20261118/short-lift-verified.json)

Every 50 ms the model checks the two participating robots' swept footprints against each other, four traffic tracks, field edges, conservative stage polygons and settled relay pieces. The first source approach waits 28.35 s for a reserved lane. Three other robots cross the first NOTE's projected area before Blue 3 arrives. Whether the NOTE stays within local detection range is an assumption. The minimum modeled gap is 0.239 in (6.1 mm). A one-inch clearance requirement stops the relay after its first deposit. Other floor stock, unplanned changes of heading and peer-to-peer interactions between the four archived tracks are outside the check. [Timing and occupancy model](relay.py)

## Weakest case and failure observations

At the same 120 s delay and 0.8-inch offset, prepositioning Blue 2 alone still yields one pickup. One shortened storage route intersects Blue 1 at 139.95 s. Staging early and taking the direct westward storage route allows Blue 2 to collect at 137.45 s, store at 140.35 s, reset at 140.40 s and collect again at 148.20 s. At the horn Blue 2 still carries the second NOTE. These reruns used the observed weakest case and are reported separately from the frozen cohort. [Final custody for every run](custody-summary.json)

Empty stock, an out-of-pad release and zero effective retention each give Blue 3 zero deposits. Removing the first NOTE after traffic crosses its projection leaves only the second available, producing one deposit. Disabling lane reservation produces an overlap with Blue 1 before pickup. The one-inch margin control also records the specific floor NOTE that blocks the retreat. [Failure receipts](controls.json)

## Reproduce the evidence

Use Python 3.10+ and the scripts from [Agent FRC 0.42.0](https://github.com/ehartye/agent-frc/tree/687618cdeedaca33ba8271e1aab4b7ce2697b805/scripts). From the season repository root run:

```sh
python -B experiments/ground-relay/replay.py /path/to/agent-frc/scripts
python -B tests/test_ground_relay.py
```

The replay reconstructs the historical role, verifies the independent cohort digests, runs both policies, and recreates the student lab data from the same per-piece history. The native survey can be checked separately with `survey.py` and the archived full body-bounds export. The compact public survey lists the exact extrema. [Source hashes](sources.json)

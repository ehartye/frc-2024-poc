# 2024 six-robot match and human supply

[Open the published match and strategy lab](https://ehartye.github.io/frc-2024-poc/field-lab.html#rehearsal). **Play match** starts the six default robots. The score and clock remain above the field. Under **Teammate defaults**, open a robot to see or change its assignments. **Playback and match setup** contains the three-match series. **Compare alliance roles** replays the same six stations with alternative work for Blue 3.

## Piece ownership and timing

The live profile stages **107 regular NOTES**. Three marked NOTES start in each wing and five on the centerline. Each alliance begins with an allocation of 48 before robot setup: 45 SOURCE pieces and three possible preloads. Each actual preload removes one specific ID from that allocation. Unused preloads stay in reserve. The default has two preloads per alliance, 46 remaining in each SOURCE and eleven on the carpet. The six HIGH NOTES are a separate class outside this study. [2024 final manual §§5.2, 5.4, 6.3.4 and 6.4](https://firstfrc.blob.core.windows.net/frc2024/Manual/HTML/2024GameManual.htm) · [Season profile](../../kickoff/physical-match.json)

Each robot has capacity one. A SOURCE human player may release one waiting NOTE onto a modeled floor outlet during TELEOP. Each operator takes 4 s for a complete handling cycle and waits for space at its one-piece outlet. The four-second interval, floor settling position and one-place capacity are explicit scenario assumptions. They represent handling a piece and waiting for the chute to clear. The simulation resolves top-down motion from the outlet onward. The actual chute is above the carpet. Score credit requires a matching, carried, named NOTE and a valid attempt. The ground exchange credits zero points and returns the same ID to its marked pad after 0.5 s.

A free ring has a 7 in outer radius, 2 in height, 0.2413 kg assumed mass, 135 in/s² carpet deceleration and 0.10 restitution. Capture can reach 2 in beyond the bumper. A robot can push a piece out of reach or arrive after another robot collects it. The motion model uses a disk outline, so it cannot demonstrate compression, grip or retention on the robot. The R16 and R17 handling labs examine those questions separately. Travel and handling times and prescribed target outcomes are classroom inputs. The 15 s AUTO and 135 s TELEOP use the engine's two continuous period clocks. Official transition attribution, LIVE timing, fouls, amplification, HIGH NOTES and referee decisions limit score interpretation.

## Opening work and field outcomes

On each alliance the first robot attempts its preload and collects the middle wing NOTE. The second attempts its own preload at the AMP and collects the outer wing NOTE. The third starts empty, enters the inner lane, collects the inner wing NOTE and brings it toward the alliance exchange. All six acquired distinct assigned IDs in AUTO in all three published scenarios. When a candidate is unavailable, the assignment can consider other reachable own-wing NOTES. Rival wing stock is ineligible. In TELEOP the receiver can recover the deposited inner NOTE and complete an AMP delivery while the feeder seeks another piece. A separate strategy comparison gives Blue 3 a SOURCE shuttle. [Role definitions](roles.json)

| Published case | Blue total | Red total | Total regular NOTES accounted for at the horn |
|---|---:|---:|---:|
| Wing opening | 18 | 14 | 107 |
| Shared center | 14 | 17 | 107 |
| Miss and Stage | 15 | 14 | 107 |

Totals include the period-end STAGE assessment where earned. In the wing opening, both alliances' inner NOTES transfer to their receivers. Blue 1 picks up the first human release. The blue operator then releases the next ID. Floor pickups and attempts can also fail after motion or congestion. The traces preserve these failures and each final owner. [Clock, inventory, attempts and endgame ledger](series.json)

## Rehearsal inventory

| Entry | Configuration and scope |
|---|---|
| [Field lab and role comparison](https://ehartye.github.io/frc-2024-poc/field-lab.html#rehearsal) | Current 0.48.0 physical match and strategy comparison. The six-slot custom setup, three published matches, custom series and role replay use the same engine and physical profile. |
| [Blue 3 relay](https://ehartye.github.io/frc-2024-poc/ground-relay.html) | Frozen R15 six-actor clock with two floor pieces assigned to the carrier. This is a matched relay drill. |
| [Centered transfer](https://ehartye.github.io/frc-2024-poc/ground-contact.html) and [off-center entry](https://ehartye.github.io/frc-2024-poc/passive-entry.html) | R16 and R17 mechanism and custody studies keep the earlier six-actor timings. Bench controls are individual trials. |
| Historical layout and series | The [earlier profile](../../kickoff/crescendo-field.json) and [original series](../speaker-cycles/match-series.json) remain pinned for the original endgame and relay receipts. The project blockout and offline development campaigns are historical fixtures. |

Build the public comparison with the shared generator pinned at `92ddf41933dd6026b26493c1d2c91505e9d15914`. CI reproduces the generated HTML and full series and checks conservation, opening assignments and human service.

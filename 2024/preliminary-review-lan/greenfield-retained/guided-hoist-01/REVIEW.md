# Guided carriage on the shared cassette

The 230 mm carriage uses two chain hooks and two cable drums on the existing pivot drive. The cassette locks at chain capture. The clutch releases the shaft, the motor reels both lines, and two carriage pins take the load. The heavy planning case retains 0.736 kg of inspection headroom, including the 6 kg general reserve and a 0.20 kg allowance for take-up hardware.

[Forward-CG climb](http://192.168.1.129:8764/?concept=guided&case=guided-front) · [Rearward-CG climb](http://192.168.1.129:8764/?concept=guided&case=guided-rear) · [Battery withdrawal](http://192.168.1.129:8764/?concept=guided&case=guided-service)

| Free-chassis test | Forward CG, +40 mm | Rearward CG, −40 mm |
|---|---:|---:|
| Minimum running clearance in five seconds disabled | 15.03 mm | 83.55 mm |
| Maximum disabled floor reaction | 0 N | 0 N |
| Minimum loaded bus, weak pack | 11.00 V | 10.98 V |
| Peak combined cable tension | 637 N | 725 N |
| Time at power cut after endgame decision | 22.79 s | 22.79 s |

The same real-time chain and robot dynamics with an inhibited keeper prevented capture. Disabling the positive slide stops brought the chassis down. With the motor simply disconnected after reeling, up to 227 mm of excess line paid out. Unloading onto the pins and applying 0.30 N·m total passive drum bias reduced the largest modeled slack to 0.19 mm. The animation uses the solved robot, chain, rotor and carriage trajectories. The latch engagement shown before winding is an allocation. Physical linkage, individual cable stretch, unequal loading, take-up torque through travel, drum ends, pin capacity and shock still need evidence.

## Hardware and clearance

The mounted rope eye needs 82 mm and its splice allowance uses another 180 mm. The 27 mm eye thimble and 26 mm sheave fit 3 mm D12 MAX 99 rope. At the maximum 233 mm carriage travel, 12 mm of straight rope remains before the thick splice reaches the sheave. The earlier 170 mm lateral cable station hit the pack lead. The selected station is 160 mm from center. The original no-take-up peak gives a conservative installed line target of 5.73 kN with the stated 1.5 dynamic and 5:1 strength factors. The catalog spliced minimum is 16.2 kN. Installed bend, dead end, actual splice, wear and local mounts still need qualification. See the private research note *2024 preliminary hoist line and termination*.

The selected stock had zero occupied intersections in 797 close internal comparisons across eight states and 667 chassis comparisons across 100 poses. Five free-NOTE trajectories clear the changed stock on sampled poses by at least 39 mm. The two chain-approach broad-phase candidates clear by at least 37.6 mm in the local channel frame. Two 553-pose climb traces were also screened against the field and moving chain. Small deflections, contact friction, real rope sag and unmodeled purchased components remain outside those results.

At the existing 80° battery service pose, the right guide alone occupies 22,030 mm³ of the required battery path. A second bore in the service disk accepts the existing pin at −40°, giving 10 mm of engagement and a 5 mm gap along the full disconnected 370 mm battery lift. Either 4° misindex produces 83.58 mm³ of interference. Verify a positive slide restraint, the two pin load paths and actual handling with disconnected leads before hands-on service. The mass allowance conservatively retains the 3 g removed by the second bore.

## Evidence

- Source geometry and service revision: `inboard-guide.step`, `service-index-guide.step`, `inboard.json`, `service-pin.json`.
- Purchased-line allowance and installation: `inboard-exchange.json`, `inboard-service.json`, `service-index.json`.
- Installed paths and retained controls: `inboard-whole-screen.json`, `inboard-route.json`, `inboard-first-*.json`, `inboard-dormant.json`, `inboard-piece-path.json`, `inboard-approach.json`, `inboard-approach-chain.json`, `return-field.json`.
- Free physics: `winch.py`, `line-return.py`, `return-{front,rear,open,no-catch}.json`, `return-budget.json`. The earlier `winch-*.json` runs preserve the unrestrained drum result.
- Run `publish-review.py` then `publish-service.py` to refresh the existing private viewer. `verify-view.cjs` checks the LAN page in Edge.

The frozen six-NOTE campaign remains six successful starts, two failed starts and twelve unrun. One miss followed a clean 106 mm opening crossing. The other occurred before seating. Carrying forward an earlier scoring animation does not count as a fresh score for this assembly.

## Power-off battery service

The current parked assembly is `service-locked-robot.step`. At the −40° index, the removable two-screw link restrains slide translation and the pin restrains rotation. The first link hit the pivot reducer. The revised link and both drivers clear insertion, and the disconnected battery has 5 mm minimum clearance throughout the 370 mm withdrawal. Two ±2 mm translation controls obstruct the shanks. Bench load rating, thread engagement and actual operator access remain open. The owner viewer shows the installation and withdrawal under battery service. See truck-lock.json and removable-truck-lock.step.

The current scoring experiment and its stop conditions are in ../lateral-station-01/REVIEW.md.

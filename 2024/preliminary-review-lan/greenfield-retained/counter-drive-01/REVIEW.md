# Six-NOTE scoring review

Profile A remains the default removable lip. The front of the robot and the 6.88 m/s scoring command are unchanged. Two separate effects explain the original misses.

## Field counter

The season drawing assigns nine grip wheels, an upper conveyor and a separate lower conveyor to the speaker. The original solver left the upper conveyor and the grip wheels stationary. It also drove the lower conveyor at 1.50 m/s. The lower 12:1 drive and measured belt geometry imply about 1.13 m/s at the catalog 12 V no-load speed. The upper conveyor and grip wheels turn on opposite sides of a gear mesh. Absolute polarity and loaded field RPM still need on-field confirmation.

On the same R01 final launch, the original model missed. With both drives at 55%, 75% and 90% of nominal no-load speed, the two modeled counters registered at 5.949, 4.747 and 5.319 seconds. At 75% with reversed upper drive neither registered. All runs preserve exactly the incoming state, wheel speeds at feed, release velocity and full opening crossing. The full NOTE cleared the opening by 106 mm. The paired calculation captures a real change in the field model; its operating speeds are assumptions. These times are from start of spin-up.

| Starting setup | Scored | Elapsed | Lowest shot bus |
|---|---:|---:|---:|
| dev · 48 NOTE segments | 6/6 | 116.63 s | 8.344 V |
| dev · 96 NOTE segments | 6/6 | 113.10 s | 8.349 V |
| R01 · 96 NOTE segments | 6/6 | 115.80 s | 8.349 V |

The mission clock retains the conservative wait for both field beams before each departure. The table reuses the three incoming sequences to isolate the field change. The previous six-of-eight results remain a separate campaign.

## Floor acquisition

The coarse R08 simulation stopped on the carpet beyond the blade. Waiting longer for a seated-piece signal left the NOTE on the floor. The flat rectangles used to represent the nose belt protrude at the 11 mm pulley radius. Halving each rectangle length while keeping the same stroke, pitch, total moving mass, surface speed and width tests whether that representation changes acquisition. Independent timestep and ring-segmentation changes also affect the coarse result.

| NOTE segments | Belt elements | Step (ms) | Pad corner excess (mm) | Side drive takes over (s) | Seat beam confirms (s) | Shot counted |
|---:|---:|---:|---:|---:|---:|---|
| 48 | 152 | 0.50 | 0.53 | 1.308 | 2.820 | yes |
| 48 | 38 | 0.25 | 6.72 | 2.330 | 3.252 | untested |
| 48 | 76 | 0.50 | 1.98 | 1.286 | 2.820 | yes |
| 96 | 152 | 0.25 | 0.53 | 1.309 | 2.820 | yes |
| 96 | 38 | 0.50 | 6.72 | 2.239 | 3.147 | untested |
| 96 | 76 | 0.25 | 1.98 | 1.295 | 2.820 | yes |

The 76- and 152-element belts both transferred the same free R08 NOTE to the side drives and confirmed its seat at 2.82 s on both clocks. These are controlled model refinements. Actual compression, friction, carpet and production lip height still need the physical pickup fixture. A matched campaign with the 76-element belt, 40 ms seat interlock and coupled field drive is recorded leg by leg in the mission receipts. It resets the prespecified count, keeps the 120 s project limit and the 8 V minimum shot bus, and stops once a nomination cannot pass. The earlier campaign and timings remain accessible.

## Review

- [Watch the driven field counter](http://192.168.1.129:8764/?concept=guided&case=counter-driven)
- [Watch the reversed upper drive](http://192.168.1.129:8764/?concept=guided&case=counter-reversed)

The two clips display the measured free NOTE and official field fixture. Field drive motion is represented in contact forces. The displayed field mesh remains stationary. A frozen whole-robot decision also needs the loaded cassette/drive budgets and the battery-service slide restraint.

Sources: FIRST season-specific GE-24351, GE-24370 and GE-24385 drawings, the issued field mesh, the V0 manual §6.5.1, and the AndyMark am-0255 12 V motor curve. The field measurements and gear checks are reproducible in counter.py and transmission.json.

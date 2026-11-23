# 2024 preliminary design

The greenfield guided cassette entered Detailed Design for the retrospective POC on 22 November 2026. The first editable Fusion article is Profile A, verified against the selected 232-body layout. The season project records the decision and component under `iterations/002/design/cad/guided-cassette/HANDOFF.md`. Later competition validation still owns the 19-of-20 scoring target. The table below preserves the earlier preliminary checkpoint.

[Open the current animated scoring rehearsal](http://192.168.1.129:8764/?concept=guided&case=loaded-cycle-48) · [Forward climb and power cut](http://192.168.1.129:8764/?concept=guided&case=guided-front) · [Rearward climb and power cut](http://192.168.1.129:8764/?concept=guided&case=guided-rear) · [Power-off battery removal](http://192.168.1.129:8764/?concept=guided&case=guided-service)

| Review question | Current evidence |
| --- | --- |
| Can the mechanism run the six-piece mission? | Both nominal settings scored six in 115.629 / 115.463 s after extending field observation through the reset. [Watch the fine replay](http://192.168.1.129:8764/?concept=guided&case=observed-reset-96). The successor carries nose phase, belt speed and the flap state between pickups. Qualification requires 19 of 20 starts. [Successive pickup experiment](counted-reset-01/REVIEW.md). |
| What changes if the pack sags? | The candidate loaded return estimates 8.712 V minimum and 0.456 V minimum motor terminal reserve at 25 A of other loads. At 39 A the limiting motor runs short while the pack remains above 8 V. See loaded-actuation-01/purchased-power.json. |
| Does the layout clear? | 232-solid parked stock, 118 loaded-lift states and 1,208 close moving-to-fixed comparisons for the bounded slide profile. Eleven changed route segments cleared fixed field stock and dormant NOTES. Full path, loaded mechanism tracking, dynamic wheel loads and safety margins must continue to agree as the proposal changes. |
| How much weight is available? | 55.963 kg high inspection estimate and 0.736 kg to the 56.699 kg limit. Charging the 59.0 g removable motor mount and six fasteners in full leaves 0.677 kg before any credit for the replaced mount. [Motor installation and service](removable-slide-01/REVIEW.md). Reconcile the new hardware and all bumper and inspection exclusions before acceptance. |
| Can the operator change the pack and can the robot end the match safely? | The two-axis battery service lock gives 5 mm minimum clearance during a 370 mm disconnected lift. Forward/rear guided-climb cases keep the tires unloaded after power is cut, with 15.4/93.8 mm final travel clearance. The simulated catches need physical proof-load, wear and duty verification. PARK has its own route. |


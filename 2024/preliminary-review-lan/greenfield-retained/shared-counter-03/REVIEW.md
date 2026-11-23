# Six-NOTE replay and shared speaker counter

[Fine continuous replay](http://192.168.1.129:8764/?concept=guided&case=continuous-cycle-96) · [Coarse continuous replay](http://192.168.1.129:8764/?concept=guided&case=continuous-cycle-48)

## Display coordinates

The pickup solver writes its center in chassis coordinates and its final free-joint state in integration coordinates. The initial publisher subtracted forward drive from both. While the robot carried each NOTE the displayed center lagged the physical center by 440 mm on five trips and 350 mm on the inner trip. The corrected publisher uses the chassis-relative coordinates directly during pickup and carry. It converts the raw free-joint state only when handing off to the field solver. The last carry sample is included.

The independent LAN check reconstructs the centroid of the displayed NOTE mesh through the published frame matrices. All 2,800 sampled pickup/carry positions on both numerical resolutions agree with their archived physical positions within 0.001 mm. The published carry-to-shot transitions also agree within 0.001 mm on all twelve deliveries. [Check](served-trajectory-check.json)

The separate triangle-surface audit tests a 128-site torus against the displayed stock at every replay frame. It finds nominal overlap up to 7.67 mm on the modeled central intake belt, 6.13 mm on a feed tread, 2.09 mm on the rear soft stop, 1.36 mm on a runner, 0.53 mm on the keeper panel and 0.13 mm on the lip. The solid orange ring shows its unloaded dimensions. Physical indentation and hard-part compliance remain open. The corrected nominal paths give at least 0.189 mm of conservative sphere-to-motor-package interval clearance under the stated 12 m/s relative-speed bound. The earlier 73.99 mm allowance used the doubly subtracted coordinate and is superseded. [Fine geometry audit](view-contact-96.json) · [Motor package](../mounted-slide-02/free-piece-clearance.json)

## Design decision

Three of the first five frozen starts met the individual-shot requirement. R02 missed the counter on its third NOTE and R03 on its sixth within the original field window. Both cleared the speaker opening. Even winning the remaining fifteen starts would yield at most eighteen of twenty. Preserve the frozen decision while evaluating whether the observation window cut off a later field event. [Decision](design-decision.json)

In the coarse model the middle NOTE came to rest on the counter sensor bracket and counter floor in the next NOTE's path. Drawing GE-24361 identifies both supports. In a shared field run the second NOTE displaced the first, both beams had an unobstructed ray at 20.745 s, and the second fully shadowed both at 21.2615 s on the first shot's local clock. Before first contact the states reproduced the independent trajectories. Disabling the interpiece response triggered the solver's excessive-penetration veto. This result supports this pair. Six-piece shared occupancy and repeatability remain open. [Coupled trajectory](pair-coupled.json) · [Counter support contacts](../counted-reset-power-02/terminal-contacts.json)


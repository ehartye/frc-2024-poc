# Preliminary design · loaded pickup to score

Profile A stays on the greenfield cassette. The screening sequence uses the 232-body guided assembly, a continuously confirmed seat, and a fixed lateral scoring station at (1.390, 5.871) m with the cassette at 122°. Detailed CAD remains paused.

## Motion and demand

The prior prescribed slide law reaches its 185 mm limit with nonzero velocity. Its apparent acceleration grew from 131 to 1,009 m/s² as the analysis step decreased from 2 to 0.25 ms. Smoothing the slide reference over 160 ms of the old 3.4 s lift changes its horizontal location by up to 7.02 mm. The operative candidate uses the same spatial curve stretched to 4.8 s. The complete rigid model returned zero common volume above 0.01 mm³ on 1,208 close comparisons at 118 sampled states. Contact between samples, compliance, bearing ratings, and physical servo tracking remain open.

The loaded power screen includes the NOTE, a 69.013 kg upper running weight, and a further 2 kg of existing design reserve allocated to the rotating cassette. The four 5.5:1 traction motors act at the wheel contacts while the truck and pivot move. With a 25 A auxiliary allocation the 11.8 V / 27 mΩ pack falls to 8.712 V. At the governing instant the drives, pivot and slide draw 27.88, 31.02 and 30.46 A from the battery. The 4.8 s candidate leaves at least 0.456 V between the pack and the required motor terminal voltage in these cases. A 39 A auxiliary allocation still clears 8 V but loses 0.080 V of motor tracking margin. Staging power is part of the design.

The NEO 550 upper current bound for the proposed slide is 38.93 A on the motor side with the example direct 8 mm/revolution, 65% efficient screw. The loaded shared pivot needs −29.65 to +31.66 A under the stated conservative mass placement. The proposed pickup/reset mode permits ±35 A on the pivot. The simulated climb has its own −25/+35 A limit. Motor selection and geometry for the spindle, nut and coupling, thermal duty, cable routing and the mass of the final installation need separate closure. The empty stationary return, including a 30 A residual allowance, screened at 9.992 V under the provisional slide curve.

## Scoring and reset

Each floor piece uses free-body contact through pickup, transport and release. The previous 3.4 s lift scored all six in both nominal discretizations and on R01 and R08. The 4.8 s lift scored six at the coarser 0.5 ms nominal resolution in 118.460 seconds, with 1.540 seconds to the project limit. A 1 s timer after the feed command allows the empty cassette to begin returning while the chassis stays parked. The test charges overlapping time only when the full game-piece envelope remains clear of the swept chassis and the residual shooter battery demand stays at or below 30 A. Other shots use a sequential reset. Counter timing is read independently. The run files retain both schedules.

The 19-of-20 operating gate remains open. See run-dev-48.json, run-dev-96.json and status.json as trials finish. A 12 m/s between-frame allowance and the 75% counter-motor speed used in field contact are modeling assumptions. The display uses actual logged piece poses, counters and route times. Loaded cassette position is commanded and receives an independent actuation screen. Pickup-to-pickup rotor initialization, true carpet/foam behavior, safe physical service, disabled holding, bus/actuator tracking and complete match integration must stay visible.

[Open the nominal bounded-lift rehearsal on the LAN](http://192.168.1.129:8764/?concept=guided&case=loaded-cycle-48). In the power ledger select **Loaded lift and drive · 4.8 s candidate** to inspect the synchronized loads.

Evidence is in clearance.json, reset-load.json, freeze.json, run-*.json, viewer-receipt.json and verified-view.json. The supporting mass/motor calculations are in ../loaded-actuation-01/purchased-power.json and retime.json.


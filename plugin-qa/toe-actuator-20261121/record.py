from pathlib import Path
import json,hashlib
vault=Path.home()/'.frc-wiki-vault';d=vault/'.wiki-master/research-scorer-cad'; q=Path(__file__).parent
rows=[json.loads(s) for f in ['toe-captures.jsonl','toe-matched-captures.jsonl'] for s in (d/f).read_text(encoding='utf-8-sig').splitlines()]
source='wiki/sources/pickup-toe-rack-and-pinion.md'
note='wiki/authored/frc/reference/2024-three-toe-actuator-trial.md'
body="""# Purchased rack and pinion geometry for the pickup toes

REV documents the 22-tooth REV-41-1503 nylon PA6 right-angle rack as compatible with its DUO plastic gear family. REV-41-1332 is the 30-tooth 5 mm hex acetal pinion, module 0.75, 20° pressure angle and 15 mm width. Pitch radius is 11.25 mm. The private manufacturer models both read as one valid solid. The rack bounds project 47.124 mm along its tooth line. The 2019-07-23 rack STEP is `2d84dec1090cf680ac69eda4ee8593b0187a42587675d07db0921834514a8495`; the 2016-09-04 pinion STEP is `f2d6208787f0b5101fbab06251bbf127fc9d6813c9a4d71bb656e6c543dfd1ae`. Both files are in private QA custody. Register the fasteners, guide stiffness and actual tooth continuity at the joints to qualify engagement on a robot. [REV rack](https://www.revrobotics.com/rev-41-1503/) · [REV plastic gears](https://www.revrobotics.com/5mm-Plastic-Hex-Bore-Gears/) · [REV gear reference](https://docs.revrobotics.com/duo-build/motion/gears)

A plausible cross-vendor combination failed before installation. REV-41-1665 is a 20° metal gear. On goBILDA 2311-0001-0118 the imported STEP's 254 planar flank normals give 14.5° and the tooth pitch is 2.5133 mm. The matching module 0.8 only establishes pitch. The vendor marks 0118 discontinued and directs readers to 0131; its December 2024 STEP postdates kickoff, and no qualified interface follows from the substitution. The selected mono-vendor pair above avoids that known pressure-angle conflict. [REV metal gears](https://www.revrobotics.com/5mm-Metal-Hex-Bore-Gears/) · [Original rack](https://www.gobilda.com/acetal-mod-0-8-gear-rack-118-tooth-41-hole/) · [Successor rack](https://www.gobilda.com/acetal-mod-0-8-gear-rack-131-tooth-41-hole/)

The private artifacts, diagnostics and STEP hashes are held in `Documents/FRC Native/plugin-qa/toe-actuator-20261121`. The complete candidate and limits are in [[wiki/authored/frc/reference/2024-three-toe-actuator-trial]].
"""
def fm(kind):
 base='---\ntype: '+kind+'\n'
 if kind=='authored':base+='project: frc\nkind: reference\n'
 base+='created: 2026-11-21\nupdated: 2026-11-21\nreviewed: 2026-11-21\nstatus: maintained\n'
 if kind=='source':base+='sources: '+json.dumps(['[['+x['path']+']]' for x in rows])+'\nsource-hashes: '+json.dumps([x['sourceHash'] for x in rows])+'\n'
 else:base+='sources: []\n'
 return base+'ai-generated: true\n---\n'
(vault/source).write_text(fm('source')+body,encoding='utf-8')
noteText="""# Three-toe pickup drive for the Blue 1 floor route

Trial two wrist-mounted NEO 550 motors with independent SPARK MAX controllers. Drive the two guided side fingers from opposite sides of one REV-41-1332 pinion. A second pinion drives the rear finger on its own guide. Both transmissions use one REV-41-1602 planetary stage at 76:21, 550 input plate 1607 and pressed pinion 1608, V2 1615 output and a captured, bearing-supported 5 mm hex shaft. Mount each pinion between supported cheeks. Fit 1503 rack segments to a stiff backed carriage and guard the mesh. Check every segment joint for tooth phase, backlash and backing stiffness through travel. The side pitch lines are 22.5 mm apart. Two 47.124 mm segments per side and three rear segments are first length reservations; prototype the splice and end engagement before fixing the cut lengths. Exact bought STEP files for the rack, pinion, motor, input plate, input pinion, output and stage are in private QA custody. [[wiki/sources/pickup-toe-rack-and-pinion]] · [[wiki/authored/frc/reference/reefscape-grip-slide-drive-sources]] · [[wiki/sources/rev-ultraplanetary-and-cb185-service]]

| Time from docking | Side pair, each | Rear | Meaning |
| --- | --- | --- | --- |
| 0.32–0.56 s | 0 to +34 mm | 0 to +34 mm | Cubic extension; both command maxima 0.2125 m/s. |
| 0.56–3.70 s | +34 mm | +34 mm | Support and carry. Finite compliance permits recoil while the target holds. |
| 3.70–3.94 s | +34 to −23 mm | +34 to −53 mm | Cubic withdrawal; 0.35625 and 0.54375 m/s command maxima. |
| After 3.94 s | −23 mm | −53 mm | Clear the receiving bed; a later reset to zero requires its own clock. |

The simulator instrumentation preserves the matched roster, finite stock, full original mechanism and actuator commands. The coarse rerun reproduces both original run and floor-cycle JSON exactly. In that trace each side uses 5.93 N at 0.109 m/s on acquisition and 1.91 N at its 0.296 m/s acquisition speed peak. During withdrawal the rear uses only 0.296 N at 0.543 m/s; its peak 6.28 N comes while holding the commanded position, at 0.155 m/s of actual recoil. Actual opposing slider differential stayed below 0.003 mm in this trial. Asymmetric entry, jam and lost tooth engagement still need independent trials.

The provisional electrical model takes 0.65 transmission efficiency, 1 N extra drag per moving toe, 90% controller efficiency, the 12 V NEO 550 linear data and the recorded applied toe efforts. Combining both lateral loads at the common shaft gives 8.23 A maximum motor current, rear 4.99 A, peak terminal voltage across the two axes 2.04 V; the most demanding coincident sampled pair takes 1.69 A at a fixed 8.884 V bus. These are coarse replay sensitivities before measured additional friction, rotor/gear/rack inertia and acceleration. Motor current and bus current differ under PWM. The full robot replay must replace the original ideal toe power term so it is counted once and reevaluate the 11.8 V/0.027 Ω battery under simultaneous scorer/lift current. The 87 mm rear motion has nominal command peak acceleration 9.06 m/s² and motor angular acceleration about 2,916 rad/s². Drive reflected inertia and local jam loads can change the selection.

For this transmission, both motor housings, single stages, bearings and fixed guides ride with the pitching wrist through reach and lift. Each translating toe carries its own rack, carriage and fasteners. Put the controllers on the chassis and model both moving harnesses. The two motors, two gear stages and two V2 outputs alone sum to approximately 0.485 kg; replace that share of the project's existing 2–6 kg retrieval/feeder/guard allowance. Replace the simulation's 0.8 kg head and three 0.08 kg fingers with CAD-derived masses, centers and moments in their actual frames. Mounting motors behind the pivot requires a separately proven transmission through the moving joints.

Keep new extremities inside the shortened-head swept envelope. The existing front rail decision in `over-bumper-pickup/direct-front-rail.md` yields 14.84 mm nominal forward reserve only with a structurally attached X420 rail under the kickoff 12-inch rule. During 60° pitch, inspect the full rack tails, both motors, fasteners, harness loops, receiver and cover, also maintaining the free ring path. A rear rack train roughly 141 mm long with its leading tip at local X+12 mm when withdrawn reaches approximately X−216 mm at full extension. This is an illustrative packaging gate. Verify stop-to-stop tooth coverage, installed force, front datum, support, pinch guarding, cable and driver access before detailing or scoring credit. The 2024 release manual PDF SHA starts `04f28dce`; printed p87 lists REV-21-1651 and pp89–90 govern the motor controller pairing.

Independently sense physical withdrawal of both sides and the rear finger, plus the existing pitch, reach and lift conditions. Jam one toe at +34 mm while its motor encoder reports clear and the head is parked. Feed must remain inhibited, the drive must time out under a tested force limit, and rearm must require confirmed clearance. Repeat with a rear jam. A shared current limit cannot guarantee equal force on the two sides if one jams; bound force at each toe with an instrumented compliant link or demonstrate an adequately conservative calibrated total limit. The previous ±15 N simulation clamp on each toe provides no qualification of a mechanical pinch limit. The first discriminator is one complete loaded capture and withdrawal in the two existing clocks, then jam clearing with no feed. This supplies no new scoring or build readiness claim.

Private data, sources and replay script are in `Documents/FRC Native/plugin-qa/toe-actuator-20261121`, including `phase-demand.json`, `replay-96/toe.npy` and the exact supplier solids. The season's `iterations/002/design/over-bumper-pickup` contains the two original head-clear replays.
"""
(vault/note).write_text(fm('authored')+noteText,encoding='utf-8')
(d/'toe-authored.json').write_text(json.dumps([source,note]),encoding='utf-8')


from pathlib import Path
import json,hashlib
q=Path(__file__).parent;V=Path.home()/'.frc-wiki-vault'
rows=[json.loads(l) for l in (V/'.wiki-master/research-scorer-cad/toe-support.jsonl').read_text().splitlines() if l]
sp='wiki/sources/pickup-toe-guides-and-home-switches.md'
ap='wiki/authored/frc/reference/2024-pickup-wrist-support.md'
def write(rel,txt):
 (V/rel).write_text(txt,encoding='utf-8')
def hdr(source=False):
 s='---\ntype: '+('source' if source else 'authored\nproject: frc\nkind: reference')+'\ncreated: 2026-11-21\nupdated: 2026-11-21\nreviewed: 2026-11-21\nstatus: maintained\n'
 s+='sources: '+json.dumps(['[['+i['path']+']]' for i in rows] if source else [])+'\n'
 if source:s+='source-hashes: '+json.dumps([i['sourceHash'] for i in rows])+'\n'
 return s+'ai-generated: true\n---\n'
write(sp,hdr(True)+'''# Bearing and switch evidence for a movable pickup head

The REV-41-1683 STEP contains a bearing block, a 679ZZ bearing and a separate hex insert. The block has six through-hole axes in the mounting plane. An installed model needs a clocked insert, an axially retained shaft and positive connections from the stationary block to the frame. The vendor describes the bearing family and its mounting arrangements. The retrieved documents supplied no application-specific life or permissible load for this installation. [REV 5 mm hex bearing block](https://www.revrobotics.com/rev-41-1683/) · [REV bearing arrangements](https://docs.revrobotics.com/duo-build/motion/intro/bearings)

The goBILDA 3103-0001-0002 is a 2 g momentary SPDT switch with a bump lever and 3.2 mm spade terminals. Its available supplier STEP depicts a rigid undeformed switch. Registering the finger on the bump can locate the mechanical trip zone. Commissioning still requires the electrical switching points, permissible operating and overtravel range, input current and contamination behavior to suit the application. The available Omron D2F family evidence supplies a family reference. A different SKU needs its own switch geometry and actuation limits. [goBILDA switch](https://www.gobilda.com/micro-limit-switch-bump-lever-2-pack/) · [Omron D2F family](https://components.omron.com/us-en/products/switches/D2F)

Project use and derived measurements are in [[wiki/authored/frc/reference/2024-pickup-wrist-support]]. The driven elements are registered in [[wiki/sources/pickup-toe-rack-and-pinion]].
''')
write(ap,hdr()+'''# Pickup head support and direct toe-home observation

The proposed 2024 pickup translates its opposed side fingers with a common pinion and drives the rear finger separately. The pivot requirement puts a 12.7 mm shaft at head X−70/Z195 over Y±218. Its actual volume intersects the earlier rear rack by 404.70 mm³ in neutral. Moving the pinion to X−106/Y−25/Z200 clears the purchased bodies at the registered shaft. The candidate uses the original recorded motion and two elastic free-ring replays during verification. [[wiki/authored/frc/reference/2024-three-toe-actuator-trial]]

Three proposed head-mounted switches each directly watch one physical finger. Both side switches sit at Z75 and the rear switch at Z62. At commanded home each finger displaces its own free bump by about 0.50 mm. Independent BRep subtraction confines the common to that bump. Each finger 10 mm behind home leaves 9.499 mm between itself and the free lever. At one common elevation a connector withdrawal gauge cuts the neighboring switch by 69.795 mm³. Lowering the rear switch 13 mm clears all nine 4 mm diameter by 16 mm terminal withdrawal gauges against the switches, other terminal gauges and three home posts. Electrical trip and full harness service remain open. Require an away-to-home edge from each switch after extension, complementary contact states, a clear head, and a timeout that stops both drives and withholds feed. An encoder on the common pinion cannot observe a separated finger. [[wiki/sources/pickup-toe-guides-and-home-switches]]

Two guide stations take moments between each lifted rack and its floor-level finger. The 22 mm span on the side guides and 31 mm span on the rear put up to 44 N on each rear sliding pad in a conservative allocation of the historical generalized axial demand to the toe. For assumed sliding friction 0.10 the opposed sides together require 25.66 N and the rear 15.09 N. The corresponding estimated motor currents at the preserved operating points are 14.05 A and 8.84 A, conditional on the existing gear and efficiency assumption. Determine drag, stiffness, stops, trapped wire and asymmetrical loads before commissioning.

The rear link needs simultaneous clearance to the field bumper, side motor, wrist and free ring. A central link cuts the side motor by 984.80 mm³ at withdrawal. Moving the link aft puts its low stock into the bumper during acquisition. The laterally offset trial keeps the riser at X−146…−138/Y30…34 and traverses back to the rear finger below the side motor. Its worst sampled bumper separation is 4.927 mm in the 96-segment clock and 4.933 mm in the 192-segment clock. These margins refer to the registered working route and the selected stock envelope. Other field states, deflection and tolerances require separate checks.

The source STEP, both clock roots, rejected geometry, 115-part initial mass estimate, detailed bearing support, switch logic and successor tests are private QA artifacts under `Documents/FRC Native/plugin-qa/toe-actuator-20261121/support`. The reference directories include `screen.json`, `screen-v2.json`, `three-witnesses-lowered.json`, `head-path-v3.json`, `loaded-cg.json` and `shaft-demand.json`. The two-link arms, actual wrist drive and the issue authority remain with season engineering. This is an offline installation study.
''')
(q/'wiki-paths.json').write_text(json.dumps([ap]))

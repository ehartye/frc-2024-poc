"""Generate only numeric mission geometry from the pinned original A2 contract."""
import pathlib,json,hashlib,argparse
ROOT=pathlib.Path(__file__).resolve().parents[1]
p=ROOT/'mission/cycle-contract-A2.json';data=p.read_bytes();j=json.loads(data);g=j['geometry'];i=g['intake'];s=g['shooter']
def array(values):return '{'+','.join(repr(float(x)) for x in values)+'}'
text='package org.agentfrc.lesson;\n/** Generated from pinned original design contract; not installed-field calibration. */\npublic final class A2MissionGeometry {\n private A2MissionGeometry(){}\n'
text+=' public static final String CONTRACT_SHA256="'+hashlib.sha256(data).hexdigest()+'";\n'
for name,value in {'TRACK':g['track'],'SHOT_X':s['shotPose']['worldRobotCenter'][0],'SHOT_Y':s['shotPose']['worldRobotCenter'][1],'SHOT_HEADING_DEG':s['shotPose']['worldHeadingDeg'],'PIVOT_X':i['pivot'][0],'PIVOT_Z':i['pivot'][2],'ARM_MAX':i['armLength'],'ARM_MIN':i['minimumLength'],'FORWARD_CENTER':i['maximumForwardCenter'],'MAXIMUM_HEIGHT':i['maximumHeight'],'HEIGHT_CLEARANCE':i['designClearance'],'CAPTURE_RADIUS':i['rollerRadius'],'RETENTION_TOP':i['retentionRollerOffset'][2]+i['retentionRollerRadius'],'SHAFT_RADIUS':i['shaftRadius'],'ARM_HALF':i['armSection'][1]/2,'SLEEVE_HALF':i['sleeveSection'][1]/2,'LINK_TOP':i['linkNormalOffset']+i['linkSection'][1],'LINK_OFFSET':i['linkNormalOffset'],'FLOOR_DEG':i['floorAngleDeg'],'STOW_DEG':i['stowAngleDeg'],'SHOOTER_X':s['pivot'][0],'SHOOTER_Z':s['pivot'][2],'GUIDE_LENGTH':s['guideLength'],'SHOT_PITCH_DEG':s['releasePitchDeg']}.items():text+=' public static final double '+name+'='+repr(float(value))+';\n'
text+=' public static final double[] FLOOR_PIECE_LOCAL='+array(j['transport']['floorPieceCenter'])+';\n'
text+=' public static final double[][] TRANSFER={'+','.join(array(v) for v in j['transport']['transferAndFeed'])+'};\n}\n'
out=ROOT/'shared/examples/java-2026-sim/src/main/java/org/agentfrc/lesson/A2MissionGeometry.java'
a=argparse.ArgumentParser();a.add_argument('--check',action='store_true');v=a.parse_args()
if v.check:
 if out.read_text()!=text:raise SystemExit('Generated geometry differs from pinned contract')
else:out.write_text(text)

"""Verify saved coupled-drive experiments and their exact executable inputs."""
import argparse
import hashlib
import json
import math
import pathlib
import xml.etree.ElementTree as ET


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify(packet, project):
    names = ['forward', 'turn', 'half-step', 'finer-torus', 'coast', 'reaction', 'no-contact', 'low-friction']
    runs = {name: json.loads((packet/name/'receipt.json').read_text(encoding='utf-8')) for name in names}
    reference = runs['forward']
    freefall_error = 0.
    power_error = 0.
    for name, run in runs.items():
        assert run['complete'] and not run['acceptance'] and not any(run['warningCounts'])
        for key in ('nativeArchiveSha256', 'scriptSha256', 'inputs', 'mujocoVersion'):
            assert run[key] == reference[key], (name, key)
        assert digest(packet/name/'trace.jsonl') == run['traceSha256']
        assert digest(packet/name/'model.xml') == run['modelSha256']
        assert run['maxima']['forceProjectionErrorN'] < 1e-10
        assert run['maxima']['coneExcessN'] < 1e-8
        assert run['maxima']['currentA'] <= run['parameters']['currentLimitA']
        xml = ET.fromstring((packet/name/'model.xml').read_text(encoding='utf-8'))
        assert xml.find("worldbody/body[@name='note']/freejoint") is not None
        assert len(xml.findall('equality/joint')) == 4
        assert all('wheel_' in x.get('joint1', '') and 'wheel_' in x.get('joint2', '') for x in xml.findall('equality/joint'))
        assert xml.find('actuator') is None
        for line in (packet/name/'trace.jsonl').read_text(encoding='utf-8').splitlines():
            row = json.loads(line)
            for motor, speed in zip(row['beltMotors'], row['beltSpeedMps']):
                power_error = max(power_error, abs(motor['terminalPowerW'] - motor['windingHeatW'] - motor['forceN'] * speed))
                assert abs(motor['currentA']) <= run['parameters']['currentLimitA']
                if run['parameters']['beltMode'] != 'coast':
                    assert abs(motor['terminalV']) <= run['parameters']['busV'] + 1e-10
            if name == 'no-contact':
                n = row['step']; dt = run['parameters']['dtS']
                freefall_error = max(freefall_error, abs(row['noteWorldM'][2] - (.301 - 9.80665*dt*dt*n*(n+1)/2)))
                assert row['contacts'] == 0
        ledger = run['energyLedgerJ']
        assert abs(ledger['beltTerminalJ'] - ledger['beltHeatJ'] - ledger['beltMechanicalJ']) < 1e-10
    assert digest(project/'sim/mujoco/native_drive_retention.py') == reference['scriptSha256']
    for rel, sha in reference['inputs'].items():
        assert digest(project/rel) == sha, rel
    for name in ('forward', 'turn', 'half-step', 'finer-torus'):
        run = runs[name]
        assert run['status'] == 'HORIZON' and run['final']['timeS'] > 1.49
        assert run['final']['chassisWorldM'][0] > .3
        assert run['maxima']['relativeXYM'] < .001 and run['maxima']['relativeZM'] < .001
        assert run['maxima']['beltStrokeM'] < .001 and run['maxima']['currentA'] > .1
        assert run['maxima']['chassisSpeedMps'] > .5 and abs(run['final']['qvel'][0]) < .05
        assert run['maxima']['relativeTiltRad'] < .01
    assert runs['turn']['maxima']['chassisYawRad'] > .03
    assert runs['coast']['status'] == 'OUTSIDE_STRAIGHT_PATCH_DOMAIN'
    assert runs['coast']['maxima']['beltStrokeM'] > .001 and runs['coast']['maxima']['currentA'] == 0.
    reaction = runs['reaction']
    assert reaction['status'] == 'HORIZON' and reaction['maxima']['comDisplacementM'] < 1e-12
    assert reaction['final']['noteWorldM'][0] > reaction['initial']['noteWorldM'][0]
    assert reaction['final']['chassisWorldM'][0] < reaction['initial']['chassisWorldM'][0]
    for name in ('no-contact', 'low-friction'):
        assert runs[name]['status'] == 'DROPPED_50MM' and runs[name]['final']['timeS'] < .3
    assert freefall_error < 1e-12 and power_error < 1e-10
    delta_x = abs(runs['half-step']['final']['chassisWorldM'][0] - runs['forward']['final']['chassisWorldM'][0])
    assert delta_x < .001
    return {'schema': 'native-drive-retention-controls/1', 'allChecksPassed': True, 'acceptance': False, 'freefallPositionErrorM': freefall_error, 'motorPowerIdentityMaxErrorW': power_error, 'halfStepFinalChassisXDifferenceM': delta_x, 'runs': runs, 'limits': ['Local contact and motor/drivetrain coupling evidence, not complete native hard-part clearance or a whole robot mission', 'Preserved initial-pose and mass/inertia hypotheses, fixed carry height and short belt patches; no acquisition/lift/launcher', 'Power identity checks do not validate all compliant contact energy or hardware calibration']}


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--packet', type=pathlib.Path, required=True)
    p.add_argument('--project', type=pathlib.Path, required=True)
    p.add_argument('--output', type=pathlib.Path, required=True)
    args = p.parse_args()
    assert not args.output.exists()
    result = verify(args.packet, args.project)
    args.output.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8', newline='\n')
    print(json.dumps({k:v for k,v in result.items() if k != 'runs'}))

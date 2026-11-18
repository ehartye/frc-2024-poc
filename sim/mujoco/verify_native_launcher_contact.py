"""Check saved experiments without equating a passing integrity audit with validation.

The manifest selects exact run paths and archived executable versions in a private
packet. It does not download CAD, contact Fusion, or step the simulation.
"""
import argparse
import hashlib
import json
import math
import pathlib
import xml.etree.ElementTree as ET


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify(packet, project, manifest_path, binding_path):
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    binding = json.loads(binding_path.read_text(encoding='utf-8'))
    sources = {sha(packet / rel): rel for rel in manifest['executedSources']}
    results = {}
    for label, rel in manifest['runs'].items():
        folder = packet / rel
        receipt = json.loads((folder / 'receipt.json').read_text(encoding='utf-8'))
        assert receipt['complete'] and receipt['acceptance'] is False
        assert receipt['scriptSha256'] in sources, (label, 'missing executed source')
        assert receipt['bindingSha256'] == sha(binding_path)
        assert receipt['nativeArchiveSha256'] == binding['nativeArchiveSha256']
        for name, digest in receipt['inputs'].items():
            assert sha(project / name) == digest, (label, name)
        for name, key in [('trace.jsonl', 'traceSha256'), ('model.xml', 'modelSha256')]:
            assert sha(folder / name) == receipt[key], (label, key)
        xml = ET.fromstring((folder / 'model.xml').read_text(encoding='utf-8'))
        assert xml.find("worldbody/body[@name='note']/freejoint") is not None
        constraints = xml.findall('equality/joint')
        assert len(constraints) == 4
        assert all('wheel_' in x.get('joint1', '') and 'wheel_' in x.get('joint2', '') for x in constraints)
        assert xml.find('actuator') is None and not any(receipt['warningCounts'])
        assert receipt['maxConeExcessN'] < 1e-8
        assert receipt['maxForceProjectionErrorN'] < 1e-8
        params = receipt['parameters']
        assert receipt['maxPhaseCurrentA'] <= params['currentLimitA'] + 1e-10
        rows = [json.loads(line) for line in (folder / 'trace.jsonl').read_text(encoding='utf-8').splitlines()]
        assert rows[0]['step'] == 0 and rows[-1] == receipt['final']
        dt = params['dtS']
        gravity = [float(x) for x in xml.find('option').get('gravity').split()]
        power_error = supply_error = motor_error = freefall_error = com_drift = 0.
        for row in rows:
            assert abs(row['timeS'] - row['step'] * dt) < 1e-10
            motors = row['launcherMotors']
            terminal_power = sum(x['terminalPowerW'] for x in motors)
            power_error = max(power_error, abs(row['busV'] * row['busCurrentA'] - terminal_power - row['driveBusPowerW']))
            supply_error = max(supply_error, abs(params['openCircuitV'] - row['busV'] - params['sourceResistanceOhm'] * row['busCurrentA']))
            assert abs(row['supplyPowerResidualW']) < 1e-6
            for motor in motors:
                motor_error = max(motor_error, abs(motor['terminalPowerW'] - motor['heatW'] - motor['shaftPowerW']))
                assert abs(motor['currentA']) <= params['currentLimitA'] + 1e-10
                if motor['currentA']:
                    assert motor['terminalV'] <= row['busV'] + 1e-10
            if params['noContact']:
                assert row['contacts'] == 0 and max(map(abs, row['noteForceWorldN'])) == 0
                n = row['step']
                predicted = [receipt['initial']['notePositionM'][i] + receipt['initial']['noteVelocityMps'][i] * n * dt + gravity[i] * dt * dt * n * (n + 1) / 2 for i in range(3)]
                freefall_error = max(freefall_error, math.dist(predicted, row['notePositionM']))
            if params['floating']:
                com_drift = max(com_drift, math.dist(receipt['initial']['systemComM'], row['systemComM']))
        assert power_error < 1e-6 and supply_error < 1e-8 and motor_error < 1e-8
        assert freefall_error < 1e-10 and com_drift < 1e-10
        # Once the final contact is over, translation must follow the engine's
        # semi-implicit Euler free-flight solution without a fitted trajectory.
        last = receipt['lastContact']
        tail = [r for r in rows if last is None or r['step'] > last['step']]
        ballistic_error = 0.
        if len(tail) > 1:
            origin = tail[0]
            for row in tail:
                assert row['contacts'] == 0
                n = row['step'] - origin['step']
                predicted = [origin['notePositionM'][i] + origin['noteVelocityMps'][i] * n * dt + gravity[i] * dt * dt * n * (n + 1) / 2 for i in range(3)]
                ballistic_error = max(ballistic_error, math.dist(predicted, row['notePositionM']))
            assert ballistic_error < 1e-9, (label, ballistic_error)
        energy = receipt['energyLedgerJ']
        assert abs(energy['launcherTerminalJ'] - energy['launcherHeatJ'] - energy['launcherShaftJ']) < 1e-8
        assert abs(energy['sourceJ'] - energy['sourceHeatJ'] - energy['busJ']) < 1e-7
        expected_rotor_energy = 4 * .5 * params['rotatingInertiaEachKgM2'] * (params['initialRpm'] * 2 * math.pi / 60) ** 2
        assert abs(receipt['initialRotorKineticJ'] - expected_rotor_energy) < 1e-10
        results[label] = {
            'receiptSha256': sha(folder / 'receipt.json'), 'scriptSha256': receipt['scriptSha256'],
            'traceSha256': receipt['traceSha256'], 'modelSha256': receipt['modelSha256'],
            'status': receipt['status'], 'parameters': params,
            'lastContactTimeS': None if last is None else last['timeS'],
            'lastContactSpeedMps': None if last is None else math.dist(last['noteVelocityMps'], [0., 0., 0.]),
            'finalAdvanceM': receipt['final']['advanceM'],
            'finalContactCount': receipt['final']['contacts'],
            'finalChassisXM': receipt['final']['chassisPositionM'][0],
            'minimumBusV': receipt['minimumBusV'], 'initialRotorKineticJ': receipt['initialRotorKineticJ'],
            'motorPowerIdentityErrorW': motor_error, 'busPowerIdentityErrorW': power_error,
            'sourceVoltageBalanceErrorV': supply_error, 'freefallPositionErrorM': freefall_error,
            'freeFlightPositionErrorM': ballistic_error, 'floatingComDriftM': com_drift,
            'maximumNoteWorkResidualJ': receipt['maxNoteWorkResidualJ'],
            'axialEdgeContactSamples': receipt['axialEdgeContactSamples'],
            'maxPenetrationM': receipt['maxPenetrationM'],
        }
    base = results['controls/rest-launch']
    half = results['controls/half-step']
    speed_delta = abs(base['lastContactSpeedMps'] - half['lastContactSpeedMps'])
    assert speed_delta < .01
    assert half['maximumNoteWorkResidualJ'] < .6 * base['maximumNoteWorkResidualJ']
    assert results['controls/drive-launch']['finalChassisXM'] > .015
    assert results['controls/drive-launch']['minimumBusV'] < base['minimumBusV']
    for label in ('controls/zero-drive', 'controls/zero-friction'):
        assert results[label]['status'] == 'HORIZON' and results[label]['finalAdvanceM'] < -.03
    assert results['controls/no-contact']['status'] == 'REVERSED_80MM'
    assert results['controls/pre-spun']['initialRotorKineticJ'] > 21
    capsule_speeds = [results['matched/grid-' + str(n)]['lastContactSpeedMps'] for n in (128, 256, 512, 1024)]
    assert capsule_speeds[-1] < .75 * capsule_speeds[0], 'Expected numerical limitation changed: inspect before updating evidence'
    sdf_labels = ['sdf-matched/points' + str(n) for n in (40, 80, 160)]
    if all(label in results for label in sdf_labels):
        assert results[sdf_labels[0]]['finalContactCount'] == 0
        assert all(results[label]['finalContactCount'] > 0 for label in sdf_labels[1:])
        sdf_disagreement = True
    else:
        sdf_disagreement = None
    return {
        'schema': 'native-launcher-contact-controls/1', 'integrityChecksPassed': True,
        'acceptance': False, 'speedPredictionAccepted': False,
        'manifestSha256': sha(manifest_path), 'bindingSha256': sha(binding_path),
        'nativeArchiveSha256': binding['nativeArchiveSha256'],
        'verifierSha256': sha(pathlib.Path(__file__)), 'runCount': len(results),
        'halfStepSpeedDifferenceMps': speed_delta,
        'matchedCapsuleSpeedMps1282565121024': capsule_speeds,
        'sdfSearchReleaseDisagreement': sdf_disagreement,
        'runs': results,
        'limits': [
            'Passing integrity, controls and balance checks does not establish contact-model convergence or hardware validity',
            'Initial staged NOTE, fixed launcher pose, constrained chassis pitch/roll; no connected acquisition/lift/shoot cycle',
            'Contact force, compliance, friction, inertia and electrical parameters are uncalibrated hypotheses',
            'Full native hard-part clearance and continuous collision clearance are still unresolved',
            'NOTE work residual is a numerical diagnostic, not a full compliant-contact energy proof',
        ],
    }


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    for name in ('packet', 'project', 'manifest', 'binding', 'output'):
        parser.add_argument('--' + name, type=pathlib.Path, required=True)
    args = parser.parse_args()
    assert not args.output.exists()
    result = verify(args.packet, args.project, args.manifest, args.binding)
    args.output.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8', newline='\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'runs'}))

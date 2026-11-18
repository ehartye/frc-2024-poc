"""Bounded native-backed drivetrain and finite straight-belt contact experiment.

Not a closed-loop belt, acquisition, cradle-motion, launcher or hardware model.
"""
import argparse
import hashlib
import json
import math
import pathlib
import sys
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def build(root, archive, dt, segments, mu, no_contact=False, floating=False):
    sys.path.insert(0, str(root / 'sim/mujoco'))
    from native_drive import NativeDrive
    from native_static_contact import make_model, contact_result
    source_path = root / 'sim/integrated/evidence/track-tread-candidate-20261118.json'
    source = json.loads(source_path.read_text(encoding='utf-8'))
    assert digest(archive) == source['native']['finalArchiveSha256']
    cp = root / 'cad/assembly/intake_components.json'
    config = json.loads(cp.read_text(encoding='utf-8'))
    assert digest(cp) == source['staticCapacity']['onePointFiveMm']['inputs']['cad/assembly/intake_components.json']
    parameters = source['native']['parameters']
    target = next(x['normalN'][0] for x in source['staticCapacity']['onePointFiveMm']['rows'] if x['resolution'] == 2048 and x['noteXmm'] == 185)
    trial, td, _ = make_model(config, parameters, segments, dt, 0, 1000, 0, gravity=False)
    stiffness = 1000 * target / (sum(contact_result(trial, td)['normalN']) / 2)
    _, _, contact_xml = make_model(config, parameters, segments, dt, mu, stiffness, 20, collision=not no_contact)
    drive = NativeDrive(root, dt=dt)
    baseline_com = np.sum(drive.m.body_mass[:, None] * drive.d.xipos, axis=0) / sum(drive.m.body_mass)
    baseline_yaw = sum(drive.m.body_inertia[i, 2] + drive.m.body_mass[i] * sum(drive.d.xipos[i, :2]**2) for i in range(1, drive.m.nbody))
    tree = ET.fromstring(drive.xml)
    tree.set('model', 'native-backed bounded drive retention')
    option = tree.find('option')
    option.set('cone', 'elliptic')
    option.set('noslip_iterations', '20')
    option.set('noslip_tolerance', '1e-12')
    if floating:
        option.set('gravity', '0 0 0')
    world = tree.find('worldbody')
    floor = world.find('geom')
    floor.set('contype', '0' if floating else '1')
    floor.set('conaffinity', '0' if floating else '1')
    chassis = world.find("body[@name='chassis']")
    for geom in chassis.findall('.//geom'):
        geom.set('contype', '1')
        geom.set('conaffinity', '1')
    contact_tree = ET.fromstring(contact_xml)
    contact_world = contact_tree.find('worldbody')
    belt_mass = .05
    rotor_inertia = 3e-5
    radius = config['pinch']['teeth'] * config['pinch']['pitch'] / (2 * math.pi * 1000)
    # Redistribute 0.1 kg from the existing 45 kg robot hypothesis; preserve its
    # initial total COM and yaw inertia. These are not measured belt properties.
    inertial = chassis.find('inertial')
    old_mass = float(inertial.get('mass'))
    old_pos = np.fromstring(inertial.get('pos'), sep=' ')
    old_i = np.fromstring(inertial.get('diaginertia'), sep=' ')
    positions = []
    local_yaw_inertias = []
    for i, geom in enumerate(contact_world.findall('geom')):
        name = f'belt_{i}'
        pos = np.fromstring(geom.get('pos'), sep=' ')
        half = np.fromstring(geom.get('size'), sep=' ')
        inertia = belt_mass / 3 * np.array([half[1]**2 + half[2]**2, half[0]**2 + half[2]**2, half[0]**2 + half[1]**2])
        positions.append(pos)
        local_yaw_inertias.append(inertia[2])
        body = ET.SubElement(chassis, 'body', name=name, pos=' '.join(map(str, pos)))
        ET.SubElement(body, 'joint', name=name, type='slide', axis='1 0 0', armature=str(rotor_inertia / radius**2))
        ET.SubElement(body, 'inertial', pos='0 0 0', mass=str(belt_mass), diaginertia=' '.join(map(str, inertia)))
        geom.set('name', name)
        geom.set('pos', '0 0 0')
        geom.set('conaffinity', '4')
        geom.set('density', '0')
        body.append(geom)
    new_mass = old_mass - 2 * belt_mass
    new_pos = (old_mass * old_pos - belt_mass * np.sum(positions, axis=0)) / new_mass
    old_yaw_origin = old_i[2] + old_mass * sum(old_pos[:2]**2)
    new_yaw = old_yaw_origin - new_mass * sum(new_pos[:2]**2) - sum(iz + belt_mass * sum(p[:2]**2) for iz, p in zip(local_yaw_inertias, positions))
    inertial.set('mass', str(new_mass))
    inertial.set('pos', ' '.join(map(str, new_pos)))
    inertial.set('diaginertia', f'{old_i[0]} {old_i[1]} {new_yaw}')
    note = contact_world.find('body')
    note.set('pos', '.185 0 .301')
    for geom in note.findall('geom'):
        geom.set('contype', '0' if no_contact else '4')
        geom.set('conaffinity', '0' if no_contact else '2')
    world.append(note)
    xml = ET.tostring(tree, encoding='unicode')
    drive.xml = xml
    drive.m = mujoco.MjModel.from_xml_string(xml)
    drive.d = mujoco.MjData(drive.m)
    drive.q = {name: int(drive.m.joint(name).qposadr[0]) for name in drive.q}
    drive.v = {name: int(drive.m.joint(name).dofadr[0]) for name in drive.v}
    assert drive.m.nv == 18 and drive.m.neq == 4 and drive.m.nu == 0
    assert abs(sum(drive.m.body_mass) - 45.235) < 1e-12
    mujoco.mj_forward(drive.m, drive.d)
    note_id = drive.m.body('note').id
    robot_com = sum(drive.m.body_mass[i] * drive.d.xipos[i] for i in range(1, drive.m.nbody) if i != note_id) / 45.
    assert np.max(np.abs(robot_com - baseline_com)) < 1e-12
    robot_yaw = sum(drive.m.body_inertia[i, 2] + drive.m.body_mass[i] * sum(drive.d.xipos[i, :2]**2) for i in range(1, drive.m.nbody) if i != note_id)
    assert abs(robot_yaw - baseline_yaw) < 1e-12
    pins = dict(drive.pins)
    for rel in ['sim/mujoco/native_drive.py', 'sim/mujoco/drivetrain_bridge.py', 'sim/mujoco/native_static_contact.py', 'cad/assembly/intake_components.json', 'sim/integrated/evidence/track-tread-candidate-20261118.json', 'sim/acquisition/free-planar-config.json']:
        pins[rel] = digest(root / rel)
    motor = json.loads((root / 'sim/acquisition/free-planar-config.json').read_text(encoding='utf-8'))['model']
    assert motor['rotorInertiaKgM2'] == rotor_inertia and abs(motor['radiusM'] - radius) < 1e-14
    return drive, pins, {'stiffness': stiffness, 'normalTargetPerSideN': target, 'radiusM': radius, 'beltMassEachKg': belt_mass, 'rotorInertiaEachKgM2': rotor_inertia, 'motorK': 1 / motor['kvRadpsPerV'], 'resistanceOhm': motor['motorResistanceOhm'], 'currentLimitA': motor['currentLimitA'], 'positionGainNpm': 1000., 'velocityGainNspm': 20., 'busV': 12.}


def belt_query(mode, q, v, parameters):
    p = parameters
    omega = v / p['radiusM']
    emf = p['motorK'] * omega
    if mode == 'coast':
        current = 0.
        terminal = emf  # Open-circuit back EMF; zero current and terminal power.
    else:
        desired = -p['positionGainNpm'] * q - p['velocityGainNspm'] * v
        terminal = 0. if mode == 'brake' else float(np.clip(emf + desired * p['radiusM'] * p['resistanceOhm'] / p['motorK'], -p['busV'], p['busV']))
        current = float(np.clip((terminal - emf) / p['resistanceOhm'], -p['currentLimitA'], p['currentLimitA']))
        terminal = emf + current * p['resistanceOhm']
        assert abs(terminal) <= p['busV'] + 1e-9
    force = p['motorK'] * current / p['radiusM']
    power = terminal * current
    heat = current**2 * p['resistanceOhm']
    residual = power - heat - force * v
    assert abs(residual) < 1e-10
    return {'forceN': force, 'currentA': current, 'terminalV': terminal, 'terminalPowerW': power, 'windingHeatW': heat, 'mechanicalPowerW': force * v, 'powerResidualW': residual}


def reaction_pulse(t, v, p):
    # A small current-controlled internal pulse checks action/reaction with no
    # floor, gravity or drive input. No NOTE or chassis external forces exist.
    current = .12 if t < .05 else 0.
    emf = p['motorK'] * v / p['radiusM']
    terminal = emf + current * p['resistanceOhm']
    force = p['motorK'] * current / p['radiusM']
    power = terminal * current
    heat = current**2 * p['resistanceOhm']
    assert abs(current) <= p['currentLimitA'] and abs(terminal) <= p['busV']
    return {'forceN': force, 'currentA': current, 'terminalV': terminal, 'terminalPowerW': power, 'windingHeatW': heat, 'mechanicalPowerW': force * v, 'powerResidualW': power-heat-force*v}


def contact_observe(model, data):
    note = model.body('note').id
    chassis = model.body('chassis').id
    rotation = data.xmat[chassis].reshape(3, 3)
    relative = rotation.T @ (data.xpos[note] - data.xpos[chassis])
    force = np.zeros(3)
    generalized = np.zeros(model.nv)
    normals = [0., 0.]
    counts = 0
    cone = 0.
    end_margin = float('inf')
    for i in range(data.ncon):
        c = data.contact[i]
        g1, g2 = map(int, c.geom)
        b1, b2 = map(int, model.geom_bodyid[[g1, g2]])
        if note not in (b1, b2):
            continue
        other = b1 if b2 == note else b2
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, other)
        assert name in ('belt_0', 'belt_1')
        raw = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, raw)
        f = (1 if b2 == note else -1) * c.frame.reshape(3, 3).T @ raw[:3]
        force += f
        mujoco.mj_applyFT(model, data, f, np.zeros(3), c.pos, note, generalized)
        normals[int(name[-1])] += raw[0]
        belt_geom = g1 if b1 == other else g2
        local_contact = data.xmat[other].reshape(3, 3).T @ (c.pos - data.xpos[other])
        end_margin = min(end_margin, float(model.geom_size[belt_geom, 0] - abs(local_contact[0])))
        cone = max(cone, math.hypot(raw[1], raw[2]) - c.friction[0] * raw[0])
        counts += 1
    adr = int(model.jnt_dofadr[model.body_jntadr[note]])
    projection_error = float(np.max(np.abs(generalized[adr:adr+6] - data.qfrc_constraint[adr:adr+6])))
    assert projection_error < 1e-8
    com = np.sum(model.body_mass[:, None] * data.xipos, axis=0) / sum(model.body_mass)
    tilt = math.acos(float(np.clip(np.dot(data.xmat[note].reshape(3, 3)[:, 2], rotation[:, 2]), -1., 1.)))
    return {'relativeNoteM': relative.tolist(), 'relativeTiltRad': tilt, 'noteWorldM': data.xpos[note].tolist(), 'chassisWorldM': data.xpos[chassis].tolist(), 'chassisYawRad': float(data.qpos[3]), 'noteForceWorldN': force.tolist(), 'normalN': normals, 'contacts': counts, 'minimumContactEndMarginM': None if not counts else end_margin, 'frictionConeExcessN': cone, 'forceProjectionErrorN': projection_error, 'systemComM': com.tolist()}


def command(t, scenario):
    if scenario in ('stationary', 'reaction') or t < .25:
        return (0., 0.), ('coast', 'coast')
    if t < .65:
        return (.15, .15), ('drive', 'drive')
    if t < .9:
        return (.12, .06) if scenario == 'turn' else (.15, .15), ('drive', 'drive')
    return (0., 0.), ('brake', 'brake')


def run(args):
    root = args.project.resolve()
    drive, pins, parameters = build(root, args.native_archive, args.dt, args.segments, args.mu, args.no_contact, args.scenario == 'reaction')
    m, d = drive.m, drive.d
    indices = [(int(m.joint(f'belt_{i}').qposadr[0]), int(m.joint(f'belt_{i}').dofadr[0])) for i in range(2)]
    assert not args.output.exists()
    args.output.mkdir(parents=True)
    (args.output / 'model.xml').write_text(drive.xml + '\n', encoding='utf-8', newline='\n')
    maxima = dict(relativeXYM=0., relativeZM=0., relativeTiltRad=0., beltStrokeM=0., currentA=0., forceProjectionErrorN=0., coneExcessN=0., comDisplacementM=0., chassisSpeedMps=0., chassisYawRad=0., noteForceN=0.)
    ledger = dict(beltTerminalJ=0., beltHeatJ=0., beltMechanicalJ=0., driveAxleJ=0.)
    initial = contact_observe(m, d)
    min_normal = [float('inf'), float('inf')]
    start = time.perf_counter()
    status = 'HORIZON'
    with (args.output / 'trace.jsonl').open('x', encoding='utf-8', newline='\n') as trace:
        for step in range(round(args.duration / args.dt) + 1):
            demands, modes = command(float(d.time), args.scenario)
            electrical, torque, omega = drive.motor_query(demands, modes)
            d.qfrc_applied[:] = 0.
            for side in range(2):
                d.qfrc_applied[drive.v[f'wheel_{side}_1']] = torque[side]
            motors = []
            for qadr, vadr in indices:
                motor = belt_query(args.belt_mode, float(d.qpos[qadr]), float(d.qvel[vadr]), parameters)
                if args.scenario == 'reaction':
                    motor = reaction_pulse(float(d.time), float(d.qvel[vadr]), parameters)
                d.qfrc_applied[vadr] = motor['forceN']
                motors.append(motor)
            mujoco.mj_forward(m, d)
            observed = contact_observe(m, d)
            relative = np.array(observed['relativeNoteM']) - np.array([.185, 0., .3])
            strokes = [float(d.qpos[q]) for q, v in indices]
            maxima['relativeXYM'] = max(maxima['relativeXYM'], float(np.linalg.norm(relative[:2])))
            maxima['relativeZM'] = max(maxima['relativeZM'], abs(float(relative[2])))
            maxima['relativeTiltRad'] = max(maxima['relativeTiltRad'], observed['relativeTiltRad'])
            maxima['beltStrokeM'] = max(maxima['beltStrokeM'], *map(abs, strokes))
            maxima['currentA'] = max(maxima['currentA'], *(abs(x['currentA']) for x in motors))
            maxima['forceProjectionErrorN'] = max(maxima['forceProjectionErrorN'], observed['forceProjectionErrorN'])
            maxima['coneExcessN'] = max(maxima['coneExcessN'], observed['frictionConeExcessN'])
            maxima['comDisplacementM'] = max(maxima['comDisplacementM'], float(np.linalg.norm(np.array(observed['systemComM']) - initial['systemComM'])))
            maxima['chassisSpeedMps'] = max(maxima['chassisSpeedMps'], math.hypot(float(d.qvel[drive.v['x']]), float(d.qvel[drive.v['y']])))
            maxima['chassisYawRad'] = max(maxima['chassisYawRad'], abs(observed['chassisYawRad']))
            maxima['noteForceN'] = max(maxima['noteForceN'], float(np.linalg.norm(observed['noteForceWorldN'])))
            min_normal = [min(a, b) for a, b in zip(min_normal, observed['normalN'])]
            row = {'step': step, 'timeS': float(d.time), **observed, 'beltStrokeM': strokes, 'beltSpeedMps': [float(d.qvel[v]) for q, v in indices], 'beltMotors': motors, 'driveDemand': demands, 'driveMode': modes, 'driveBusPowerW': electrical['busPowerW'], 'qpos': d.qpos.tolist(), 'qvel': d.qvel.tolist()}
            if relative[2] < -.05:
                status = 'DROPPED_50MM'
            # Stay within the earlier straight-fixture XY domain. The contact
            # point margin is twice maximum material travel, and far above the
            # 128-segment 0.046 mm centerline approximation error. It is a local
            # contact guard, not a continuous clearance proof for the full CAD.
            elif max(map(abs, strokes)) > .001 or np.linalg.norm(relative[:2]) > .001 or observed['relativeTiltRad'] > .01 or observed['minimumContactEndMarginM'] is not None and observed['minimumContactEndMarginM'] < .002:
                status = 'OUTSIDE_STRAIGHT_PATCH_DOMAIN'
            if step % 10 == 0 or status != 'HORIZON' or step == round(args.duration / args.dt):
                trace.write(json.dumps(row) + '\n')
            if status != 'HORIZON' or step == round(args.duration / args.dt):
                break
            ledger['beltTerminalJ'] += args.dt * sum(x['terminalPowerW'] for x in motors)
            ledger['beltHeatJ'] += args.dt * sum(x['windingHeatW'] for x in motors)
            ledger['beltMechanicalJ'] += args.dt * sum(x['mechanicalPowerW'] for x in motors)
            ledger['driveAxleJ'] += args.dt * sum(a*b for a, b in zip(torque, omega))
            mujoco.mj_step(m, d)
            assert np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all()
    warning_counts = [int(w.number) for w in d.warning]
    assert not any(warning_counts)
    result = {'schema': 'native-drive-retention-experiment/1', 'complete': True, 'acceptance': False, 'status': status, 'nativeArchiveSha256': digest(args.native_archive), 'inputs': pins, 'scriptSha256': digest(pathlib.Path(__file__)), 'mujocoVersion': mujoco.__version__, 'parameters': {**parameters, 'dtS': args.dt, 'segments': args.segments, 'mu': args.mu, 'scenario': args.scenario, 'beltMode': 'finite-current-pulse' if args.scenario == 'reaction' else args.belt_mode, 'noContact': args.no_contact, 'maximumPatchTravelM': .001, 'maximumNoteRelativeXYM': .001, 'minimumContactEndMarginM': .002}, 'maxima': maxima, 'minimumNormalN': min_normal, 'initial': initial, 'final': row, 'energyLedgerJ': ledger, 'warningCounts': warning_counts, 'traceSha256': digest(args.output / 'trace.jsonl'), 'modelSha256': digest(args.output / 'model.xml'), 'wallSeconds': time.perf_counter()-start, 'limits': ['Only native straight-face geometry; finite translational patches stop at 1 mm material travel/NOTE XY displacement or 2 mm contact endpoint margin, not full circulating belts', 'Cradle fixed at a hypothetical 0.3 m carry height; no moving lift, pitch, hard-part clearance or acquisition verification', 'Finite ideal reciprocal pinch motors with current and voltage limits; drive and pinch share ideal 12 V, no battery sag or regenerative-storage qualification', 'Mass/inertia/contact stiffness/friction/controller gains remain hypotheses; native archive identity does not calibrate them', 'No launcher, shot, field goal, hardware validation or GUI demonstration']}
    (args.output / 'receipt.json').write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8', newline='\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('inputs','initial','final','limits')}))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--project', type=pathlib.Path, required=True)
    parser.add_argument('--native-archive', type=pathlib.Path, required=True)
    parser.add_argument('--output', type=pathlib.Path, required=True)
    parser.add_argument('--dt', type=float, default=.0002)
    parser.add_argument('--segments', type=int, default=128)
    parser.add_argument('--mu', type=float, default=.6)
    parser.add_argument('--duration', type=float, default=1.5)
    parser.add_argument('--scenario', choices=['stationary','forward','turn','reaction'], default='forward')
    parser.add_argument('--belt-mode', choices=['hold','coast','brake'], default='hold')
    parser.add_argument('--no-contact', action='store_true')
    args = parser.parse_args()
    assert 0 < args.dt <= .0005 and 0 < args.duration <= 10 and args.segments >= 32 and args.segments % 4 == 0 and 0 <= args.mu <= 1
    run(args)

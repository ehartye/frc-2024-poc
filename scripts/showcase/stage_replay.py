"""Animate reopened R8 Fusion meshes alongside the analytic center-chain reference."""
import argparse
import json
import math
from pathlib import Path
import struct
import numpy as np

from build_model import build

ROOT = Path(__file__).resolve().parents[2]


def write(mesh_manifest, out):
    receipt = json.loads((ROOT/'cad/student/r8-reopened.json').read_bytes())
    screen = json.loads((ROOT/'experiments/stage-climb/static-screen.json').read_bytes())
    if json.loads(mesh_manifest.read_bytes())['sourceSha256'] != receipt['sha256'] or screen['nativeSha256'] != receipt['sha256']:
        raise ValueError('Mismatched native mesh or load study')
    build(mesh_manifest,out,False)
    raw = out.read_bytes()
    n = struct.unpack_from('<I',raw,12)[0]
    model = json.loads(raw[20:20+n])
    binary = bytearray(raw[28+n:])
    def accessor(data, shape):
        arr = np.asarray(data,dtype='<f4')
        binary.extend(bytes(-len(binary)%4))
        view = len(model['bufferViews'])
        model['bufferViews'].append(dict(buffer=0,byteOffset=len(binary),byteLength=arr.nbytes))
        binary.extend(arr.tobytes())
        row = dict(bufferView=view,componentType=5126,count=len(arr),type=shape)
        if shape in ('SCALAR','VEC3'):
            row.update(min=arr.reshape(len(arr),-1).min(axis=0).tolist(),max=arr.reshape(len(arr),-1).max(axis=0).tolist())
        model['accessors'].append(row)
        return len(model['accessors'])-1
    nodes = model['nodes']
    prior = list(model['scenes'][0]['nodes'])
    robot = len(nodes)
    nodes.append(dict(name='R8 robot about its floor datum',children=prior))
    scene = model['scenes'][0]['nodes'] = [robot]
    trace=json.loads((ROOT/'experiments/stage-climb/solo-trajectory.json').read_bytes())
    if trace['nativeSha256'] != receipt['sha256']:
        raise ValueError('Timed trajectory belongs to a different native archive')
    keys=trace['samples']['engaged']
    times=[p['tS'] for p in keys]
    t = accessor(times,'SCALAR')
    clip = dict(name='Take the Stage',samplers=[],channels=[],extras=dict(native=receipt['sha256'],
        poweredCutoffS=10,passiveEndS=15,basis='Illustrative center-chain sequence using reopened joint travel and quasi-static suspended pose'))
    model['animations'] = [clip]
    def channel(node,key,values):
        shape = 'VEC4' if key=='rotation' else 'VEC3'
        i = len(clip['samplers'])
        clip['samplers'].append(dict(input=t,output=accessor(values,shape),interpolation='LINEAR'))
        clip['channels'].append(dict(sampler=i,target=dict(node=node,path=key)))

    hook=np.array([.0005,-.030,.480])
    def gl(p): return [p[0],p[2],-p[1]]
    z=screen['stageChecks'][0]['loadedSeatMm']/1000
    low=screen['stageChecks'][0]['unloadedLowMm']/1000
    channel(robot,'translation',[gl(np.array(p['originMm'])/1000) for p in keys])
    channel(robot,'rotation',[[*gl(p['quaternionXYZW'][:3]),p['quaternionXYZW'][3]] for p in keys])
    for i in prior:
        if nodes[i]['name'].startswith('R8 sliding saddle'):
            channel(i,'translation',[[0,p['slideMm']/1000,0] for p in keys])
        if nodes[i]['name'].startswith('R8 transverse lock'):
            channel(i,'translation',[[p['pinWithdrawnMm']/1000,0,0] for p in keys])

    # Each gold cylinder follows a section of the modeled chain centerline.
    points,normals=[],[]
    for i in range(8):
        ang,bng=i*math.tau/8,(i+1)*math.tau/8
        a,b=[math.cos(ang),math.sin(ang)],[math.cos(bng),math.sin(bng)]
        for xy,depth in ((a,-.5),(b,-.5),(b,.5),(a,-.5),(b,.5),(a,.5)):
            points.append([xy[0]*.008,xy[1]*.008,depth])
            normals.append([*xy,0.])
    mat=len(model['materials'])
    model['materials'].append(dict(name='Analytic chain centerline',pbrMetallicRoughness=dict(
        baseColorFactor=[.91,.50,.09,1],metallicFactor=.15,roughnessFactor=.63)))
    mesh=len(model['meshes'])
    model['meshes'].append(dict(primitives=[dict(attributes=dict(POSITION=accessor(points,'VEC3'),
        NORMAL=accessor(normals,'VEC3')),material=mat)]))
    spec = screen['chainFit']
    half,anchor,a = (spec[k]/1000 for k in ('halfSpanMm','anchorApproxMm','catenaryAmm'))
    stations=np.linspace(-half,half,17)
    def endpoint(s,loaded):
        h = z+(anchor-z)*abs(s)/half if loaded else low+a*(math.cosh(s/a)-1)
        return np.array([hook[0],h,-hook[1]+s])
    for count,(left,right) in enumerate(zip(stations,stations[1:])):
        i=len(nodes)
        nodes.append(dict(name=f'Chain analytical segment {count+1}',mesh=mesh))
        scene.append(i)
        poses=[]
        for key in keys:
            fraction=(low-key['chainLowMm']/1000)/(low-z)
            def current(at): return (1-fraction)*endpoint(at,False)+fraction*endpoint(at,True)
            p,b=current(left),current(right)
            delta=b-p
            angle=-math.atan2(delta[1],delta[2])
            poses.append(((p+b)/2,[math.sin(angle/2),0,0,math.cos(angle/2)],[1,1,float(np.linalg.norm(delta))]))
        for j,key in enumerate(('translation','rotation','scale')):
            nodes[i][key]=list(poses[0][j])
            channel(i,key,[p[j] for p in poses])
    binary.extend(bytes(-len(binary)%4))
    model['buffers'][0]['byteLength']=len(binary)
    encoded=json.dumps(model,separators=(',',':'),allow_nan=False).encode()
    encoded+=b' '*(-len(encoded)%4)
    out.write_bytes(struct.pack('<4sIIII',b'glTF',2,28+len(encoded)+len(binary),len(encoded),0x4e4f534a)+
                   encoded+struct.pack('<II',len(binary),0x004e4942)+binary)


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('mesh_manifest',type=Path)
    args=parser.parse_args()
    write(args.mesh_manifest,ROOT/'docs/detailed-design/archive/draft-001/media/r8-stage.glb')

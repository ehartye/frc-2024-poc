"""Tessellate every first-party solid from the student archive in robot coordinates."""
from collections import defaultdict
from hashlib import sha256
import json
import struct

import adsk.core as core

from physical_bumper import registered_solid
from export_swerve import tessellate


def extract(root, digest, folder):
    folder.mkdir(exist_ok=True, parents=True)
    groups = defaultdict(bytearray)
    indices = defaultdict(list)
    inventory = []
    colors = {}
    for occ in root.allOccurrences:
        for body in occ.bRepBodies:
            key = str(len(inventory))
            name = occ.component.name + ' / ' + body.appearance.name
            groups[name].extend(tessellate(registered_solid(occ,body)))
            indices[name].append(key)
            inventory.append(dict(key=key, occurrence=occ.fullPathName, body=body.name))
            for prop_id in ('opaque_albedo','surface_albedo'):
                prop = core.ColorProperty.cast(body.appearance.appearanceProperties.itemById(prop_id))
                if prop:
                    c = prop.value
                    colors[name] = [c.red/255,c.green/255,c.blue/255,1]
                    break
    meshes = []
    for i,(name,facets) in enumerate(groups.items()):
        count,remainder = divmod(len(facets),50)
        if not count or remainder:
            raise ValueError(name)
        data = bytes(80)+struct.pack('<I',count)+facets
        file = f'assembly-{i}.stl'
        (folder/file).write_bytes(data)
        meshes.append(dict(name=name,file=file,triangles=count,sha256=sha256(data).hexdigest(),
                           rgba=colors[name],sourceBodyKeys=indices[name]))
    receipt = dict(schema='registered-student-export/1',sourceFormat='F3D',sourceSha256=digest,
                   units='mm',frame='robot-floor-XYZ',meshes=meshes,bodyInventory=inventory)
    (folder/'manifest.json').write_text(json.dumps(receipt,indent=2)+'\n',encoding='utf8')

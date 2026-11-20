"""Persistent physical owners for the 2024 archive, independent of Fusion body order."""
import json
from pathlib import Path
import re

HERE = Path(__file__).resolve().parent
SOURCE = json.loads((HERE/'source-bodies.json').read_bytes())


def key(text):
    return re.sub(r'[^A-Za-z0-9]+','-',text).strip('-').lower()


def identify(component, body):
    ordinal = re.match(r'R7 (\d+) ',body)
    identity = 'r7-body-'+ordinal[1] if ordinal else key(component)+'/'+key(body)
    name = SOURCE[ordinal[1]] if ordinal else body
    if ordinal and 487 <= int(ordinal[1]) < 503:
        return identity,None,name
    if component.startswith(('Front left ','Front right ','Rear left ','Rear right ')):
        return identity,'maxswerve-'+key(' '.join(component.split()[:2])),name
    if ordinal and 144 <= int(ordinal[1]) <= 161:
        return identity,'electrical-'+key(name),name
    if ordinal and 593 <= int(ordinal[1]) <= 611 and int(ordinal[1]) not in (595,596,597):
        return identity,identity,name
    if re.search(r'[- ](?:head|shaft)$',name) and any(x in name for x in ('bolt','bracket','yoke','pad backing','gusset')):
        return identity,key(re.sub(r'[- ](?:head|shaft)$',' screw',name)),name
    if 'slider clamp' in name:
        return identity,key(re.sub(r'clamp (screw|head) nominal','clamp screw',name)),name
    if 'tension jack' in name:
        return identity,key(re.sub(r'tension jack.*','tension jack screw',name)),name
    return identity,identity,name

"""Screen the changed fixed frame against the independent lift/pitch and archived pickup traces."""
import argparse
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'cad/native'),str(ROOT/'sim/pressure')]
from verify_swerve_travel import mechanism_clearance
from swerve_bumper_note import measure


def check(reopened,trace_root):
    r=json.loads(reopened.read_bytes())
    all_rows=r['rows']
    moving={'inventory':[dict(visible=True,role='physical',motionGroup=row['component'].split()[-1],boundsMm=row['boundsMm'])
                         for row in all_rows if row['component'] in ('R7 cradle','R7 carriage')]}
    obstacle=[]
    for row in all_rows:
        b=row['body'];group=row['component'];bb=row['boundsMm']
        i=int(b.split()[1]) if b.startswith('R7 ') else -1
        if i==592:
            # Partition the tube and two integral screw landings because the center is open.
            x0,lo,z0,x1,hi,_=bb
            for name,box in [('bottom',[x0,lo,z0,x1,hi,z0+3.175]),('top',[x0,lo,122.225,x1,hi,125.4]),
                             ('back',[x0,lo,77.775,x0+3.175,hi,122.225]),('front',[x1-3.175,lo,77.775,x1,hi,122.225])]:
                obstacle.append(('outer crossmember '+name,box))
            for y in (-235,235):
                obstacle.append(('outer landing',[x1-9.525,y-15,125.4,x1,y+15,160.]))
        elif group=='R9 removable bumper cores':
            obstacle.append((b+' plus cover allowance',[x+(-1.5 if i<3 else 1.5) for i,x in enumerate(bb)]))
        elif group in ('R9 bumper washers','R9 front frame') or (593<=i<=611):
            obstacle.append((b,bb))
    envelope=mechanism_clearance(moving,[bb for _,bb in obstacle])
    result=dict(archiveSha256=r['archiveSha256'],configuration='R9 lowered mechanical assembly',
        frontAndBumperSweep=envelope,pickupAndCarry={},scope='Prescribed R7 traces and independent mechanism range. R9 has no new mission replay.')
    result['assemblySweep']={}
    for group in ('R9 upper lift drive','R9 electrical supports','R9 mast attachment','R9 local breaker harness'):
        if not any(row['component']==group for row in all_rows):continue
        result['assemblySweep'][group]=mechanism_clearance(moving,[row['boundsMm'] for row in all_rows if row['component']==group])
    for name in ('D1','D2'):
        result['pickupAndCarry'][name]=measure(None,trace_root/f'plant-{name}-v3',trace_root/f'carry-{name}-v3',rows=obstacle)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for k in ('observations','trace_folder','output'):p.add_argument(k,type=Path)
    a=p.parse_args();a.output.write_text(json.dumps(check(a.observations,a.trace_folder),indent=2)+'\n',encoding='utf8',newline='\n')

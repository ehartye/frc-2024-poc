from pathlib import Path
p=Path('layout.py');s=p.read_text(encoding='utf8')
s=s.replace('park=125; pick=-32','park=125; pick=-50').replace('pivot=(185,0,356)','pivot=(185,0,487)\nslide=[]')
a=s.index('    outline=');b=s.index('    local(',a)
s=s[:a]+'''    outline=[(-141,146),(-110,146),(15,424),(165,424),(253,146),(286,146),(193,442),(-4,442)]
    add('Pivot A-frame · '+side,prism([(x,yp,z) for x,z in outline],(0,12,0)),'Pivot support')
    add('Slide base · '+side,box(-47,yp,442,273,12,7),'Pivot support')
    add('Slide guide · '+side,box(-45,yp+2,449,269,8,6),'Pivot support','mechanism','Profile rail envelope')
    truck=box(165,yp-1,455,40,14,54)
    add('Translating bearing block · '+side,cut(truck,cyl((185,yp-2,487),Y,17,16)),'Pivot support','mechanism','Linear truck and bearing housing')
    add('Pivot bearing · '+side,ann((185,yp-1,487),Y,17,9,14),'Pivot support','mechanism','Steel bearing envelope')
    slide.extend(['Translating bearing block · '+side,'Pivot bearing · '+side])
''' +s[b:]
s=s.replace('(185,260,356)','(185,261,487)').replace('(185,296,356)','(185,297,487)')
s=s.replace('for pos in (125,370,599):', '''slide.extend(['Pivot motor','Pivot reducer'])
add('Slide screw',cyl((-49,275,428),X,5,279),'Pivot support','mechanism','Steel lead screw')
add('Slide drive motor',cyl((-102,275,428),X,14,53),'Pivot support','electrical','Geared motor envelope')
for x in [-49,220]:
    add('Slide bearing mount '+str(x),cut(box(x,254,416,10,32,26),cyl((x,275,428),X,5,10)),'Pivot support')
add('Travel nut and clevis',cut(box(178,261,420,14,24,35),cyl((178,275,428),X,5,14)),'Pivot support','mechanism','Threaded nut and offset bracket')
slide.append('Travel nut and clevis')
for pos in (125,370,599):''')
s=s.replace("def centerpose(angle,s,n):\n    return mat(angle,*pivot)","""def pivot_x(angle):
    a=radians(angle);c=cos(a);v=sin(a)
    reach=max(-18*c,643*c)+max(44*v,-67*v)
    return min(185,634-reach)
def centerpose(angle,s,n):
    return mat(angle,pivot_x(angle),0,pivot[2])""")
p.write_text(s,encoding='utf8')
p=Path('travel.py');s=p.read_text(encoding='utf8')
s=s.replace("'Battery withdrawal']","'Battery withdrawal','Battery at supported service angle']")
s=s.replace("    return {'arm':mat(theta,*pivot)@inverse", "    if case==cases[5]:theta=80;piece=centerpose(theta,215,50.4);batt=mat(0,0,0,370*t)\n    return {'slide':mat(0,pivot_x(theta)-185,0,0),'arm':mat(theta,pivot_x(theta),0,pivot[2])@inverse")
s=s.replace("else 'piece' if n==held","else 'slide' if n in slide else 'piece' if n==held")
s=s.replace("case==cases[4] and n in ('Pack lead','Distribution lead')","case in cases[4:] and n in ('Pack lead','Distribution lead',held)")
s=s.replace("for n in moving+[held]:","for n in moving+([held] if held in ids else []):")
p.write_text(s,encoding='utf8')


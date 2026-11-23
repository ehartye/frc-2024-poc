from pathlib import Path
p=Path('stock.py');s=p.read_text()
s=s.replace('out={n:', '''# Machine a straight outside-return relief in the six side posts, leaving the mounting ends full.
for n in list(previous):
    if n.startswith(('Lid post','Climb yoke post')):
        sign=1 if n.endswith('left') else -1
        pocket=place(box(65,217.8 if sign>0 else -220.8,31,360,3,34),mat(park,*pivot))
        G[n]=cut(G[n],pocket)
        additions.append(n)
previous-=set(additions)
out={n:''')
p.write_text(s)


from pathlib import Path
p=Path(__file__).resolve().parent
file=p.parent.parent/'www/review.js';s=file.read_text(encoding='utf8')
for a,b in [("const k=current().key;","const k=current().fixture||current().key;"),("cs.key==='shot'","(cs.fixture||cs.key)==='shot'"),("cs.key==='climb'","(cs.fixture||cs.key)==='climb'"),("c.key!=='shot'","(c.fixture||c.key)!=='shot'")]:
    if a in s:
        assert s.count(a)==1
        s=s.replace(a,b)
    else:assert b in s
tmp=file.with_suffix('.next');tmp.write_text(s,encoding='utf8');tmp.replace(file)


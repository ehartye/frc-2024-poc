from pathlib import Path
p=Path(__file__).resolve().parents[2]/'www/review.js'
s=p.read_text(encoding='utf8')
def change(a,b):
    global s
    assert s.count(a)==1,a
    s=s.replace(a,b)
change("function poseAt(c,n,t){return c.series?", """function meshName(c,name){return c.fieldNotes?.[name]?'Chambered NOTE':name}
function displayedEntries(c){
 if(!c.fieldNotes)return entries;
 const mesh=data.bodies['Chambered NOTE'];
 return entries.filter(([name])=>name!=='Chambered NOTE').concat(Object.keys(c.fieldNotes).map(name=>[name,mesh]));
}
function poseAt(c,n,t){return c.fieldNotes?.[n]?.[t]||c.series?""")
change("const labels=[];for(const [name,b] of entries){","const labels=[];for(const [name,b] of displayedEntries(cs)){")
change("else if(!selNames.has(name))col=b.dim;","else if(!selNames.has(meshName(cs,name)))col=b.dim;")
change("paint(sceneGPU.mesh.get(name),m,col);","paint(sceneGPU.mesh.get(meshName(cs,name)),m,col);")
change("return entries.filter(([n])=>!cs.removed.includes(n)","return displayedEntries(cs).filter(([n])=>!cs.removed.includes(n)")
change("const b=data.bodies[n],s=shape(b),box=$('inspector')","const b=data.bodies[meshName(current(),n)],s=shape(b),box=$('inspector')")
change("text('span','COMPONENT INSPECTOR'),text('h2',n)","text('span',current().fieldNotes?.[n]?'FIELD PIECE':'COMPONENT INSPECTOR'),text('h2',n)")
p.write_text(s,encoding='utf8')


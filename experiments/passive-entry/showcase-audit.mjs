// Audit the surfaces actually delivered by the public GLBs in their initial viewer pose.
import {readFileSync, writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
const here = new URL('./',import.meta.url);
const read = path => readFileSync(new URL(path,here));
const json = path => JSON.parse(read(path));
const tolerance = .20;
const subtract = (a,b) => [a[0]-b[0],a[1]-b[1],a[2]-b[2]];
const add = (a,b) => [a[0]+b[0],a[1]+b[1],a[2]+b[2]];
const scale = (a,k) => [k*a[0],k*a[1],k*a[2]];
const dot = (a,b) => a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
const cross = (a,b) => [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const clamp = x => Math.max(0,Math.min(1,x));
function bounds(points) {
  const out=[Infinity,Infinity,Infinity,-Infinity,-Infinity,-Infinity];
  for(const p of points) for(let i=0;i<3;i++){out[i]=Math.min(out[i],p[i]);out[i+3]=Math.max(out[i+3],p[i]);}
  return out;
}
function apart(a,b) {
  return [0,1,2].some(i=>a[i]>b[i+3]+tolerance || b[i]>a[i+3]+tolerance);
}
const near = v=>dot(v,v)<=tolerance*tolerance;
function pointEdge(p,a,d){
  return near(subtract(subtract(p,a),scale(d,clamp(dot(subtract(p,a),d)/(dot(d,d)||1)))));
}
function inside(p,face) {
  return face.edges.every((e,i)=>dot(cross(e,subtract(p,face.p[i])),face.n)>=-1e-8);
}
function hitsFace(a,b,face) {
  if(!face.nn) return false;
  const s=dot(subtract(a,face.p[0]),face.n), t=dot(subtract(b,face.p[0]),face.n);
  if(s*s<=tolerance*tolerance*face.nn && inside(a,face)) return true;
  return s*t<=0 && s!==t && inside(add(a,scale(subtract(b,a),s/(s-t))),face);
}
function edgePair(a,u,b,v){
  const w=subtract(a,b), uu=dot(u,u), vv=dot(v,v), uv=dot(u,v);
  const det=uu*vv-uv*uv;
  if(det>1e-14*uu*vv){
    const s=(uv*dot(v,w)-vv*dot(u,w))/det, t=(uu*dot(v,w)-uv*dot(u,w))/det;
    if(s>=0&&s<=1&&t>=0&&t<=1 && near(subtract(add(w,scale(u,s)),scale(v,t)))) return true;
  }
  return pointEdge(a,b,v)||pointEdge(b,a,u);
}
function facePair(a,b) {
  if(apart(a.box,b.box)) return false;
  for(let i=0;i<3;i++) {
    if(hitsFace(a.p[i],a.p[(i+1)%3],b)||hitsFace(b.p[i],b.p[(i+1)%3],a)) return true;
    for(let j=0;j<3;j++) if(edgePair(a.p[i],a.edges[i],b.p[j],b.edges[j]))return true;
  }
  return false;
}
function tree(faces){
  const box=bounds(faces.flatMap(f=>[f.box.slice(0,3),f.box.slice(3)]));
  if(faces.length<=6) return {box,faces};
  const span=subtract(box.slice(3),box);const axis=span.indexOf(Math.max(...span));
  faces.sort((a,b)=>(a.box[axis]+a.box[axis+3])-(b.box[axis]+b.box[axis+3]));
  const half=Math.floor(faces.length/2);
  return {box,branches:[tree(faces.slice(0,half)),tree(faces.slice(half))]};
}
function adjacent(a,b){
  if(apart(a.box,b.box))return false;
  if(a.branches)return a.branches.some(child=>adjacent(b,child));
  if(b.branches)return b.branches.some(child=>adjacent(a,child));
  return a.faces.some(f=>b.faces.some(g=>facePair(f,g)));
}
class Sets {
  constructor(){this.parents=[];}
  add(){const i=this.parents.length;this.parents.push(i);return i;}
  root(i){while(this.parents[i]!==i){this.parents[i]=this.parents[this.parents[i]];i=this.parents[i];}return i;}
  join(i,j){this.parents[this.root(j)]=this.root(i);}
}
const bill=json('../../cad/assembly-readiness/mass/actual.json');
const registration=json('registered-boxes.json');
if(registration.sourceSha256!==bill.archiveSha256)throw Error('Wrong bounds register');
function nativeNames(box){
  return registration.boundsByNativeBillRowMm.flatMap((b,i)=>
    b.every((x,k)=>Math.abs(x-box[k])<=.21)?[bill.bodies[i].body]:[]);
}
function audit(file){
  const manifest=json('../../docs/detailed-design/archive/draft-001/media/'+file+'.json');
  const asset=manifest.assets.glb??manifest.assets.model, raw=read('../../'+asset.path.replace(/^docs\//,'docs/detailed-design/archive/draft-001/'));
  if(createHash('sha256').update(raw).digest('hex')!==asset.sha256)throw Error(asset.path);
  const size=raw.readUInt32LE(12),g=JSON.parse(raw.subarray(20,20+size)),data=raw.subarray(28+size);
  function array(i){
    const a=g.accessors[i],v=g.bufferViews[a.bufferView],w={SCALAR:1,VEC3:3,VEC4:4}[a.type];
    const Type={5126:Float32Array,5123:Uint16Array,5125:Uint32Array}[a.componentType];
    if(v.byteStride||!w||!Type||v.buffer!==0)throw Error('Unexpected accessor layout');
    return new Type(data.buffer,data.byteOffset+(v.byteOffset||0)+(a.byteOffset||0),w*a.count);
  }
  // The viewer pauses the first R7 clip at 0. R9 has only a parked pose.
  for(const c of g.animations?.[0].channels??[]){
    const sample=g.animations[0].samplers[c.sampler], values=array(sample.output);
    if(array(sample.input)[0]!==0 || sample.interpolation==='CUBICSPLINE')throw Error('Unsupported initial key');
    const path=c.target.path;
    g.nodes[c.target.node][path]=Array.from(values.slice(0,path==='rotation'?4:3));
  }
  const shells=[],excluded=[];
  function visit(index,parent){
    const n=g.nodes[index];
    if(n.name==='NOTE'||n.name==='Speaker measurement guides'){excluded.push(n.name);return;}
    if(n.matrix)throw Error('Unexpected matrix transform');
    const [x,y,z,r]=n.rotation??[0,0,0,1],q=[x,y,z],s=n.scale??[1,1,1];
    function world(p){
      const v=p.map((a,j)=>a*s[j]),turn=scale(cross(q,v),2);
      return parent(add(n.translation??[0,0,0],add(v,add(scale(turn,r),cross(q,turn)))));
    }
    if(n.mesh!==undefined) for(const prim of g.meshes[n.mesh].primitives){
      const pos=array(prim.attributes.POSITION),ix=array(prim.indices), vertices=[],ids=[],sets=new Sets(),keys=new Map();
      for(let k=0;k<pos.length;k+=3){
        const p=world(Array.from(pos.slice(k,k+3))),key=p.map(x=>Math.round(x*1e9)).join(',');
        if(!keys.has(key))keys.set(key,sets.add());
        ids.push(keys.get(key)); vertices.push([p[0]*1000,-p[2]*1000,p[1]*1000]);
      }
      for(let k=0;k<ix.length;k+=3){sets.join(ids[ix[k]],ids[ix[k+1]]);sets.join(ids[ix[k]],ids[ix[k+2]]);}
      const groups=new Map();
      for(let k=0;k<ix.length;k+=3){
        const tag=sets.root(ids[ix[k]]),p=[vertices[ix[k]],vertices[ix[k+1]],vertices[ix[k+2]]];
        if(!groups.has(tag))groups.set(tag,[]);
        const edges=p.map((v,j)=>subtract(p[(j+1)%3],v)),normal=cross(edges[0],edges[1]);
        groups.get(tag).push({p,edges,n:normal,nn:dot(normal,normal),box:bounds(p)});
      }
      for(const faces of groups.values()){
        const geometry=tree(faces), box=geometry.box;
        shells.push({node:n.name,geometry,box:box.map(v=>+v.toFixed(3)),names:nativeNames(box)});
      }
    }
    for(const child of n.children??[])visit(child,world);
  }
  for(const i of g.scenes[g.scene??0].nodes)visit(i,p=>p);
  const edges=[],sets=new Sets(); shells.forEach(()=>sets.add());
  for(let i=0;i<shells.length;i++) for(let j=0;j<i;j++){
    if(adjacent(shells[i].geometry,shells[j].geometry)){edges.push([j,i]);sets.join(j,i);}
  }
  const groups=new Map();
  shells.forEach((_,i)=>{const r=sets.root(i);if(!groups.has(r))groups.set(r,[]);groups.get(r).push(i);});
  const islands=[...groups.values()].sort((a,b)=>b.length-a.length||a[0]-b[0]);
  return {sourceSha256:manifest.sourceSha256,assetSha256:asset.sha256,excluded,shells:shells.map(({geometry,...s})=>s),edges,islands};
}
const receipt={nearSurfaceMm:tolerance,r7:audit('swerve-r7'),r9:audit('r9-assembly')};
writeFileSync(new URL('showcase-audit.json',here),JSON.stringify(receipt)+'\n');

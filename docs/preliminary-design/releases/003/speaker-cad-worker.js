// Select the blue speaker subtree from FIRST's AP203 assembly before tessellation.
// Assembly relations point from parent representation to child representation.
function speakerStep(source) {
  const records = [], names = new Map(), children = new Map(), products = new Map();
  const breps = new Map(), context = new Map();
  const ids = line => Array.from(line.matchAll(/#(\d+)/g), m => Number(m[1]));
  for (const match of source.matchAll(/^#(\d+)\s*=\s*([^;]+);/gm)) {
    const id = Number(match[1]), value = match[2].trim();
    records[id] = [match.index, match[0].length];
    if (value.startsWith('SHAPE_REPRESENTATION (')) {
      names.set(id, value.match(/'([^']*)'/)[1]);
    } else if (value.startsWith('( REPRESENTATION_RELATIONSHIP')) {
      const [parent, child] = ids(value);
      if (!children.has(parent)) children.set(parent, []);
      children.get(parent).push({id, child});
    } else if (value.startsWith('CONTEXT_DEPENDENT_SHAPE_REPRESENTATION')) {
      context.set(ids(value)[0], id);
    } else if (value.startsWith('SHAPE_REPRESENTATION_RELATIONSHIP')) {
      const [from] = ids(value);
      if (!breps.has(from)) breps.set(from, []);
      breps.get(from).push(id);
    } else if (value.startsWith('SHAPE_DEFINITION_REPRESENTATION')) {
      products.set(ids(value)[1], id);
    }
  }
  const target = [...names].find(([,n]) => n === 'GE-24300_Briweld Field Blue')?.[0];
  const roots = [...children.keys()].filter(id => children.get(id).some(e => e.child === target));
  if (!target || roots.length !== 1) throw Error('Blue speaker assembly registration changed');
  const seeds = new Set();
  const include = id => {
    if (seeds.has(id)) return;
    seeds.add(id);
    const product = products.get(id);
    if (product) seeds.add(product);
    for (const rep of breps.get(id) || []) seeds.add(rep);
    for (const edge of children.get(id) || []) {
      if (id === roots[0] && edge.child !== target) continue;
      const n = names.get(edge.child) || '';
      if (!children.has(edge.child) && !/^(GE-|FE-|42SMR$)|Truss|Grip_Wheel/.test(n)) continue;
      seeds.add(context.get(edge.id));
      include(edge.child);
    }
  };
  include(roots[0]);
  const seen = new Set(), stack = [...seeds], lines = [source.slice(0, source.indexOf('DATA;')+5)];
  while (stack.length) {
    const id = stack.pop();
    if (seen.has(id)) continue;
    seen.add(id);
    const record = records[id];
    if (!record) throw Error('Missing referenced STEP entity '+id);
    const line = source.slice(record[0], record[0]+record[1]);
    lines.push(line);
    for (const ref of ids(line)) if (!seen.has(ref)) stack.push(ref);
  }
  lines.push('ENDSEC;', 'END-ISO-10303-21;');
  return lines.join('\n');
}

const archiveUrl = 'https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/CRESCENDO2024Field-STEP.zip';
const archiveDigest = '87185d3bf324bb3f35787ed6be715caf6151f6a9e81794830b3f743c381b4f18';
const readerBase = 'https://cdn.jsdelivr.net/npm/occt-import-js@0.0.23/dist/';
async function verifiedBytes(url, expected) {
  const integrity = 'sha256-'+btoa(String.fromCharCode(...expected.match(/../g).map(pair=>parseInt(pair,16))));
  const response = await fetch(url,{integrity});
  if (!response.ok) throw Error('Download returned HTTP '+response.status);
  return response.arrayBuffer();
}
async function fieldMember(zip) {
  const header = new DataView(zip);
  if (header.getUint32(0,true)!==0x04034b50 || header.getUint16(8,true)!==8) throw Error('Unexpected field archive');
  const nameLength=header.getUint16(26,true), start=30+nameLength+header.getUint16(28,true);
  if (new TextDecoder().decode(zip.slice(30,30+nameLength))!=='FE-2024.STEP') throw Error('Unexpected archive member');
  const stream = new Blob([zip.slice(start,start+header.getUint32(18,true))]).stream();
  return new Response(stream.pipeThrough(new DecompressionStream('deflate-raw'))).text();
}
function displayGroups(meshes) {
  const groups = {speakerPanel:[],speakerRamp:[],sensors:[]};
  for (const mesh of meshes) {
    const pos=mesh.attributes.position.array, idx=mesh.index.array;
    // XCAF can return unnamed sensor sub-solids. Classify them by their actual
    // placed bounds in the sensor housing envelope measured from the STEP.
    let sensor=true;
    for (let j=0;j<pos.length && sensor;j+=3) {
      const x=8270.875-pos[j], y=4105.275+pos[j+2], z=pos[j+1]-4.7752;
      sensor=x>=-1180 && x<=-1063 && y>=5735 && y<=5788 && z>=1277 && z<=1435;
    }
    const group=sensor?'sensors':/^GE-(24307|24308|24309|24355|24391|24392)/.test(mesh.name)?'speakerRamp':'speakerPanel';
    groups[group].push({pos,idx});
  }
  const buffers={};
  for (const [key,items] of Object.entries(groups)) {
    const out=new Float32Array(items.reduce((total,m)=>total+m.idx.length*6,0));
    let cursor=0;
    for (const {pos,idx} of items) {
      for (let i=0;i<idx.length;i+=3) {
        const a=idx[i]*3,b=idx[i+1]*3,c=idx[i+2]*3;
        const ux=pos[a]-pos[b],uy=pos[b+2]-pos[a+2],uz=pos[b+1]-pos[a+1];
        const vx=pos[a]-pos[c],vy=pos[c+2]-pos[a+2],vz=pos[c+1]-pos[a+1];
        const nx=uy*vz-uz*vy,ny=uz*vx-ux*vz,nz=ux*vy-uy*vx, len=Math.hypot(nx,ny,nz)||1;
        for (const j of [a,b,c]) {
          out[cursor++]=8270.875-pos[j];out[cursor++]=4105.275+pos[j+2];out[cursor++]=pos[j+1]-4.7752;
          out[cursor++]=nx/len;out[cursor++]=ny/len;out[cursor++]=nz/len;
        }
      }
    }
    if (!out.length) throw Error('Missing speaker geometry: '+key);
    buffers[key]=out.buffer;
  }
  return buffers;
}
async function loadFieldSpeaker() {
  const cacheName='frc-2024-speaker-003-v1-'+archiveDigest;
  const cache=self.caches ? await self.caches.open(cacheName).catch(()=>null) : null;
  const groups=['speakerPanel','speakerRamp','sensors'];
  const key=name=>new URL('./cad-preview-cache/'+name,self.location.href).href;
  const hits=cache ? await Promise.all(groups.map(group=>cache.match(key(group)))) : [];
  if(hits.length && hits.every(Boolean)){
    const buffers={};
    for(let i=0;i<groups.length;i++) buffers[groups[i]]=await hits[i].arrayBuffer();
    return buffers;
  }
  const [zip,js,wasm]=await Promise.all([
    verifiedBytes(archiveUrl,archiveDigest),
    verifiedBytes(readerBase+'occt-import-js.js','3fb44ce11d00611f9b3f3c5775d520ebab48930c1f08279b7b1316f05f0d3379'),
    verifiedBytes(readerBase+'occt-import-js.wasm','33391fc9d94ea5c869a6718488bf0a9a464222bac9bdc764dfe1690cef281952')
  ]);
  const readerUrl=URL.createObjectURL(new Blob([js],{type:'text/javascript'}));
  try {importScripts(readerUrl)} finally {URL.revokeObjectURL(readerUrl)}
  postMessage({status:'Preparing the grey speaker from field CAD…'});
  const api=await occtimportjs({wasmBinary:wasm});
  const subset=speakerStep(await fieldMember(zip));
  const model=api.ReadStepFile(new TextEncoder().encode(subset),{linearUnit:'millimeter',linearDeflectionType:'absolute_value',linearDeflection:1,angularDeflection:.27});
  if(!model.success || model.meshes.length!==207) throw Error('Speaker geometry failed its assembly check');
  const buffers=displayGroups(model.meshes);
  if(cache) await Promise.all(groups.map(g=>cache.put(key(g),new Response(buffers[g])))).catch(()=>{});
  return buffers;
}
if(typeof WorkerGlobalScope!=='undefined' && self instanceof WorkerGlobalScope) {
  self.onmessage=async()=>{
    try {const buffers=await loadFieldSpeaker();postMessage({buffers},Object.values(buffers))}
    catch(error){postMessage({error:error.message})}
  };
}


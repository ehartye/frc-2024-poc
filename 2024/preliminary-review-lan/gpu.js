let sceneGPU,paintPending=false,locationMap={},lastProjection;
function initGL(){
 const c=$('canvas');gl=c.getContext('webgl',{antialias:true,alpha:false});
 if(!gl)throw Error('Enable WebGL to view the assembly.');
 const vertex='attribute vec3 point;attribute vec3 normal;uniform mat4 pose;uniform mat4 camera;uniform vec3 tint;varying vec3 shade;void main(){gl_Position=camera*pose*vec4(point,1.0);vec3 n=mat3(pose)*normal;float light=.83+.28*dot(n,vec3(.32,-.42,.84));shade=tint*light+vec3(.035);}';
 const fragment='precision mediump float;varying vec3 shade;void main(){gl_FragColor=vec4(shade,1.0);}';
 program=gl.createProgram();for(const [type,src] of [[gl.VERTEX_SHADER,vertex],[gl.FRAGMENT_SHADER,fragment]]){
  const shader=gl.createShader(type);gl.shaderSource(shader,src);gl.compileShader(shader);if(!gl.getShaderParameter(shader,gl.COMPILE_STATUS))throw Error(gl.getShaderInfoLog(shader));gl.attachShader(program,shader)}
 gl.linkProgram(program);if(!gl.getProgramParameter(program,gl.LINK_STATUS))throw Error(gl.getProgramInfoLog(program));gl.useProgram(program);
 for(const key of ['camera','pose','tint'])locationMap[key]=gl.getUniformLocation(program,key);
 position=gl.getAttribLocation(program,'point');color=gl.getAttribLocation(program,'normal');gl.enableVertexAttribArray(position);gl.enableVertexAttribArray(color);gl.enable(gl.DEPTH_TEST);
 function upload(values,mode){const buffer=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,buffer);gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(values),gl.STATIC_DRAW);return {buffer,count:values.length/6,mode}}
 function triangles(s){const out=[];for(const f of s.f){const [a,b,c]=f.map(i=>s.v[i]);const u=b.map((x,i)=>x-a[i]),v=c.map((x,i)=>x-a[i]);const n=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]],l=Math.hypot(...n)||1;for(const p of [a,b,c])out.push(...p,n[0]/l,n[1]/l,n[2]/l)}return upload(out,gl.TRIANGLES)}
 const mesh=new Map(entries.map(([n,b])=>[n,triangles(b)]));const lineData=[];const add=(a,b)=>lineData.push(...a,0,0,1,...b,0,0,1);
 const size=[16540,8210];for(let x=0;x<=size[0];x+=250)add([x,0,0],[x,size[1],0]);for(let y=0;y<=size[1];y+=250)add([0,y,0],[size[0],y,0]);
 const grid=upload(lineData,gl.LINES);lineData.length=0;data.corners.forEach((p,i)=>add(p,data.corners[(i+1)%4]));
 sceneGPU={mesh,grid,aperture:upload(lineData,gl.LINES),ground:triangles({v:[[0,0,-.1],[size[0],0,-.1],[size[0],size[1],-.1],[0,size[1],-.1]],f:[[0,1,2],[0,2,3]]})};
}
function modelMatrix(m){return [m[0],m[4],m[8],0,m[1],m[5],m[9],0,m[2],m[6],m[10],0,m[3],m[7],m[11],1]}
function draw(){if(!cam||paintPending)return;paintPending=true;requestAnimationFrame(render)}
function render(){
 paintPending=false;if(!sceneGPU)return;
 const cs=current(),t=+$('time').value,canvas=$('canvas'),width=canvas.clientWidth,height=canvas.clientHeight,dpr=Math.min(devicePixelRatio||1,2);
 const pw=Math.round(width*dpr),ph=Math.round(height*dpr);
 if(canvas.width!==pw||canvas.height!==ph){canvas.width=pw;canvas.height=ph;gl.viewport(0,0,pw,ph)}
 gl.clearColor(.058,.114,.207,1);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);
 const ca=Math.cos(cam.az),sa=Math.sin(cam.az),ce=Math.cos(cam.el),se=Math.sin(cam.el),sc=cam.zoom*Math.min(width,height)/480;
 const u=[-sa,ca,0].map(v=>v*2*sc/width),v=[-se*ca,-se*sa,ce].map(v=>v*2*sc/height),z=[ce*ca,ce*sa,se].map(v=>-v/45000);
 const dot=(a,b)=>a.reduce((s,x,i)=>s+x*b[i],0);
 const off=[2*cam.x/width-dot(u,cam.center),-2*cam.y/height-dot(v,cam.center),-dot(z,cam.center)];
 lastProjection=p=>[dot(u,p)+off[0],dot(v,p)+off[1],dot(z,p)+off[2]];
 gl.uniformMatrix4fv(locationMap.camera,false,[u[0],v[0],z[0],0,u[1],v[1],z[1],0,u[2],v[2],z[2],0,...off,1]);
 function paint(item,mat,col){gl.bindBuffer(gl.ARRAY_BUFFER,item.buffer);gl.vertexAttribPointer(position,3,gl.FLOAT,false,24,0);gl.vertexAttribPointer(color,3,gl.FLOAT,false,24,12);gl.uniformMatrix4fv(locationMap.pose,false,modelMatrix(mat));gl.uniform3fv(locationMap.tint,col);gl.drawArrays(item.mode,0,item.count)}
 const xy=cs.positions[t],field=[1,0,0,-xy[0],0,1,0,-xy[1],0,0,1,0];if($('floor').checked){if(se>0)paint(sceneGPU.ground,field,[.1,.17,.25]);paint(sceneGPU.grid,field,[.16,.25,.34])}paint(sceneGPU.aperture,field,[.3,.78,.95]);
 const labels=[];for(const [name,b] of entries){if(cs.removed.includes(name)||(isolated&&isolated!==name))continue;
  const m=cs.frames[name]?.[t]||cs.frames[name]?.[0]||identity;
  let col=b.tint;if(marked.includes(name))col=[1,.34,.14];else if(!selNames.has(name))col=b.dim;if(name===selected)col=[1,.79,.27];
  paint(sceneGPU.mesh.get(name),m,col);
  if($('labels').checked||marked.includes(name)||name===selected){const p=lastProjection(flat(m,b.anchor));if(Math.abs(p[0])<=1&&Math.abs(p[1])<=1){const tag=text('span',name);tag.className='tag';tag.style.left=(50+50*p[0])+'%';tag.style.top=(50-50*p[1])+'%';labels.push(tag)}}
 }
 const layer=$('names');if(labels.length||layer.childElementCount)layer.replaceChildren(...labels);$('fraction').textContent=Math.round(t/24*100)+'%';
}
function candidatesForPick(){
 const cs=current(),t=+$('time').value;
 return entries.filter(([n])=>!cs.removed.includes(n)&&(!isolated||n===isolated)).map(([name,b])=>{
  const m=cs.frames[name]?.[t]||cs.frames[name]?.[0]||identity;
  return {name,points:b.v.map(p=>lastProjection(flat(m,p))),faces:b.f}
 })
}


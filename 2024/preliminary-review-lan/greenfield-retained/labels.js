// Place part names in one margin. The two vertical rails form an ordered fan:
// both ends have the same Y order, then leaders run horizontally to material.
let labelPage=0, followLabel=null;
const calloutNodes=[];
function labelRail(width){return Math.min(212,Math.max(92,width*.235))}
function drawLeaders(labels,width,height,rail){
 const layer=$('names'),nav=$('labelPages'),all=$('labels').checked;
 if(!labels.length){layer.hidden=true;nav.hidden=true;return}
 layer.hidden=false;
 let svg=layer.firstElementChild;
 const ns='http://www.w3.org/2000/svg';
 function element(kind){return document.createElementNS(ns,kind)}
 if(!svg){svg=element('svg');svg.classList.add('callout-diagram');svg.setAttribute('aria-label','Component callouts');layer.append(svg)}
 svg.setAttribute('viewBox','0 0 '+width+' '+height);
 const rowHeight=24,capacity=Math.max(1,Math.floor((height-36)/rowHeight)),pages=[];
 for(const point of labels){
  let page=pages.find(items=>items.length<capacity&&items.every(other=>Math.abs(other.y-point.y)>=7));
  if(!page){page=[];pages.push(page)}page.push(point);
 }
 if(followLabel){const i=pages.findIndex(page=>page.some(p=>p.name===followLabel));if(i>=0)labelPage=i;followLabel=null}
 labelPage=Math.min(labelPage,pages.length-1);
 const page=pages[labelPage].sort((a,b)=>a.y-b.y);
 nav.hidden=!all||pages.length<2;$('labelPageCount').textContent=(labelPage+1)+' / '+pages.length;
 $('labelPrev').disabled=labelPage===0;$('labelNext').disabled=labelPage===pages.length-1;
 // Clamp each label between the previous row and the room needed by later rows.
 let previous=-rowHeight;
 for(let i=0;i<page.length;i++){
  const p=page[i],row=Math.max(17,previous+rowHeight,Math.min(p.y,height-17-(page.length-i-1)*rowHeight));previous=row;
  let node=calloutNodes[i];
  if(!node){
   const g=element('g'),line=element('polyline'),dot=element('circle'),label=element('text'),title=element('title');
   dot.setAttribute('r','2.7');label.classList.add('tag');label.setAttribute('text-anchor','end');label.setAttribute('dominant-baseline','middle');
   g.append(line,dot,label,title);svg.append(g);node={g,line,dot,label,title};calloutNodes.push(node);
   label.addEventListener('click',()=>inspect(g.dataset.part));
  }
  node.g.style.display='';node.g.dataset.part=p.name;
  node.g.classList.toggle('chosen',p.name===selected||marked.includes(p.name));
  const start=rail-12,turn=rail+14;
  node.line.setAttribute('points',[[start,row],[turn,p.y],[p.x,p.y]].map(q=>q.join(',')).join(' '));
  node.dot.setAttribute('cx',p.x);node.dot.setAttribute('cy',p.y);
  node.label.setAttribute('x',start-5);node.label.setAttribute('y',row);
  node.label.textContent=p.name;node.title.textContent=p.name;
  const max=rail-24;
  if(node.label.getComputedTextLength()>max){node.label.setAttribute('textLength',max);node.label.setAttribute('lengthAdjust','spacingAndGlyphs')}
  else node.label.removeAttribute('textLength');
 }
 for(const node of calloutNodes.slice(page.length))node.g.style.display='none';
}


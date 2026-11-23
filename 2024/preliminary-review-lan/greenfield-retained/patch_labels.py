from pathlib import Path
p=Path(__file__).resolve().parent;www=p.parent/'www'
def put(f,s):
    stage=f.with_suffix('.pending');stage.write_text(s,encoding='utf8');stage.replace(f)
js=(www/'review.js').read_text(encoding='utf8')
if 'function drawLeaders(' not in js:
    js=js.replace('function render(){',(p/'labels.js').read_text(encoding='utf8')+'\nfunction render(){')
    js=js.replace(' const ca=Math.cos(cam.az)'," const rail=($('labels').checked||selected||marked.length)?labelRail(width):0;\n const ca=Math.cos(cam.az)").replace('Math.min(width,height)/480','Math.min(width-rail,height)/480').replace('2*cam.x/width-dot','(2*cam.x+rail)/width-dot')
    start=js.index("  if($('labels').checked||marked.includes(name)||name===selected)")
    end=js.index("\n }",start)
    js=js[:start]+"""  if(rail&&($('labels').checked||marked.includes(name)||name===selected)){
   const point=lastProjection(flat(m,b.leaderAnchor)),x=(point[0]+1)*width/2,y=(1-point[1])*height/2;
   if(x>rail+17&&x<width-4&&y>4&&y<height-4)labels.push({name,x,y});
  }"""+js[end:]
    a=js.index(" const layer=$('names');if(labels.length");b=js.index("$('fraction')",a)
    js=js[:a]+' drawLeaders(labels,width,height,rail);'+js[b:]
    js=js.replace('function inspect(n){selected=n;','function inspect(n){followLabel=n;selected=n;')
    js=js.replace('s+p[i],0)/b.v.length);}',"s+p[i],0)/b.v.length);let best=Infinity;for(const p of b.v){const d=p.reduce((sum,x,i)=>sum+(x-b.anchor[i])**2,0);if(d<best){best=d;b.leaderAnchor=p}}}")
    js=js.replace(" for(const id of ['labels','floor'])", " $('labelPrev').onclick=()=>{labelPage=Math.max(0,labelPage-1);draw()};$('labelNext').onclick=()=>{labelPage+=1;draw()};\n for(const id of ['labels','floor'])")
    put(www/'review.js',js)
html=(www/'index.html').read_text(encoding='utf8')
if 'id="labelPages"' not in html:put(www/'index.html',html.replace(' Labels</label>',' Labels</label><span id="labelPages" hidden><button id="labelPrev" aria-label="Previous callouts">‹</button><output id="labelPageCount"></output><button id="labelNext" aria-label="Next callouts">›</button></span>'))
css=(www/'review.css').read_text(encoding='utf8')
if '.callout-diagram' not in css:put(www/'review.css',css+"""
.callout-diagram{width:100%;height:100%;display:block;overflow:hidden}.callout-diagram polyline{stroke:#c4d8ee;stroke-opacity:.82;fill:none;stroke-width:1;vector-effect:non-scaling-stroke}.callout-diagram circle{fill:#ffcf5b;stroke:#142842;stroke-width:1}.callout-diagram .tag{fill:#edf5ff;font:600 12px 'Segoe UI',sans-serif;paint-order:stroke;stroke:#112641;stroke-width:3;stroke-linejoin:round;pointer-events:auto;cursor:pointer}.callout-diagram .chosen polyline{stroke:#ffcf5b;stroke-width:1.7}.callout-diagram .chosen .tag{fill:#ffcf5b}#labelPages:not([hidden]){display:inline-flex;align-items:center;gap:5px}#labelPages button{padding:0 7px;font-size:18px;min-width:26px;line-height:22px}#labelPageCount{font:12px var(--mono)}#names[hidden]{display:none}
""")


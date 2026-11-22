/* Selected-duty, battery-side load ledger and role-based commissioning checks. */
function signed(v,unit,digits){return (v>=0?'+':'−')+Math.abs(v).toFixed(digits)+' '+unit}
function resourcePanel(){
 const e=duty(),limit=56.69904625,worst=data.inspection[1],voc=e.openCircuitV??(e.loadedBusV[0]+e.resistance*e.batterySideA[1]),r=e.resistance,minimum=e.requiredMinV,available=(voc-minimum)/r,imax=e.batterySideA[1];
 const pAt=i=>i*(voc-r*i),peak=Math.max(pAt(e.batterySideA[0]),pAt(imax),e.batterySideA[0]<=voc/(2*r)&&imax>=voc/(2*r)?pAt(voc/(2*r)):0);
 const total=$('resources');total.replaceChildren();
 function meter(label,value,message,used,capacity,negative){const block=document.createElement('section');block.className='budget-meter';block.append(text('span',label),text('b',value));const bar=document.createElement('progress');bar.max=Math.max(used,capacity)*1.04;bar.value=used;bar.setAttribute('aria-label',label);block.classList.toggle('over',negative);block.append(bar,text('small',message));total.append(block)}
 meter('Inspection weight',data.inspection[0].toFixed(3)+'–'+worst.toFixed(3)+' kg','Limit '+limit.toFixed(3)+' kg · headroom '+signed(limit-worst,'kg',3)+' high / '+signed(limit-data.inspection[0],'kg',3)+' low',worst,limit,worst>limit);
 meter('Battery voltage under load',e.loadedBusV[0].toFixed(3)+' V minimum','Requirement '+minimum.toFixed(2)+' V · headroom '+signed(e.loadedBusV[0]-minimum,'V',3),voc-e.loadedBusV[0],voc-minimum,e.loadedBusV[0]<minimum);
 meter('Battery-side demand',imax.toFixed(1)+' A · '+(peak/1000).toFixed(2)+' kW','Pack budget at '+minimum.toFixed(1)+' V: '+available.toFixed(1)+' A / '+(minimum*available/1000).toFixed(2)+' kW',imax,available,imax>available);
 const mount=$('loads');mount.replaceChildren(text('p',e.case+' · '+voc.toFixed(1)+' V open circuit · '+(1000*r).toFixed(1)+' mΩ battery model. Select a load to highlight its components.'));
 for(const line of e.lines){
  const d=document.createElement('details');const s=document.createElement('summary');s.textContent=line.name+'　 '+range(line.amps,1)+' A';d.append(s);
  const minW=line.amps[0]*e.loadedBusV[0],maxW=line.amps[1]*e.loadedBusV[1];d.append(text('p','Battery input envelope '+Math.floor(minW)+'–'+Math.ceil(maxW)+' W · pack sag contribution '+range(line.amps.map(i=>r*i),3)+' V. Power bounds combine voltage and current extrema.'));
  const group=text('button','Locate this load');group.onclick=()=>{isolated=null;selected=null;marked=line.members.slice();draw();$('stage').scrollIntoView({block:'nearest'});};d.append(group);
  const members=document.createElement('div');members.className='member-buttons';line.members.forEach(n=>{if(data.bodies[n]){const b=text('button',n);b.onclick=()=>{isolated=null;inspect(n);$('stage').scrollIntoView({block:'nearest'})};members.append(b)}});d.append(members);d.append(text('p',line.members.length>1?'Measured as a shared load. Individual current split is unavailable.':'Assigned to this component in the duty model.'));mount.append(d)
 }
 const v=$('required');v.replaceChildren();const names=new Set(entries.filter(([n,b])=>!($('envelope').checked&&b.added)).map(([n])=>n));const motors=Array.from(names).filter(n=>/^Swerve (drive|steer) motor/.test(n));
 const expected=[['Control computer',['roboRIO flat']],['Power distribution',['PDH']],['Battery',['Battery and terminal case']],['Main breaker',['main breaker']],['Radio',['radio']],['Four drive wheels',['Front left tire','Front right tire','Rear left tire','Rear right tire']],['Four drive and four steer motors', ['Front left','Front right','Rear left','Rear right'].flatMap(c=>['Swerve drive motor · '+c,'Swerve steer motor · '+c])],['Feeder drive',['Feeder motor']]];
 let missing=0;for(const [role,list] of expected){const absent=list.filter(n=>!names.has(n));missing+=absent.length;const row=document.createElement('div');row.className='role-row '+(absent.length?'warn':'');row.append(text('strong',absent.length?'Missing':'Present'),text('span',role+'　 '+(list.length-absent.length)+'/'+list.length));if(absent.length)row.append(text('small',absent.join(', ')));v.append(row)}
 const warns=$('warningText');warns.textContent=(missing?missing+' missing required representations. ':'Required component counts present. ')+(worst>limit?'High estimate exceeds inspection weight. ':'')+(e.loadedBusV[0]<minimum?'Selected duty falls below voltage requirement. ':'')+'Presence checks use registered component identities. Electrical connection, integration and hardware readiness require separate evidence.';
}


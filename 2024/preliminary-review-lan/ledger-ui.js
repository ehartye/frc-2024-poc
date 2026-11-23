function weightLedger(){
 const root=$('weightBody');root.replaceChildren();
 const limit=56.69904625,low=data.inspection[0],high=data.inspection[1];
 const heading=text('p','The purchasing range crosses the inspection limit. The low case clears by '+(limit-low).toFixed(3)+' kg. The high case needs '+(high-limit).toFixed(3)+' kg removed.');
 root.append(heading);
 const ruler=document.createElement('div');ruler.className='weight-ruler';
 const start=limit-1,end=limit+1;const band=document.createElement('i');band.style.left=((low-start)/(end-start)*100)+'%';band.style.width=((high-low)/(end-start)*100)+'%';ruler.append(band,text('b','Limit '+limit.toFixed(3)+' kg'));root.append(ruler);
 const [tab,body]=table(['Inspection allocation','Low, kg','High, kg']);for(const row of data.mass.filter(r=>r.kg[1]>0)){
 const tr=body.insertRow();tr.insertCell().textContent=row.id;for(const v of row.kg)tr.insertCell().textContent=v.toFixed(3)}
 const total=body.insertRow();total.className='grand-total';total.insertCell().textContent='Inspection total';for(const v of data.inspection)total.insertCell().textContent=v.toFixed(3);root.append(tab);
 const details=document.createElement('details');details.append(text('summary','Running configuration · '+range(data.running,2)+' kg'));details.append(text('p','Add the battery, one game piece and the finished bumper scenario to the inspection estimate.'));
 const [ta,bd]=table(['Additional article','Low, kg','High, kg']);for(const r of data.details.filter(r=>['battery','one NOTE','finished bumper scenario'].includes(r.name))){const tr=bd.insertRow();tr.insertCell().textContent=r.name;for(const v of r.kg)tr.insertCell().textContent=v.toFixed(3)}details.append(ta);root.append(details)
}
function drawPowerBars(){
 const e=duty(),holder=$('powerBars');holder.replaceChildren();const max=e.batterySideA[1];
 for(const [i,line] of e.lines.entries()){
 const b=document.createElement('button');b.className='load-bar';b.setAttribute('aria-label','Inspect '+line.name);
 const caption=text('span',line.name),val=text('strong',line.amps[1].toFixed(1)+' A'),track=document.createElement('i'),fill=document.createElement('i');fill.style.width=Math.max(.5,100*line.amps[1]/Math.max(1,max))+'%';fill.style.background=['#2f66b0','#328a8a','#dfac29'][i%3];track.append(fill);b.append(caption,val,track);
 b.onclick=()=>{const target=$('loads').children[i+1];target.open=true;target.scrollIntoView({block:'nearest'});isolated=null;marked=line.members.slice();selected=null;draw()};holder.append(b)
 }
}


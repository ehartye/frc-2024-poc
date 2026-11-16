import {createDesign,assertDesign,canonicalStringify} from './design-artifact.mjs';
let current=createDesign();
export function getDesign(){return JSON.parse(canonicalStringify(current))}
export function setDesign(value){assertDesign(value);current=JSON.parse(canonicalStringify(value));return getDesign()}
// Axis-aligned difference: non-overlapping boxes, retaining explicit support volume.
export function subtractBox(box,cut){const lo=box.center.map((v,i)=>v-box.size[i]/2),hi=box.center.map((v,i)=>v+box.size[i]/2),cl=cut.center.map((v,i)=>Math.max(lo[i],v-cut.size[i]/2)),ch=cut.center.map((v,i)=>Math.min(hi[i],v+cut.size[i]/2));if(cl.some((x,i)=>x>=ch[i]))return [box];const spans=[[lo[0],cl[0],lo[1],hi[1],lo[2],hi[2]],[ch[0],hi[0],lo[1],hi[1],lo[2],hi[2]],[cl[0],ch[0],lo[1],cl[1],lo[2],hi[2]],[cl[0],ch[0],ch[1],hi[1],lo[2],hi[2]],[cl[0],ch[0],cl[1],ch[1],lo[2],cl[2]],[cl[0],ch[0],cl[1],ch[1],ch[2],hi[2]]];return spans.filter(s=>s[1]>s[0]&&s[3]>s[2]&&s[5]>s[4]).map((s,i)=>({...box,id:box.id+'-fragment-'+i,center:[(s[0]+s[1])/2,(s[2]+s[3])/2,(s[4]+s[5])/2],size:[s[1]-s[0],s[3]-s[2],s[5]-s[4]]}))}

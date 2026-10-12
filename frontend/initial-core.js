(function(root){
 'use strict';
 function overlap(a,b){const volume=x=>x.max.reduce((v,n,i)=>v*Math.max(.001,n-x.min[i]),1),intersection=a.max.reduce((v,n,i)=>v*Math.max(0,Math.min(n,b.max[i])-Math.max(a.min[i],b.min[i])),1);return intersection/Math.min(volume(a),volume(b));}
 function plan(existing,structures,objects,B,C,uuid,batch){
  const added=[],skipped={invalid:0,overlap:0,duplicate:0,limit:0};
  for(const item of [...structures,...objects]){
   if(!B.validBox(item)||C.geometryWarnings(item).length){skipped.invalid++;continue;}
   const all=[...existing,...added];
   if(all.some(b=>b.id===item.id||b.provenance?.proposal_id===item.id)){skipped.duplicate++;continue;}
   // Conservative protection, not an instance-identity verdict. Never replace old geometry.
   if(all.some(b=>{const fraction=overlap(b,item);return fraction>.6||(fraction>0&&C.supportOverlap(b,item)>.15);})){skipped.overlap++;continue;}
   if(all.length>=5000){skipped.limit++;continue;}
   const b=structuredClone(item);b.id=uuid();b.sequence=B.nextSequence(all,b.category);b.status='proposed';b.provenance={...b.provenance,proposal_id:item.id,initial_batch:batch};added.push(b);
  }
  return {added,skipped};
 }
 root.InitialCore={plan};if(typeof module!=='undefined')module.exports=root.InitialCore;
})(globalThis);

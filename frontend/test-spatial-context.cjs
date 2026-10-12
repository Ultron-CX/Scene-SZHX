const assert=require('node:assert/strict'),C=require('./project-core.js'),crypto=require('node:crypto');
const a='zone-'+crypto.randomUUID(),b='zone-'+crypto.randomUUID(),oid='obj-'+crypto.randomUUID();
const z=(id,parent)=>({zone_id:id,name:'Synthetic',kind:'room',parent_id:parent,source:'manual',verification:'unverified',notes:'not bank truth'});
const s={schema:'scenesem.spatial-context.v1',zones:[z(a,null),z(b,a)],memberships:[{object_id:oid,zone_id:b,source:'automatic_candidate',verification:'unverified',notes:''}]};
assert.equal(C.validateSpatial(s),s);assert.equal(C.validateSpatial(null),null);assert.equal(C.spatialSummary(s,[]).missing_objects[0],oid);assert.equal(C.spatialSummary(s,[{object_id:oid}]).unassigned,0);
for(const mutate of [v=>v.zones[0].parent_id=b,v=>v.zones[0].parent_id='missing',v=>v.zones.push(v.zones[0]),v=>v.memberships[0].zone_id='missing',v=>v.memberships.push(v.memberships[0]),v=>v.zones[0].verification='verified',v=>v.memberships[0].source='bank_truth',v=>v.extra=true,v=>v.memberships[0].object_id='wrong']){const next=structuredClone(s);mutate(next);assert.throws(()=>C.validateSpatial(next));}
const reordered=Object.fromEntries(Object.entries(s).reverse());assert(C.sameSpatial(s,reordered));assert(!C.sameSpatial(s,null));
const unknown=structuredClone(s);unknown.memberships[0].zone_id=null;assert.equal(C.spatialSummary(unknown,[{object_id:oid}]).unassigned,1);
console.log('PASS spatial contract: hierarchy cycles/references, provenance, unknown membership, dangling objects, canonical equality.');

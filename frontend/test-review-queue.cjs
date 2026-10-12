const assert=require('node:assert/strict'),P=require('./project-core.js');
const regions=[{id:'r1',category:'chair',status:'reviewed'}];
const o={object_id:'obj-1',alias:'',region_ids:['r1'],evidence_ids:[],reviews:[],task_qualification:false};
const state=x=>P.reviewState(x,regions,P.basis(x,regions));
assert.equal(state(o).status,'unreviewed');assert.equal(state(o).evidence_count,0);
const reviewed=structuredClone(o);reviewed.reviews=[{checks:Object.fromEntries(P.dimensions.map(d=>[d,'pass']))}];
assert.equal(state(reviewed).status,'recorded');assert.equal(state(reviewed).needs_work,false);assert.equal(state(reviewed).task_qualification,false);
for(const [value,status]of [['issue','issue'],['unknown','unknown'],['pending','unreviewed'],['invalid','unreviewed']]){const item=structuredClone(reviewed);item.reviews[0].checks.background=value;assert.equal(state(item).status,status);}
assert.equal(P.reviewState(reviewed,regions).status,'stale','restored browser draft without a verified in-session basis stays conservative');
assert.equal(P.reviewState(reviewed,[{...regions[0],category:'screen'}],P.basis(reviewed,regions)).status,'stale');
assert.equal(state({...reviewed,review_stale:true}).status,'stale');
assert.equal(state({...o,import_requires_review:true}).status,'stale');
const missing={...reviewed,region_ids:['r1','deleted']};assert.equal(state(missing).status,'missing');assert.equal(state(missing).checks.identity,'pending');
const corrected={...reviewed,mask_corrections:[{evidence_id:'e',component_index:0,category:'screen'}]};assert.equal(P.reviewState(corrected,regions,P.basis(reviewed,regions)).status,'stale');
const withEvidence={...o,evidence_ids:['a'.repeat(64)]},original=JSON.stringify([withEvidence,regions]);assert.equal(state(withEvidence).evidence_count,1);assert.equal(state(withEvidence).status,'unreviewed');assert.equal(JSON.stringify([withEvidence,regions]),original);
console.log('PASS review queue: pending/issue/unknown/stale/import/missing references, basis changes, no evidence-to-quality or task promotion; immutable inputs. Synthetic only.');

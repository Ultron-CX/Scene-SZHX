const assert=require('node:assert/strict'),C=require('./project-core.js'),{randomUUID}=require('node:crypto');
const a={attribute_id:'attr-'+randomUUID(),subject_kind:'object',subject_id:'obj-'+randomUUID(),name:'服务类型',value:'Synthetic',basis:'simulation',source:'Test plan v1',recorder:'',observed_on:null,valid_from:'2026-09-01',valid_until:'2026-09-30',verification:'unverified'},b={schema:'scenesem.business-context.v1',attributes:[a]};
assert.equal(C.validateBusiness(b),b);assert.equal(C.validateBusiness(null),null);
for(const [k,v] of [['basis','confirmed'],['source',''],['valid_until','2026-08-31'],['valid_from','2026-02-30'],['observed_on','0000-01-01'],['subject_id','wrong'],['verification','verified'],['extra',true]]){const t=structuredClone(b);t.attributes[0][k]=v;assert.throws(()=>C.validateBusiness(t));}
const field=structuredClone(b);field.attributes[0].basis='field_report';assert.throws(()=>C.validateBusiness(field));Object.assign(field.attributes[0],{recorder:'Synthetic witness',observed_on:'2026-09-14'});C.validateBusiness(field);
const unknown=structuredClone(b);unknown.attributes[0].basis='unknown';assert.throws(()=>C.validateBusiness(unknown));unknown.attributes[0].value='';C.validateBusiness(unknown);
const duplicate=structuredClone(b);duplicate.attributes.push(duplicate.attributes[0]);assert.throws(()=>C.validateBusiness(duplicate));
console.log('PASS business claims: strict schema, source/date constraints, unknown/simulation/field report separation. No truth promotion.');

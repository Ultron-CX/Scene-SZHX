'use strict';
const assert=require('node:assert/strict'),{plan}=require('./initial-core.js');
// Synthetic dependency contracts only: these stubs do not validate real geometry.
const box=(id,x=0,extra={})=>({id,category:'chair',min:[x,0,0],max:[x+10,1,1],...extra});
function run(existing,structures,objects=[],support=0){
 let ids=0,supportCalls=0;
 const B={validBox:b=>!b.invalid,nextSequence:(all,c)=>1+Math.max(0,...all.filter(b=>b.category===c).map(b=>b.sequence||0))};
 const C={geometryWarnings:b=>b.warning?['synthetic warning']:[],supportOverlap:()=>{supportCalls++;return support;}};
 return {...plan(existing,structures,objects,B,C,()=>`new-${++ids}`,'batch-test'),supportCalls,ids};
}
const counts=(r,expected)=>assert.deepEqual(r.skipped,{invalid:0,overlap:0,duplicate:0,limit:0,...expected});
let cases=0;function test(name,fn){fn();cases++;console.log(`PASS ${name}`);}
test('clone, metadata, category sequence and structural-first ordering',()=>{
 const old=[box('old',100,{sequence:7})];
 const structures=[box('structure',0,{category:'wall',provenance:{nested:{source:'synthetic'}},support:[[1,2,3]]})];
 const objects=[box('object',20)];const before=structuredClone({old,structures,objects});
 const r=run(old,structures,objects);counts(r,{});
 assert.deepEqual(r.added.map(b=>[b.id,b.category,b.sequence,b.status,b.provenance.proposal_id]),[['new-1','wall',1,'proposed','structure'],['new-2','chair',8,'proposed','object']]);
 assert.equal(r.added[0].provenance.initial_batch,'batch-test');
 r.added[0].min[0]=-90;r.added[0].support[0][0]=99;r.added[0].provenance.nested.source='edited';
 assert.deepEqual({old,structures,objects},before);
});
test('original and proposal duplicates, including new rows in current batch',()=>{
 const r=run([box('old',100,{provenance:{proposal_id:'prior'}})],
 [box('old'),box('prior'),box('fresh'),box('fresh',20),box('new-1',30),box('second',40)]);
 counts(r,{duplicate:4});assert.deepEqual(r.added.map(b=>b.sequence),[1,2]);assert.equal(r.ids,2);
});
test('AABB strict 0.6 threshold and disjoint/touching support guard',()=>{
 for(const [x,rejected,calls] of [[4,false,1],[3.999,true,0],[4.001,false,1],[10,false,0],[11,false,0]]){
  const r=run([box('old')],[box('next',x)]);counts(r,{overlap:Number(rejected)});assert.equal(r.supportCalls,calls);
 }
 const r=run([box('old')],[box('far',11)],[],1);counts(r,{});assert.equal(r.supportCalls,0);
});
test('support strict 0.15 threshold with positive AABB intersection',()=>{
 for(const [support,rejected] of [[.14999,false],[.15,false],[.15001,true]]){
  const r=run([box('old')],[box('next',5)],[],support);counts(r,{overlap:Number(rejected)});assert.equal(r.supportCalls,1);
 }
});
test('4999 accepts one row then enforces capacity',()=>{
 const existing=Array.from({length:4999},(_,i)=>box(`old-${i}`,100+i*20));
 const r=run(existing,[box('first',-40),box('second',-20)]);counts(r,{limit:1});assert.equal(r.added.length,1);assert.equal(r.ids,1);assert.equal(existing.length,4999);
});
test('reason priority invalid before duplicate before overlap before capacity',()=>{
 const existing=Array.from({length:5000},(_,i)=>box(`old-${i}`,100+i*20));
 const input=[box('old-0',100,{invalid:true}),box('old-0',100,{warning:true}),box('old-0',100),box('overlap',100),box('capacity',-20)];
 const r=run(existing,input);counts(r,{invalid:2,duplicate:1,overlap:1,limit:1});assert.equal(r.ids,0);
 assert.equal(Object.values(r.skipped).reduce((a,b)=>a+b,0)+r.added.length,input.length);
});
console.log(`${cases} InitialCore boundary cases passed; synthetic logic only, no browser/GPU/semantic accuracy validation.`);

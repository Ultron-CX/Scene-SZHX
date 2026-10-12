// Synthetic projection/visibility regression; not semantic accuracy or GPU acceptance.
const assert=require('node:assert/strict'),P=require('./project-core.js');
const near=.1,far=10,A=(far+near)/(near-far),k=2*far*near/(near-far);
const frame={width:64,height:64,camera:{near,far,projection:[1,0,0,0,0,1,0,0,0,0,A,-1,0,0,k,0],inverse:[1,0,0,0,0,1,0,0,0,0,0,1/k,0,0,-1,A/k]}};
const depth=new DataView(new ArrayBuffer(64*64*4));
for(let i=0;i<4096;i++)depth.setFloat32(i*4,(2-near)/(far-near),true);
const at=(x,y)=>P.unproject(frame,x,y,(2-near)/(far-near));
const region=(id,p)=>({id,geometry:{type:'surface-voxels',origin:p.map(n=>n-.005),cell:.01,cells:[[0,0,0]]}});
const regions=[region('inside',at(20,20)),region('exclusive-right',at(22,20)),region('offscreen',[100,0,-2]),region('behind',[0,0,2]),region('invalid',at(24,20)),region('occluded',at(26,20)),{id:'box',geometry:{type:'box'}}];
depth.setFloat32((20*64+24)*4,NaN,true);depth.setFloat32((20*64+26)*4,(1-near)/(far-near),true);
const result={components:[{category:'chair',runs:[[20,20,22]]},{category:'computer',runs:[[20,20,23]]},{category:'screen',runs:[]}]};
const before=JSON.stringify({frame,regions,result}),depthBefore=new Uint8Array(depth.buffer).slice();
const comparison=P.compareEvidence(frame,depth,regions,result);
assert.deepEqual(comparison.stats,{total:6,checked:6,offscreen:2,invalid_depth:1,depth_disagreement:1,visible:2,sampled:false,unsupported_regions:1});
assert.deepEqual(comparison.masks[0].inside,[0]);assert.deepEqual(comparison.masks[0].outside,[1]);
assert.equal(comparison.masks[0].contains(22,20),false,'runs use exclusive right edge');
assert.deepEqual(comparison.masks[0].bounds,[20,20,22,21]);assert.equal(comparison.masks[0].pixels,2);
assert.deepEqual(comparison.masks[1].inside,[0,1],'overlapping masks stay independent, not identity assignments');
assert.equal(comparison.masks[2].bounds,null);assert.deepEqual(comparison.masks[2].outside,[0,1]);
assert.deepEqual(comparison.points.map(p=>p.region_id),['inside','exclusive-right']);
for(const invalid of [-.01,1,Infinity,NaN]){const d=new DataView(depth.buffer.slice(0));d.setFloat32((20*64+20)*4,invalid,true);assert.equal(P.compareEvidence(frame,d,[regions[0]],result).stats.invalid_depth,1);}
const dense=structuredClone(regions[0]);dense.geometry.cells=Array.from({length:40001},()=>[0,0,0]);
const sampled=P.compareEvidence(frame,depth,[dense],null);assert.equal(sampled.stats.total,40001);assert(sampled.stats.sampled);assert(sampled.stats.checked<=20000);assert.equal(sampled.stats.checked,sampled.stats.visible);assert.deepEqual(sampled.masks,[]);
assert.deepEqual(P.compareEvidence(frame,depth,[],null).points,[]);
assert.equal(JSON.stringify({frame,regions,result}),before);assert.deepEqual(new Uint8Array(depth.buffer),depthBefore);
console.log('PASS evidence comparison: mask edges/overlaps, missing depth, occlusion, offscreen/behind, empty masks, bounded sampling and immutable inputs. Synthetic only.');

// Frozen v8 evidence only. No inference, browser access or production writes.
const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict'),crypto=require('node:crypto');
const P=require('../frontend/project-core.js'),root=path.resolve(__dirname,'..'),store=path.join(root,'data/projects/k2');
const hash=b=>crypto.createHash('sha256').update(b).digest('hex');
const file=path.join(store,'revisions/00000008.json'),original=fs.readFileSync(file),project=JSON.parse(original);
const blob=id=>{const bytes=fs.readFileSync(path.join(store,'blobs',id));assert.equal(hash(bytes),id);return bytes;};
const scene=JSON.parse(fs.readFileSync(path.join(root,'artifacts/k2-annotation-scene-v2/scene.json')));
assert.equal(project.annotations.scene_id,scene.scene_id);assert.deepEqual(project.annotations.source_hashes,scene.source_hashes);assert.deepEqual(project.annotations.T_lcc_from_las,scene.T_lcc_from_las);
const checks=[];
for(const object of project.objects){
 const regions=project.annotations.boxes.filter(r=>object.region_ids.includes(r.id));
 assert.equal(object.task_qualification,false);assert.equal(object.reviews.length,0);
 for(const id of object.evidence_ids){const frame=JSON.parse(blob(id));if(frame.kind!=='recognition')continue;
  assert.deepEqual(frame.source_hashes,scene.source_hashes);
  const bytes=blob(frame.depth),validity=blob(frame.validity),rgb=blob(frame.rgb),result=JSON.parse(blob(frame.result));
  assert.equal(bytes.length,frame.width*frame.height*4);assert.equal(validity.length,frame.width*frame.height);assert.equal(result.image_sha256,hash(rgb));
  const comparison=P.compareEvidence(frame,new DataView(bytes.buffer,bytes.byteOffset,bytes.byteLength),regions,result);
  const c=comparison.stats;assert.equal(c.checked,c.offscreen+c.invalid_depth+c.depth_disagreement+c.visible);
  const masks=comparison.masks.map(m=>({index:m.index,category:m.category,inside:m.inside.length,outside:m.outside.length}));
  for(const m of masks)assert.equal(m.inside+m.outside,c.visible);
  const category=regions[0].category,expected=category==='chair'?{index:9,visible:420,inside:400,outside:20}:category==='screen'?{index:2,visible:125,inside:125,outside:0}:null;
  assert(expected,'Unexpected frozen object');assert.equal(c.visible,expected.visible);assert.equal(masks[expected.index].inside,expected.inside);assert.equal(masks[expected.index].outside,expected.outside);
  if(category==='screen')assert.equal(masks[expected.index].category,'computer','Frozen class disagreement must not be silently corrected');
  checks.push({object_id:object.object_id,evidence_id:id,region_category:category,stats:c,selected_mask:masks[expected.index]});
 }
}
assert.equal(checks.length,2);assert.equal(hash(fs.readFileSync(file)),hash(original));
const report={at:new Date().toISOString(),version:JSON.parse(fs.readFileSync(path.join(root,'package.json'))).version,frozen_revision:8,revision_sha256:hash(original),checks,production_modified:false,browser_gpu_tested:false,semantic_accuracy_accepted:false,real_robot_commands:false,interpretation:'Frozen archived evidence arithmetic only; outside-mask points are disagreements, not confirmed background. Invisible geometry stays unknown.'};
if(process.argv.includes('--record')){const dir=path.join(root,'artifacts','s1-'+report.at.slice(0,10).replaceAll('-',''));fs.mkdirSync(dir,{recursive:true});const out=path.join(dir,'evidence-comparison-'+Date.now()+'.json');fs.writeFileSync(out,JSON.stringify(report,null,2)+'\n',{flag:'wx'});console.log(out);}
console.log(JSON.stringify(report,null,2));

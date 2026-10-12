// Frozen v8 dry-run only. No production save, browser access, inference or robot commands.
const fs=require('node:fs'),path=require('node:path'),crypto=require('node:crypto'),assert=require('node:assert/strict'),C=require('../frontend/project-core.js'),B=require('../frontend/boxes.js');
const root=path.resolve(__dirname,'..'),file=path.join(root,'data/projects/k2/revisions/00000008.json'),raw=fs.readFileSync(file),doc=JSON.parse(raw),regions=doc.annotations.boxes,sha=b=>crypto.createHash('sha256').update(b).digest('hex');
const a=regions.find(r=>r.category==='chair'&&r.sequence===24),b=regions.find(r=>r.category==='chair'&&r.sequence===119);assert(a&&b);
const scene=JSON.parse(fs.readFileSync(path.join(root,'artifacts/k2-annotation-scene-v2/scene.json')));assert.deepEqual(doc.annotations.source_hashes,scene.source_hashes);assert.deepEqual(doc.annotations.T_lcc_from_las,scene.T_lcc_from_las);
const cases=[];
for(const batch of [false,true]){const objects=batch?[...doc.objects,...C.registerBatch(regions,doc.objects,()=>crypto.randomUUID())]:doc.objects,original=JSON.stringify(objects);
 const plan=C.identityAssociationPlan(regions,objects,a.id,b.id,crypto.randomUUID());C.validate({...doc,objects:plan.objects},scene,B);
 assert.equal(JSON.stringify(objects),original);assert.deepEqual(plan.region_ids,[a.id,b.id]);assert(plan.objects.every(o=>!o.task_qualification));assert(plan.objects.every(o=>!o.reviews.length));
 cases.push({scope:batch?'in_memory_batch_intake_then_association':'frozen_existing_plus_unregistered',before_objects:objects.length,after_objects:plan.objects.length,target:plan.target_id,source:plan.source_id,region_ids:plan.region_ids,evidence_count:plan.evidence_count,geometry_changed:plan.geometry_changed,identity_conclusion:plan.identity_conclusion});
}
assert.equal(sha(fs.readFileSync(file)),sha(raw));
const report={at:new Date().toISOString(),frozen_revision:8,revision_sha256:sha(raw),core_sha256:sha(fs.readFileSync(path.join(root,'frontend/project-core.js'))),scene_id:scene.scene_id,alignment_status:scene.alignment_status,cases,production_modified:false,browser_gpu_tested:false,semantic_accuracy_validated:false,real_robot_commands:false};
if(process.argv.includes('--record')){const dir=path.join(root,'artifacts/dataset-20260914');fs.mkdirSync(dir,{recursive:true});const out=path.join(dir,'identity-association-'+Date.now()+'.json');fs.writeFileSync(out,JSON.stringify(report,null,2)+'\n',{flag:'wx'});console.log(out);}console.log(JSON.stringify(report,null,2));

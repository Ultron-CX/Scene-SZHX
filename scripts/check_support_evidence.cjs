// Read-only frozen examples. Existing frames are reference observations, not invented original sources.
const fs=require('node:fs'),path=require('node:path'),crypto=require('node:crypto'),assert=require('node:assert/strict'),C=require('../frontend/project-core.js');
const root=path.resolve(__dirname,'..'),store=path.join(root,'data/projects/k2'),file=path.join(store,'revisions/00000008.json'),raw=fs.readFileSync(file),doc=JSON.parse(raw),regions=doc.annotations.boxes;
const sha=b=>crypto.createHash('sha256').update(b).digest('hex'),blob=id=>{assert.match(id,/^[a-f0-9]{64}$/);const b=fs.readFileSync(path.join(store,'blobs',id));assert.equal(sha(b),id);return b;};
const scene=JSON.parse(fs.readFileSync(path.join(root,'artifacts/k2-annotation-scene-v2/scene.json')));assert.deepEqual(doc.annotations.source_hashes,scene.source_hashes);assert.deepEqual(doc.annotations.T_lcc_from_las,scene.T_lcc_from_las);
const report=C.supportConflicts(regions),pairs=report.pairs,chosen=[...pairs.slice(0,3),...pairs.filter(p=>C.pairEvidenceSources(p,regions,doc.objects).some(s=>s.a!=='scene_reference'||s.b!=='scene_reference')).slice(0,1)];
const checks=[];
for(const p of chosen){const sources=C.pairEvidenceSources(p,regions,doc.objects),a=regions.find(r=>r.id===p.a),b=regions.find(r=>r.id===p.b),views=[];
 for(const s of sources){const m=JSON.parse(blob(s.id));if(m.kind!=='recognition')continue;
  const d=blob(m.depth),validity=blob(m.validity),rgb=blob(m.rgb),result=JSON.parse(blob(m.result));assert.equal(result.image_sha256,sha(rgb));
  const c=C.compareSupportPair(m,new DataView(d.buffer,d.byteOffset,d.byteLength),validity,a,b,result,scene);
  views.push({evidence_id:s.id,reference_for_a:s.a,reference_for_b:s.b,rgb_sha256:sha(rgb),visibility:c.visibility,a:c.a.stats,b:c.b.stats,masks:c.masks.slice(0,5),identity_conclusion:c.identity_conclusion});
 }
 checks.push({a:p.a,b:p.b,a_label:a.category+'-'+a.sequence,b_label:b.category+'-'+b.sequence,common_cells:p.common_cells,region_source_references:{a:sources.filter(s=>s.a==='region_reference').length,b:sources.filter(s=>s.b==='region_reference').length},views});
}
assert.equal(checks.length,4);assert(checks.every(c=>c.views.length===2));assert.equal(sha(fs.readFileSync(file)),sha(raw));
const output={schema:'scenesem.support-evidence-diagnostic.v1',at:new Date().toISOString(),frozen_revision:8,revision_sha256:sha(raw),core_sha256:sha(fs.readFileSync(path.join(root,'frontend/project-core.js'))),scene_id:scene.scene_id,alignment_status:scene.alignment_status,selection:'three highest overlaps plus first pair with object/region evidence reference; not representative accuracy sampling',pair_count:pairs.length,pairs_with_region_references:pairs.filter(p=>C.pairEvidenceSources(p,regions,doc.objects).some(s=>s.a==='region_reference'||s.b==='region_reference')).length,pairs_with_object_or_region_references:pairs.filter(p=>C.pairEvidenceSources(p,regions,doc.objects).some(s=>s.a!=='scene_reference'||s.b!=='scene_reference')).length,checks,production_modified:false,browser_gpu_tested:false,semantic_accuracy_validated:false,real_robot_commands:false};
if(process.argv.includes('--record')){const dir=path.join(root,'artifacts/dataset-20260914');fs.mkdirSync(dir,{recursive:true});const f=path.join(dir,'support-evidence-'+Date.now()+'.json');fs.writeFileSync(f,JSON.stringify(output,null,2)+'\n',{flag:'wx'});console.log(f);}
console.log(JSON.stringify(output,null,2));

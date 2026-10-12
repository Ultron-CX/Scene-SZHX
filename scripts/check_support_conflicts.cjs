// Frozen local diagnostic; never save a project revision or label.
const fs=require('node:fs'),path=require('node:path'),crypto=require('node:crypto'),C=require('../frontend/project-core.js');
const root=path.resolve(__dirname,'..'),file=path.join(root,'data/projects/k2/revisions/00000008.json'),raw=fs.readFileSync(file),doc=JSON.parse(raw);
const start=performance.now(),result=C.supportConflicts(doc.annotations.boxes),elapsed_ms=performance.now()-start;
const report={...result,at:new Date().toISOString(),scope:'frozen_v8_not_browser_not_accuracy',revision:8,scene_id:doc.annotations.scene_id,
 alignment_status:'trajectory_candidate_not_surface_certified',project_sha256:crypto.createHash('sha256').update(raw).digest('hex'),
 core_sha256:crypto.createHash('sha256').update(fs.readFileSync(path.join(root,'frontend/project-core.js'))).digest('hex'),elapsed_ms,labels_modified:false,
 pairs:result.pairs.map(p=>({...p,a_label:doc.annotations.boxes.find(b=>b.id===p.a).category+'-'+doc.annotations.boxes.find(b=>b.id===p.a).sequence,b_label:doc.annotations.boxes.find(b=>b.id===p.b).category+'-'+doc.annotations.boxes.find(b=>b.id===p.b).sequence}))};
if(process.argv.includes('--record')){const dir=path.join(root,'artifacts/dataset-20260914');fs.mkdirSync(dir,{recursive:true});const output=path.join(dir,'support-conflicts-'+Date.now()+'.json');fs.writeFileSync(output,JSON.stringify(report,null,2)+'\n',{flag:'wx'});console.log(output);}
console.log(JSON.stringify({stats:result.stats,pairs:result.pairs.length,elapsed_ms,examples:report.pairs.slice(0,5),semantic_accuracy_validated:false},null,2));

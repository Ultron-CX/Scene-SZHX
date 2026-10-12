// Read-only frozen diagnostic snapshot, not current browser state or accuracy.
const fs=require('node:fs'),path=require('node:path'),crypto=require('node:crypto');
const C=require('../frontend/project-core.js'),O=require('../frontend/object-core.js');
const root=path.resolve(__dirname,'..'),file=path.join(root,'data/projects/k2/revisions/00000008.json');
const bytes=fs.readFileSync(file),doc=JSON.parse(bytes),regions=doc.annotations.boxes;
const t=C.triage(doc.objects,regions,new Map(doc.objects.map(o=>[o.object_id,C.basis(o,regions)])),O.sizeAssessment);
const report={at:new Date().toISOString(),scope:'frozen_v8_not_live_browser_not_accuracy',scene_id:doc.annotations.scene_id,revision:8,
 project_sha256:crypto.createHash('sha256').update(bytes).digest('hex'),rule:'class_aabb_max_span_v1',progress:t.progress,
 size_checked:t.size_checked,size_unsupported:t.size_unsupported,anomalies:t.anomalies,information_count:t.information.length,
 default_anomaly_count:t.anomalies.length,old_combined_count:t.all.filter(x=>x.kind!=='size_suspect').length,
 source_code_sha256:Object.fromEntries(['frontend/project-core.js','frontend/object-core.js'].map(f=>[f,crypto.createHash('sha256').update(fs.readFileSync(path.join(root,f))).digest('hex')])),
 labels_modified:false,human_reviewed:0,semantic_accuracy_validated:false,real_robot_commands:false};
if(process.argv.includes('--record')){const dir=path.join(root,'artifacts/dataset-20260914');fs.mkdirSync(dir,{recursive:true});const output=path.join(dir,'triage-'+Date.now()+'.json');fs.writeFileSync(output,JSON.stringify(report,null,2)+'\n',{flag:'wx'});console.log(output);}
console.log(JSON.stringify({progress:t.progress,size_checked:t.size_checked,size_unsupported:t.size_unsupported,default_anomaly_count:t.anomalies.length,old_combined_count:report.old_combined_count,semantic_accuracy_validated:false},null,2));

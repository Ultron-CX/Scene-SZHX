// Repeatable local S1 acceptance evidence; never overwrite the migration records.
const fs=require('node:fs'),path=require('node:path'),cp=require('node:child_process'),crypto=require('node:crypto');
const root=path.resolve(__dirname,'..'),python=path.join(root,'.venv','Scripts','python.exe');
const commands=[['frontend-regression',process.execPath,['scripts/test.cjs']],['local-store-and-isolated-http',python,['-X','utf8','scripts/test_project_store.py']],['bundle-import-and-clean-store-restore',python,['-X','utf8','scripts/test_bundle_import.py']]];
commands.push(['internal-semantic-dataset',python,['-X','utf8','scripts/test_semantic_dataset.py']]);
commands.push(['spatial-context-store-bundle-dataset',python,['-X','utf8','scripts/test_spatial_context.py']]);
commands.push(['business-context-store-bundle-dataset',python,['-X','utf8','scripts/test_business_context.py']]);
commands.push(['selected-material-offline-delivery',python,['-X','utf8','scripts/test_material_delivery.py']]);
commands.push(['project-bundle-material-restore',python,['-X','utf8','scripts/test_bundle_materials.py']]);
commands.push(['source-materials-local-archive',python,['-X','utf8','scripts/test_source_materials.py']]);
commands.push(['robot-semantic-map-adapter',python,['-X','utf8','scripts/test_robot_semantic_map.py']]);
commands.push(['r1-sample-preparation',python,['-X','utf8','scripts/test_r1_sample.py']]);
commands.push(['r1-sample-http-isolation',python,['-X','utf8','scripts/test_sample_scope.py']]);
if(process.argv.includes('--http'))commands.push(['live-http-read-only',process.execPath,['scripts/check_http.cjs']]);
const results=commands.map(([name,exe,args])=>{const r=cp.spawnSync(exe,args,{cwd:root,encoding:'utf8',maxBuffer:4*1024*1024,windowsHide:true});return {name,exit_code:r.status,stdout:r.stdout,stderr:r.stderr,error:r.error?.message};});
const files=['frontend/initial-core.js','frontend/test-initial-workflow.cjs','frontend/project-core.js','frontend/project-ui.js','frontend/project.css','frontend/object-core.js','frontend/object-ui.js','frontend/completion-ui.js','frontend/scene-app.js','frontend/workbench.html','frontend/test-project.cjs','scripts/project_store.py','scripts/serve_annotation_tool.py','scripts/test_project_store.py','scripts/bundle_import.py','scripts/region_validation.py','scripts/test_bundle_import.py'];
files.push('frontend/test-evidence-comparison.cjs','frontend/test-review-queue.cjs','frontend/test-workbench-ux.cjs','frontend/workbench-ux.js','frontend/workflow.css','frontend/test-batch-intake.cjs','frontend/test-camera-navigation.cjs','scripts/check_evidence_comparison.cjs','scripts/check_s1.cjs');
files.push('frontend/test-issue-list.cjs','frontend/test-support-conflicts.cjs','scripts/check_support_conflicts.cjs','scripts/check_triage.cjs','scripts/semantic_dataset.py','scripts/export_semantic_dataset.py','scripts/test_semantic_dataset.py');
 files.push('frontend/test-zone-memberships.cjs','frontend/test-support-evidence.cjs','scripts/check_support_evidence.cjs','frontend/test-identity-association.cjs','scripts/check_identity_association.cjs','frontend/test-spatial-context.cjs','scripts/test_spatial_context.py','scripts/check_spatial_delivery.py');
files.push('scripts/test_bundle_materials.py','scripts/check_material_delivery.py','scripts/test_material_delivery.py','scripts/test_source_materials.py','frontend/test-business-date.cjs','frontend/test-business-context.cjs','scripts/test_business_context.py','scripts/check_business_delivery.py','scripts/check_http.cjs');
files.push('frontend/test-r1-dataset-flow.cjs','scripts/r1_flow_bridge.py','frontend/sample-scope.js','frontend/test-sample-scope.cjs','scripts/sample_scope.py','scripts/prepare_r1_sample.py','scripts/test_r1_sample.py','scripts/test_sample_scope.py','scripts/robot_semantic_map.py','scripts/test_robot_semantic_map.py');
const report={product:'SceneSem',version:JSON.parse(fs.readFileSync(path.join(root,'package.json'))).version,at:new Date().toISOString(),scope:'R1 simplified UI, paired initial/undo and synthetic UI-to-real-store dataset flow; legacy S1/D2/D3/D4 compatibility; no browser or human quality acceptance',passed:results.every(r=>r.exit_code===0),results,sha256:Object.fromEntries(files.map(f=>[f,crypto.createHash('sha256').update(fs.readFileSync(path.join(root,f))).digest('hex')])),live_user_labels_modified:false,browser_gpu_tested:false,real_seed_objects_human_reviewed:0,synthetic_seed_objects_roundtripped:25,real_robot_commands:false};
report.sha256['frontend/test-zone-recheck.cjs']=crypto.createHash('sha256').update(fs.readFileSync(path.join(root,'frontend/test-zone-recheck.cjs'))).digest('hex');
if(process.argv.includes('--record')){const dir=path.join(root,'artifacts','s1-'+report.at.slice(0,10).replaceAll('-',''));fs.mkdirSync(dir,{recursive:true});const file=path.join(dir,'validation-'+Date.now()+'.json');fs.writeFileSync(file,JSON.stringify(report,null,2)+'\n',{flag:'wx'});console.log(file);}
for(const r of results)console.log(r.name+': '+(r.exit_code===0?'PASS':'FAIL')+(r.exit_code===0?'':'\n'+r.stdout+'\n'+r.stderr));
process.exitCode=report.passed?0:1;

const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm'),crypto=require('node:crypto');
const P=require('./project-core.js'),B=require('./boxes.js'),O=require('./object-core.js');
const meta={scene_id:'synthetic-s1',coordinate_frame:'TEST',source_hashes:{fixture:'test'},T_lcc_from_las:[]};
const r={id:'region-1',category:'chair',sequence:1,status:'reviewed',min:[0,0,0],max:[1,1,1]},r2={...r,id:'region-2',sequence:2,min:[1,0,0],max:[2,1,1]};
const o=P.register(r,[],crypto.randomUUID());assert(o.object_id.startsWith('obj-'));assert.deepEqual(o.reviews,[]);assert.equal(o.original_evidence,'missing');assert.throws(()=>P.register(r,[o],crypto.randomUUID()));
const linked=P.link(o,r2,[o]);assert.deepEqual(linked.region_ids,['region-1','region-2']);assert.deepEqual(o.region_ids,['region-1']);
const eid='a'.repeat(64),enriched=structuredClone(r);enriched.provenance={cleanup_source:{provenance:{evidence_ids:[eid]}}};assert.deepEqual(P.evidenceIds(enriched),[eid]);assert.deepEqual(P.reconcile([o],[enriched])[0].evidence_ids,[eid]);
assert.notEqual(P.basis(o,[r]),P.basis(o,[{...r,category:'table'}]));assert.notEqual(P.basis(o,[r]),P.basis(o,[]));
const document={schema:'scenesem.project.v1',revision:1,annotations:B.save([r,r2],meta),objects:[linked],real_robot_commands:false};assert.deepEqual(P.validate(document,meta,B),[r,r2]);assert.throws(()=>P.validate({...document,objects:[linked,linked]},meta,B));assert.throws(()=>P.validate(document,{...meta,T_lcc_from_las:[1]},B));
const near=.1,far=10,A=(far+near)/(near-far),k=2*far*near/(near-far),frame={width:64,height:64,near,far,projection:[1,0,0,0,0,1,0,0,0,0,A,-1,0,0,k,0],inverse:[1,0,0,0,0,1,0,0,0,0,0,1/k,0,0,-1,A/k],camera_position:[0,0,0],world_matrix:[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1],fov:60,horizontal_fov:false,pose_id:'synthetic',depth:new Float32Array([NaN,.2])};
const bytes=Buffer.from(P.framePayload(frame).depth_f32le,'base64');assert(Number.isNaN(bytes.readFloatLE(0)));assert(Math.abs(bytes.readFloatLE(4)-.2)<1e-7);
const archived={width:64,height:64,camera:frame},xyz=P.unproject(archived,31,32,(2-near)/(far-near));assert(Math.abs(xyz[2]+2)<1e-10);assert.deepEqual(P.projectPoint(archived,xyz).slice(0,2),[31.5,32.5]);assert.equal(P.unproject(archived,1,2,NaN),null);
// Original evidence references survive mask lifting and compatible view merges.
const depthFrame={...frame,depth:new Float32Array(4096).fill((2-near)/(far-near))};
const result={width:64,height:64,backend:'instance',model:'synthetic',model_sha256:'x',image_sha256:'im',evidence_id:eid};
const component={category:'chair',label:'chair',score:.8,instance_id:1,runs:Array.from({length:20},(_,i)=>[20+i,20,40])};
const lifted=O.fromMask(component,depthFrame,result);assert(lifted.length);assert.deepEqual(lifted[0].provenance.evidence_ids,[eid]);const next=structuredClone(lifted[0]);next.provenance.evidence_ids=['b'.repeat(64)];O.merge(lifted,[next]);assert(lifted[0].provenance.evidence_ids.includes('b'.repeat(64)));

// Node VM only: exercise actual S1 buttons against a synthetic in-memory API.
const nodes=new Map();
function element(){return {id:'',value:'',checked:false,hidden:false,disabled:false,children:[],style:{setProperty(){}},classList:{toggle(){}},textContent:'',append(...x){this.children.push(...x);},prepend(...x){this.children.unshift(...x);},add(x){this.children.push(x);},replaceChildren(...x){this.children=x;},setAttribute(){},addEventListener(){},getContext:()=>({drawImage(){},fillRect(){}}),getBoundingClientRect:()=>({left:0,top:0,width:64,height:64}),set innerHTML(v){for(const m of v.matchAll(/id="([^"]+)"/g))get(m[1]);get('archiveEvidence').checked=true;},showModal(){this.open=true;},close(){this.open=false;},click(){this.onclick?.();},remove(){}};}
function get(id){if(!nodes.has(id)){const e=element();e.id=id;nodes.set(id,e);}return nodes.get(id);}
function createElement(){const e=element();e.insertBefore=(child,before)=>e.children.splice(Math.max(0,e.children.indexOf(before)),0,child);Object.defineProperty(e,'id',{get(){return this.elementId||'';},set(value){this.elementId=value;nodes.set(value,this);}});return e;}
get('spatialForm').insertBefore=function(child){this.children.push(child);};
const local=new Map(),versions=[];let rejectSave=false,rejectImport=false,inferFailure='',inferCalls=0,lastInferOptions;
const captureFrame={...depthFrame,image:'synthetic-only'},captureResult={...result,scene_id:meta.scene_id,source_hashes:meta.source_hashes,components:[component]};
const fetchMock=async(url,options)=>{let status=200,data;
 if(url==='/project/save'){
  const p=JSON.parse(options.body);if(rejectSave||p.expected_revision!==versions.length){status=409;data={error:'synthetic conflict'};}
  else{const objects=p.objects;for(const x of objects){if(x.pending_mask_correction){x.mask_corrections=[...(x.mask_corrections||[]),{...x.pending_mask_correction,raw_category:'chair',scope:'evidence_mask_only_not_region_or_identity',revision:versions.length+1,at:'synthetic'}];delete x.pending_mask_correction;}if(x.pending_review){x.reviews.push({...x.pending_review,revision:versions.length+1,at:'synthetic'});delete x.pending_review;x.import_requires_review=false;}x.review_stale=false;}
   const d={schema:'scenesem.project.v1',revision:versions.length+1,annotations:p.annotations,objects,real_robot_commands:false,...(p.spatial_context?{spatial_context:p.spatial_context}:{}),...(p.business_context?{business_context:p.business_context}:{})};versions.push(structuredClone(d));data={head:versions.length,document:d};}
 }else if(url==='/project/import/preview')data={import_id:'a'.repeat(32),source_revision:1,scene_id:meta.scene_id,objects:1,regions:2,evidence:0,expanded_bytes:1000,missing_original_evidence:1,missing_region_links:0,document:structuredClone(document)};
 else if(url==='/project/import/commit'){
  if(rejectImport){status=503;data={error:'synthetic import failure; no UI switch'};}
  else{const p=JSON.parse(options.body);await fetchMock('/project/save',{body:JSON.stringify(p.checkpoint)});const checkpoint_revision=versions.length;const imported=structuredClone(document);imported.revision=versions.length+1;imported.objects.forEach(o=>{o.import_requires_review=true;o.review_stale=true;});versions.push(imported);data={head:versions.length,document:imported,checkpoint_revision};}
 }
 else if(url==='/health')data={spatial_context:true,zone_boundaries:true,business_context:true,source_materials:true,dataset_materials:true,bundle_materials:true};
 else if(url==='/project/revisions')data={revisions:versions.map((_,i)=>i+1)};
 else if(url.startsWith('/project/revision/'))data={head:versions.length,document:structuredClone(versions[Number(url.split('/').at(-1))-1])};
 else if(url.endsWith('/manifest'))data={...archived,kind:'recognition',result:'result',valid_pixels:4096,capture_scope:'synthetic'};
 else if(url.endsWith('/result'))data=captureResult;
 else if(url.endsWith('/depth'))return {ok:true,arrayBuffer:async()=>captureFrame.depth.buffer};
 else if(url==='/evidence/capture')data={evidence_id:'b'.repeat(64)};
 else throw Error(url);
 return {ok:status===200,status,json:async()=>data};
};
const context={BoxCore:B,ProjectCore:P,ObjectCore:O,structuredClone,btoa:s=>Buffer.from(s,'binary').toString('base64'),console,setTimeout,clearTimeout,crypto:crypto.webcrypto,confirm:()=>true,prompt:()=> '1',fetch:fetchMock,
 Option:function(text,value){return {textContent:text,value};},localStorage:{getItem:k=>local.get(k)||null,setItem:(k,v)=>local.set(k,v)},document:{getElementById:get,createElement,querySelector:selector=>selector==='dialog[open]'?[...nodes.values()].find(n=>n.open)||null:get(selector),querySelectorAll:()=>[...nodes.values()],body:element(),addEventListener(){},activeElement:{tagName:'BODY',blur(){}}},window:{addEventListener(){}},metaFixture:meta,regionsFixture:[r,r2],
 CompletionUI:{active:()=>false},CleanupUI:{active:()=>false},Image:class {async decode(){}},pc:{Vec3:class {}}};
vm.createContext(context);vm.runInContext(fs.readFileSync(__dirname+'/scene-app.js','utf8').replace(/init\(\);\s*$/,''),context);const run=s=>vm.runInContext(s,context);
context.ObjectUI={busy:()=>!!context.ProjectUI?.busy()};run("meta=metaFixture;boxes=structuredClone(regionsFixture);selected=boxes[0].id;storageKey='synthetic-test-only';filterScene=()=>{};resetDraft=()=>{};loadGS=()=>{};fitBox=()=>{};");
vm.runInContext(fs.readFileSync(__dirname+'/project-ui.js','utf8'),context);
const tick=async()=>{for(let i=0;i<5;i++)await new Promise(r=>setImmediate(r));};
(async()=>{
 get('objectRegister').onclick();assert.equal(get('projectObject').children.length,2);assert.equal(run('boxes.length'),2);const firstId=get('projectObject').value;
 get('objectAlias').value='Test chair';get('objectAlias').onchange();await get('projectSave').onclick();assert.equal(versions.length,1);assert.equal(versions[0].objects[0].alias,'Test chair');assert.equal(versions[0].objects[0].object_id,firstId);assert.deepEqual(versions[0].annotations.boxes,[r,r2]);
 run('selected=boxes[1].id');get('objectLink').onclick();await get('projectSave').onclick();assert.deepEqual(versions[1].objects[0].region_ids,['region-1','region-2']);
 get('reviewerName').value='Synthetic tester';get('reviewNotes').value='Test only; not a real review';
 const reviewDraft=local.get('scenesem-project-draft-v1:'+meta.scene_id);await get('objectReviewSave').onclick();assert.equal(versions.length,2);assert.equal(local.get('scenesem-project-draft-v1:'+meta.scene_id),reviewDraft,'preview never stashes a pending review');assert.equal(get('reviewConfirmDialog').open,true);get('reviewCancel').onclick();assert.equal(get('reviewConfirmDialog').open,false);assert.equal(get('reviewNotes').value,'Test only; not a real review');
 await get('objectReviewSave').onclick();get('reviewNotes').value='changed after preview';await get('reviewConfirm').onclick();assert.match(get('reviewConfirmError').textContent,/过期/);assert.equal(versions.length,2);get('reviewCancel').onclick();
 await get('objectReviewSave').onclick();run('boxes[0].category="table"');await get('reviewConfirm').onclick();assert.match(get('reviewConfirmError').textContent,/过期/);assert.equal(versions.length,2);run('boxes[0].category="chair"');get('reviewCancel').onclick();
 await get('objectReviewSave').onclick();rejectSave=true;await get('reviewConfirm').onclick();assert.equal(versions.length,2);assert.equal(get('reviewConfirmDialog').open,true);assert.match(get('reviewConfirmError').textContent,/conflict/);assert.equal(local.get('scenesem-project-draft-v1:'+meta.scene_id),reviewDraft);rejectSave=false;
 await get('reviewConfirm').onclick();assert.equal(get('reviewConfirmDialog').open,false);assert.equal(versions.at(-1).objects[0].reviews.length,1);assert.equal(versions.at(-1).objects[0].reviews[0].checks.coverage,'pending');assert.match(get('reviewQueueSummary').textContent,/未完成核查 1/);await get('reviewConfirm').onclick();assert.equal(versions.length,3,'duplicate confirmation cannot create another review');
 rejectSave=true;get('objectAlias').value='unsaved';get('objectAlias').onchange();await get('projectSave').onclick();assert.match(get('projectStatus').textContent,/conflict/);assert.equal(versions.length,3);rejectSave=false;
 const before=JSON.parse(run('JSON.stringify(boxes)'));context.prompt=()=>{throw Error('prompt() is not supported.');};
 await get('projectLoad').onclick();assert.equal(get('projectLoadDialog').open,true);assert.equal(versions.length,3,'opening version selector is read-only');get('projectLoadCancel').onclick();assert.equal(get('projectLoadDialog').open,false);assert.equal(versions.length,3);
 await get('projectLoad').onclick();run('boxes[0].category="table"');await get('projectLoadConfirm').onclick();assert.match(get('projectLoadError').textContent,/新修改/);assert.equal(versions.length,3);run('boxes[0].category="chair"');get('projectLoadCancel').onclick();
 await get('projectLoad').onclick();get('projectLoadVersion').value='999';await get('projectLoadConfirm').onclick();assert.equal(versions.length,3);assert.match(get('projectLoadError').textContent,/请选择/);get('projectLoadVersion').value='1';rejectSave=true;await get('projectLoadConfirm').onclick();assert.equal(versions.length,3);assert.deepEqual(JSON.parse(run('JSON.stringify(boxes)')),before);assert.equal(get('projectLoadDialog').open,true);rejectSave=false;
 await get('projectLoadConfirm').onclick();assert.equal(get('projectLoadDialog').open,false);assert.equal(versions.length,4,'load must first save current state as checkpoint');assert.equal(versions[3].objects[0].alias,'unsaved');assert.deepEqual(JSON.parse(run('JSON.stringify(boxes)')),before);await get('projectSave').onclick();assert.equal(versions.at(-1).objects[0].alias,'Test chair');assert.equal(versions.at(-1).objects[0].object_id,firstId);
 await get('objectRemove').onclick();assert.equal(versions.at(-1).objects.length,0);assert.equal(versions.at(-2).objects.length,1);assert.deepEqual(JSON.parse(run('JSON.stringify(boxes)')),before,'removing object archive must not delete geometry');
 assert.equal(context.ProjectUI.busy(),false);await tick();
 const importBaseline=versions.length;
 get('projectImportFile').files=[{name:'fixture.zip',size:1000}];await get('projectImportFile').onchange();assert.equal(versions.length,importBaseline,'preview must not save or switch labels');get('projectImportCancel').onclick();assert.deepEqual(JSON.parse(run('JSON.stringify(boxes)')),before);
 await get('projectImportFile').onchange();run('boxes[0].category="table"');await get('projectImportConfirm').onclick();assert.equal(versions.length,importBaseline,'edits after preview must prevent commit');assert.match(get('projectStatus').textContent,/新修改/);run('boxes[0].category="chair"');get('projectImportCancel').onclick();
 await get('projectImportFile').onchange();rejectImport=true;await get('projectImportConfirm').onclick();assert.equal(versions.length,importBaseline);assert.deepEqual(JSON.parse(run('JSON.stringify(boxes)')),before);assert.equal(context.ProjectUI.busy(),false);rejectImport=false;
 await get('projectImportConfirm').onclick();assert.equal(versions.length,importBaseline+2);assert.deepEqual(versions.at(-2).annotations.boxes,before);assert.equal(versions.at(-1).objects[0].object_id,linked.object_id);assert.deepEqual(JSON.parse(run('JSON.stringify(boxes)')),[r,r2]);
 // Evidence-only recognition: actual UI control path, synthetic capture/inference, no geometry updates.
 context.ObjectUI.capture=async()=>captureFrame;
 context.ObjectUI.infer=async(f,options)=>{inferCalls++;lastInferOptions=options;if(inferFailure)throw Error(inferFailure);return captureResult;};
 context.CompletionCore={visibleSeeds:()=>[{},{},{}],targetROI:()=>[0,0,64,64]};
 run("gsEntity={};view='points';setView=v=>{view=v;};selected='wrong-region';");
 const baselineVersions=versions.length;await get('objectRecognize').onclick();assert.equal(inferCalls,0);assert.equal(versions.length,baselineVersions);assert.match(get('projectStatus').textContent,/不一致/);
 run("selected=boxes[0].id");get('objectConfidence').value='.6';get('archiveEvidence').checked=false;get('filterEnabled').checked=true;get('clipZ').value='1';get('clipZ').max='5';
 context.ObjectUI.hasPending=()=>true;await get('objectRecognize').onclick();assert.equal(inferCalls,0);assert.match(get('projectStatus').textContent,/隐藏候选/);context.ObjectUI.hasPending=()=>false;
 inferFailure='synthetic inference failure';await get('objectRecognize').onclick();assert.equal(context.ProjectUI.busy(),false);assert.equal(run('view'),'points');assert.equal(get('filterEnabled').checked,true);assert.equal(get('clipZ').value,'1');assert.equal(versions.length,baselineVersions);
 inferFailure='';await get('objectRecognize').onclick();assert.equal(versions.length,baselineVersions+1);assert.deepEqual(versions.at(-1).annotations.boxes,[r,r2]);assert.equal(lastInferOptions.backend,'instance');assert(lastInferOptions.frame.depth_f32le,'archive explicitly requested even with automatic archiving off');assert.equal(versions.at(-1).objects[0].reviews.length,0);assert.deepEqual(versions.at(-1).objects[0].evidence_ids,[eid]);assert.equal(get('evidenceDialog').open,true);assert.match(get('evidenceInfo').textContent,/未用于生成关联区域/);
 get('evidenceClose').onclick();rejectSave=true;await get('objectRecognize').onclick();assert.equal(versions.length,baselineVersions+1);assert.match(get('projectStatus').textContent,/证据已归档.*未确认保存/);assert.equal(context.ProjectUI.busy(),false);assert.equal(run('view'),'points');rejectSave=false;
 const callsBefore=inferCalls;run('boxes[0].geometry={type:"surface-voxels",cells:[[0,0,0]],cell:.04,origin:[0,0,0]}');context.CompletionCore.visibleSeeds=()=>[];await get('objectRecognize').onclick();assert.equal(inferCalls,callsBefore);assert.match(get('projectStatus').textContent,/可见支持点/);
 context.CompletionCore.visibleSeeds=()=>[{},{},{}];captureResult.source_hashes={wrong:'batch'};await get('objectRecognize').onclick();assert.match(get('projectStatus').textContent,/批次不一致/);assert.equal(versions.length,baselineVersions+1);assert.equal(lastInferOptions.detail,'target');assert.deepEqual(lastInferOptions.target_roi,[0,0,64,64]);captureResult.source_hashes=meta.source_hashes;run('delete boxes[0].geometry');
 assert.deepEqual(JSON.parse(run('JSON.stringify(boxes)')),[r,r2]);
 // Preview/cancel/stale/save/revert of a mask category never edits the raw result or regions.
 get('reviewerName').value='synthetic';get('reviewNotes').value='test imported record only';await get('objectReviewSave').onclick();await get('reviewConfirm').onclick();
 get('objectEvidence').children[0].onclick();await tick();get('evidenceMask').value='0';get('evidenceMask').onchange();
 // Zoom clicks must map back to original pixels, not use the full-frame origin.
 const originalRuns=component.runs;component.runs=[[30,30,31]];
 get('evidenceClose').onclick();get('objectEvidence').children[0].onclick();await tick();get('evidenceMask').value='0';get('evidenceMask').onchange();
 get('evidenceZoom').checked=true;get('evidenceZoom').onchange();assert.equal(get('evidenceCanvas').width,49);assert.equal(get('evidenceCanvas').height,49);
 get('evidenceCanvas').onclick({clientX:32,clientY:32});assert.match(get('evidencePixel').textContent,/像素 \(30, 30\)/);assert.equal(get('evidenceHitMasks').children[1].value,'0');
 get('evidenceCanvas').onclick({clientX:-10,clientY:100});assert.match(get('evidencePixel').textContent,/像素 \(6, 54\)/,'clicks are clamped to the crop');
 component.runs=originalRuns;get('evidenceClose').onclick();get('objectEvidence').children[0].onclick();await tick();get('evidenceMask').value='0';get('evidenceMask').onchange();
 const correctionBaseline=versions.length,rawResultBefore=JSON.stringify(captureResult);get('correctionPreview').onclick();assert.match(get('correctionError').textContent,/填写/);
 get('correctionCategory').value='screen';get('correctionReviewer').value='synthetic';get('correctionNotes').value='VM only';get('correctionPreview').onclick();assert.equal(get('correctionConfirmation').hidden,false);assert.equal(versions.length,correctionBaseline);
 get('correctionCancel').onclick();assert.equal(get('correctionConfirmation').hidden,true);assert.equal(versions.length,correctionBaseline);
 get('correctionPreview').onclick();run('boxes[0].category="table"');await get('correctionConfirm').onclick();assert.match(get('correctionError').textContent,/过期/);assert.equal(versions.length,correctionBaseline);run('boxes[0].category="chair"');
 get('correctionPreview').onclick();rejectSave=true;await get('correctionConfirm').onclick();assert.equal(versions.length,correctionBaseline);assert.equal(context.ProjectUI.busy(),false);rejectSave=false;
 await get('correctionConfirm').onclick();assert.equal(versions.length,correctionBaseline+1);assert.equal(versions.at(-1).objects[0].mask_corrections[0].category,'screen');assert.equal(P.maskCategory(versions.at(-1).objects[0],eid,0,'chair'),'screen');
 get('correctionCategory').value='__raw__';get('correctionPreview').onclick();await get('correctionConfirm').onclick();assert.equal(versions.length,correctionBaseline+2);assert.equal(versions.at(-1).objects[0].mask_corrections.length,2);assert.equal(P.maskCategory(versions.at(-1).objects[0],eid,0,'chair'),'chair');assert.deepEqual(versions.at(-1).annotations.boxes,[r,r2]);assert.equal(JSON.stringify(captureResult),rawResultBefore);
 // Queue navigation is read-only and cannot discard an in-progress review form.
 run('boxes.push({...structuredClone(regionsFixture[0]),id:"region-3",sequence:3});selected="region-3"');get('objectRegister').onclick();const secondId=get('projectObject').value;
 const queueBaseline=versions.length,queueDraft=local.get('scenesem-project-draft-v1:'+meta.scene_id),queueGeometry=run('JSON.stringify(boxes)');
 get('reviewQueueFilter').value='needs_work';get('reviewQueueFilter').onchange();assert.equal(get('reviewQueueNext').disabled,false);get('reviewQueueNext').onclick();assert.notEqual(get('projectObject').value,secondId);assert.equal(run('selected'),'region-3','queue selection does not move the camera or choose a spatial region');
 const queueFirst=get('projectObject').value;get('reviewNotes').value='Unsubmitted notes';get('reviewQueueNext').onclick();assert.equal(get('projectObject').value,queueFirst);assert.match(get('projectStatus').textContent,/未提交审核/);assert.equal(get('reviewNotes').value,'Unsubmitted notes');
 get('projectObject').value=secondId;get('projectObject').onchange();assert.equal(get('projectObject').value,queueFirst,'dropdown has the same discard guard');get('objectReviewReset').onclick();get('reviewQueueNext').onclick();assert.equal(get('projectObject').value,secondId);
 get('reviewQueueFilter').value='recorded';get('reviewQueueFilter').onchange();assert.equal(get('reviewQueueNext').disabled,true);assert.equal(get('projectObject').value,secondId);assert.match(get('objectNextStep').textContent,/不在筛选内/);
 get('reviewQueueFilter').value='no_evidence';get('reviewQueueFilter').onchange();assert.equal(get('reviewQueueNext').disabled,true);assert.equal(versions.length,queueBaseline);assert.equal(local.get('scenesem-project-draft-v1:'+meta.scene_id),queueDraft);assert.equal(run('JSON.stringify(boxes)'),queueGeometry);
 // Batch intake is one explicit atomic save, not geometry edits or human approval.
 run('boxes.push({...structuredClone(regionsFixture[0]),id:"region-4",sequence:4},{...structuredClone(regionsFixture[1]),id:"region-5",sequence:5})');
 const batchGeometry=run('JSON.stringify(boxes)'),batchVersions=versions.length,batchLocal=local.get('scenesem-project-draft-v1:'+meta.scene_id);
 get('reviewNotes').value='keep this';get('batchRegister').onclick();assert.match(get('projectStatus').textContent,/未提交审核/);get('objectReviewReset').onclick();
 context.ObjectUI.hasPending=()=>true;get('batchRegister').onclick();assert.match(get('projectStatus').textContent,/未处理/);context.ObjectUI.hasPending=()=>false;
 get('batchRegister').onclick();assert.equal(get('batchRegisterDialog').open,true);assert.match(get('batchRegisterSummary').textContent,/新增 2/);assert.equal(versions.length,batchVersions);assert.equal(local.get('scenesem-project-draft-v1:'+meta.scene_id),batchLocal);get('batchRegisterCancel').onclick();assert.equal(get('batchRegisterDialog').open,false);
 get('batchRegister').onclick();run('boxes[4].category="table"');await get('batchRegisterConfirm').onclick();assert.match(get('batchRegisterError').textContent,/过期/);assert.equal(versions.length,batchVersions);run('boxes[4].category=regionsFixture[1].category');get('batchRegisterCancel').onclick();
 get('batchRegister').onclick();rejectSave=true;await get('batchRegisterConfirm').onclick();assert.match(get('batchRegisterError').textContent,/conflict/);assert.equal(versions.length,batchVersions);assert.equal(local.get('scenesem-project-draft-v1:'+meta.scene_id),batchLocal);assert.equal(get('batchRegisterDialog').open,true);rejectSave=false;
 await get('batchRegisterConfirm').onclick();assert.equal(versions.length,batchVersions+1);assert.equal(get('batchRegisterDialog').open,false);assert.equal(run('JSON.stringify(boxes)'),batchGeometry);const added=versions.at(-1).objects.filter(x=>x.region_ids.includes('region-4')||x.region_ids.includes('region-5'));assert.equal(added.length,2);for(const x of added){assert.equal(x.identity_state,'candidate_not_human_confirmed');assert.deepEqual(x.reviews,[]);assert.equal(x.task_qualification,false);}
 await get('batchRegisterConfirm').onclick();assert.equal(versions.length,batchVersions+1);get('batchRegister').onclick();assert.match(get('projectStatus').textContent,/没有重复收录/);
 // Dataset preview/download never writes versions or silently saves drafts.
 const datasetVersions=versions.length;get('datasetExport').onclick();assert.equal(get('datasetDialog').open,true);get('datasetCancel').onclick();assert.equal(get('datasetDialog').open,false);
 get('datasetExport').onclick();get('objectAlias').value='changed for guard';get('objectAlias').onchange();get('datasetDownload').onclick();assert.match(get('datasetError').textContent,/先保存/);assert.equal(versions.length,datasetVersions);get('datasetCancel').onclick();await get('projectSave').onclick();
 const links=[],originalCreate=context.document.createElement;context.document.createElement=tag=>{const el=originalCreate(tag);if(tag==='a')links.push(el);return el;};get('datasetExport').onclick();get('datasetDownload').onclick();assert.equal(links.at(-1).href,'/project/dataset/'+versions.length);assert.equal(versions.length,datasetVersions+1);get('datasetDownload').onclick();assert.equal(links.length,1);context.document.createElement=originalCreate;
 const issueVersions=versions.length;assert.match(get('issueSummary').textContent,/疑似异常 0/);assert.equal(get('issueLocate').disabled,true);assert.equal(get('issueSelect').children.length,0);assert.match(get('issueProgress').textContent,/未完成核查/);
 get('issueScope').value='information';get('issueScope').onchange();assert(get('issueSelect').children.length>0);assert.match(get('issueSummary').textContent,/资料\/复核提示/);
 get('reviewNotes').value='do not lose';get('issueScope').value='all';get('issueScope').onchange();assert.equal(get('reviewNotes').value,'do not lose','filtering cannot clear the review form');get('issueLocate').onclick();assert.match(get('projectStatus').textContent,/未提交审核/);get('objectReviewReset').onclick();run('chooseRegion=b=>{selected=b.id}');get('issueLocate').onclick();assert.equal(versions.length,issueVersions);assert(run('boxes.some(b=>b.id===selected)'));get('issueScope').value='anomalies';get('issueScope').onchange();assert.equal(get('issueLocate').disabled,true);
 const supportBaseline=versions.length,supportDraft=local.get('scenesem-project-draft-v1:'+meta.scene_id);get('supportCheck').onclick();assert.equal(get('supportDialog').open,true);assert.equal(get('supportLocateA').disabled,true);get('supportClose').onclick();
 run('boxes[0].geometry={type:"surface-voxels",origin:[0,0,0],cell:.04,cells:Array.from({length:10},(_,i)=>[i,0,0])};boxes[1].geometry=structuredClone(boxes[0].geometry)');
 get('supportCheck').onclick();assert.equal(get('supportPair').children.length,1);assert.match(get('supportDetail').textContent,/共享 10/);assert.equal(get('supportLocateB').disabled,false);
 run('boxes[1].category="table"');get('supportLocateA').onclick();assert.match(get('supportError').textContent,/已变化/);get('supportClose').onclick();get('supportCheck').onclick();get('supportLocateB').onclick();assert.equal(run('selected'),r2.id);assert.equal(get('supportDialog').open,false);assert.equal(versions.length,supportBaseline);assert.equal(local.get('scenesem-project-draft-v1:'+meta.scene_id),supportDraft);run('delete boxes[0].geometry;delete boxes[1].geometry;boxes[1].category=regionsFixture[1].category');
 // Pair replay uses only GETs; cancellation/races/missing evidence cannot change data or leave a misleading image.
 run('boxes[0].geometry={type:"surface-voxels",origin:[0,0,0],cell:.01,cells:Array.from({length:10},(_,i)=>[i,0,-201])};boxes[1].geometry=structuredClone(boxes[0].geometry)');
 const pairFetch=context.fetch,pairImage=context.Image,pairRequests=[];let pairReject=false,pairDelay=null;
 context.fetch=async(url,options)=>{pairRequests.push({url,method:options?.method||'GET'});if(pairDelay){const wait=pairDelay;pairDelay=null;await wait;}if(pairReject)return {ok:false,json:async()=>({error:'missing archive'})};
  if(url.endsWith('/manifest'))return {ok:true,json:async()=>({...archived,...meta,schema:'scenesem.evidence.v1',kind:'recognition',rgb:result.image_sha256,result:'test',depth_encoding:'float32-le-linear-normalized-top-left'})};
  if(url.endsWith('/validity'))return {ok:true,arrayBuffer:async()=>new Uint8Array(4096).fill(1).buffer};return pairFetch(url,options);};
 context.Image=class{naturalWidth=64;naturalHeight=64;async decode(){}};
 get('supportCheck').onclick();assert.equal(get('pairLoad').disabled,false);assert.match(get('pairSourceInfo').textContent,/不能补造/);await get('pairLoad').onclick();assert.equal(get('pairView').hidden,false);assert.match(get('pairInfo').textContent,/身份仍未知/);get('pairMask').value='0';get('pairMask').onchange();assert.match(get('pairMaskInfo').textContent,/当前视角/);
 get('pairOverlay').value='none';get('pairOverlay').onchange();pairReject=true;await get('pairLoad').onclick();assert.equal(get('pairView').hidden,true);assert.match(get('pairInfo').textContent,/missing archive/);pairReject=false;
 let release;pairDelay=new Promise(r=>release=r);const closing=get('pairLoad').onclick();get('supportClose').onclick();release();await closing;assert.equal(get('pairView').hidden,true);assert.equal(get('supportDialog').open,false);
 get('supportCheck').onclick();pairDelay=new Promise(r=>release=r);const stale=get('pairLoad').onclick();run('boxes[1].category="plant"');release();await stale;assert.equal(get('pairView').hidden,true);assert.match(get('pairInfo').textContent,/已变化/);get('supportClose').onclick();
 get('supportCheck').onclick();pairDelay=new Promise(r=>release=r);const switched=get('pairLoad').onclick();get('supportPair').onchange();release();await switched;assert.equal(get('pairView').hidden,true);get('supportClose').onclick();
 assert(pairRequests.every(x=>x.method==='GET'));assert.equal(versions.length,supportBaseline);assert.equal(local.get('scenesem-project-draft-v1:'+meta.scene_id),supportDraft);context.fetch=pairFetch;context.Image=pairImage;run('delete boxes[0].geometry;delete boxes[1].geometry;boxes[1].category=regionsFixture[1].category');
 context.ObjectUI.hasPending=()=>true;get('supportCheck').onclick();assert.match(get('projectStatus').textContent,/未处理/);context.ObjectUI.hasPending=()=>false;
 // Identity preview checkpoints first, then changes object ownership only after a successful CAS save.
 run('boxes[0].geometry={type:"surface-voxels",origin:[0,0,0],cell:.04,cells:Array.from({length:10},(_,i)=>[i,0,0])};boxes[3].geometry=structuredClone(boxes[0].geometry);for(const i of [0,3]){boxes[i].min=[0,0,0];boxes[i].max=[.4,.04,.04]}');
 const identityStart=versions.length,identityDraft=local.get('scenesem-project-draft-v1:'+meta.scene_id),identityBoxes=run('JSON.stringify(boxes)');
 const openIdentity=()=>{get('supportCheck').onclick();get('identityDirection').value='a';get('identityPreview').onclick();};
 openIdentity();assert.equal(get('identityDialog').open,true);assert.match(get('identityRegions').textContent,/region-1/);assert.match(get('identityRegions').textContent,/region-4/);assert.equal(versions.length,identityStart);get('identityCancel').onclick();assert.equal(local.get('scenesem-project-draft-v1:'+meta.scene_id),identityDraft);
 openIdentity();await get('identityConfirm').onclick();assert.match(get('identityError').textContent,/依据/);assert.equal(versions.length,identityStart);get('identityReason').value='synthetic grouping, not identity truth';run('boxes[3].category="plant"');await get('identityConfirm').onclick();assert.match(get('identityError').textContent,/过期/);assert.equal(versions.length,identityStart);get('identityCancel').onclick();run('boxes[3].category=regionsFixture[0].category');
 openIdentity();get('identityReason').value='synthetic';rejectSave=true;await get('identityConfirm').onclick();assert.equal(versions.length,identityStart);assert.equal(get('identityConfirm').disabled,true);rejectSave=false;get('identityCancel').onclick();
 // Second-write failure leaves the successfully saved checkpoint selected, not a half-associated draft.
 const identityFetch=context.fetch;let identityWrites=0;context.fetch=async(url,options)=>{if(url==='/project/save'&&++identityWrites===2)return {ok:false,json:async()=>({error:'synthetic second write conflict'})};return identityFetch(url,options);};
 openIdentity();get('identityReason').value='synthetic';await get('identityConfirm').onclick();assert.equal(versions.length,identityStart+1);assert.equal(versions.at(-1).objects.length,versions[identityStart-1].objects.length);assert.match(get('identityError').textContent,/检查点/);assert.equal(get('identityConfirm').disabled,true);assert.equal(run('JSON.stringify(boxes)'),identityBoxes);context.fetch=identityFetch;get('identityCancel').onclick();
 openIdentity();get('identityReason').value='synthetic explicit association';await get('identityConfirm').onclick();assert.equal(versions.length,identityStart+3);assert.equal(get('identityDialog').open,false);assert.equal(versions.at(-1).objects.length,versions.at(-2).objects.length-1);assert.equal(run('JSON.stringify(boxes)'),identityBoxes);const checkpoint=versions.at(-2),associated=versions.at(-1);assert.deepEqual(associated.annotations,checkpoint.annotations);assert(associated.objects.some(o=>o.region_ids.includes('region-1')&&o.region_ids.includes('region-4')));assert(associated.objects.every(o=>o.task_qualification===false));
 await get('identityConfirm').onclick();assert.equal(versions.length,identityStart+3,'duplicate confirm does not save again');
 // The existing version restore flow recovers exact pre-association ownership; the associated revision remains immutable.
 await get('projectLoad').onclick();get('projectLoadVersion').value=String(checkpoint.revision);await get('projectLoadConfirm').onclick();await get('projectSave').onclick();assert.deepEqual(versions.at(-1).objects,checkpoint.objects);assert.deepEqual(versions.at(-1).annotations,checkpoint.annotations);assert.equal(associated.objects.length,checkpoint.objects.length-1);
 // D4 staged edits: batch membership, no pre-confirm writes, CAS failure, version/draft restore.
 const spatialStart=versions.length,spatialGeometry=run('JSON.stringify(boxes)');
 const prepareSpatial=async()=>{await get('spatialManage').onclick();assert.equal(get('spatialDialog').open,true);get('zoneName').value='Synthetic floor';get('zoneKind').value='floor';get('zoneParent').value='';get('zoneNotes').value='contract fixture';get('zoneApply').onclick();const parent=get('zoneEdit').value;get('zoneEdit').value='';get('zoneEdit').onchange();get('zoneName').value='Synthetic room';get('zoneParent').value=parent;get('zoneKind').value='room';get('zoneApply').onclick();const room=get('zoneEdit').value;get('zoneObjects').selectedOptions=versions.at(-1).objects.slice(0,2).map(o=>({value:o.object_id}));get('membershipZone').value=room;get('membershipNotes').value='synthetic only';get('membershipApply').onclick();get('spatialPreview').onclick();assert.equal(get('spatialReview').hidden,false);};
 await prepareSpatial();assert.equal(versions.length,spatialStart);get('spatialCancel').onclick();assert.equal(versions.at(-1).spatial_context,undefined);
 await prepareSpatial();const spatialFetch=context.fetch;let spatialWrites=0;context.fetch=async(url,options)=>{if(url==='/project/save'&&++spatialWrites===2)return {ok:false,json:async()=>({error:'synthetic conflict'})};return spatialFetch(url,options);};await get('spatialConfirm').onclick();assert.equal(versions.length,spatialStart+1);assert.equal(versions.at(-1).spatial_context,undefined);assert.equal(get('spatialConfirm').disabled,true);assert.match(get('spatialError').textContent,/检查点/);context.fetch=spatialFetch;get('spatialCancel').onclick();
 await prepareSpatial();await get('spatialConfirm').onclick();assert.equal(versions.length,spatialStart+3);assert.equal(get('spatialDialog').open,false);const spatialSaved=structuredClone(versions.at(-1));assert.equal(spatialSaved.spatial_context.zones.length,2);assert.equal(spatialSaved.spatial_context.memberships.length,2);assert.equal(run('JSON.stringify(boxes)'),spatialGeometry);assert.deepEqual(spatialSaved.objects,versions.at(-2).objects);
 await get('projectSave').onclick();assert.deepEqual(versions.at(-1).spatial_context,spatialSaved.spatial_context);assert.deepEqual(JSON.parse(local.get('scenesem-project-draft-v1:'+meta.scene_id)).spatial_context,spatialSaved.spatial_context);
 await get('projectLoad').onclick();get('projectLoadVersion').value=String(spatialStart);await get('projectLoadConfirm').onclick();await get('projectSave').onclick();assert.equal(versions.at(-1).spatial_context,undefined);await get('projectLoad').onclick();get('projectLoadVersion').value=String(spatialSaved.revision);await get('projectLoadConfirm').onclick();await get('projectSave').onclick();assert.deepEqual(versions.at(-1).spatial_context,spatialSaved.spatial_context);
 // Bounds and proposals remain working-copy edits until checkpoint + explicit save.
 const boundaryStart=versions.length,boundaryDraft=local.get('scenesem-project-draft-v1:'+meta.scene_id);
 await get('spatialManage').onclick();get('zoneName').value='Synthetic bounds';get('zoneKind').value='room';get('zoneParent').value='';get('zoneBoundEnabled').checked=true;get('zoneMin').value='-1, -1, -1';get('zoneMax').value='2, 2, 2';get('zoneMin').oninput();
 get('zoneShow').onclick();assert.equal(get('spatialDialog').open,false);assert.equal(get('returnZoneEditor').hidden,false);
 let strokes=0;const pen={save(){},restore(){},setLineDash(){},beginPath(){},moveTo(){},lineTo(){},stroke(){strokes++}};
 context.ProjectUI.drawZoneBoundary(pen,p=>({x:p[0],y:p[1],z:1}));assert.equal(strokes,12);get('returnZoneEditor').onclick();assert.equal(get('spatialDialog').open,true);context.ProjectUI.drawZoneBoundary(pen,p=>({x:p[0],y:p[1],z:1}));assert.equal(strokes,12);
 get('zoneSuggest').onclick();assert.match(get('zoneSuggestInfo').textContent,/先将表单/);get('zoneApply').onclick();get('zoneSuggest').onclick();assert(get('zoneSuggestions').children.length>0);
 get('zoneSuggestions').selectedOptions=get('zoneSuggestions').children.map(x=>({value:x.value}));get('zoneMin').oninput();get('zoneAccept').onclick();assert.match(get('zoneSuggestInfo').textContent,/先将表单/);get('zoneApply').onclick();
 const previousCategory=run('boxes[3].category');run('boxes[3].category="plant"');get('zoneAccept').onclick();assert.match(get('zoneSuggestInfo').textContent,/过期/);run('boxes[3].category='+JSON.stringify(previousCategory));
 get('zoneSuggest').onclick();get('zoneSuggestions').selectedOptions=get('zoneSuggestions').children.map(x=>({value:x.value}));get('zoneAccept').onclick();assert.match(get('zoneSuggestInfo').textContent,/尚未保存/);assert.equal(versions.length,boundaryStart);assert.equal(local.get('scenesem-project-draft-v1:'+meta.scene_id),boundaryDraft);
 get('spatialPreview').onclick();assert.equal(get('spatialReview').hidden,false);assert.match(get('spatialDiff').textContent,/范围 -1,-1,-1 → 2,2,2/);await get('spatialConfirm').onclick();assert.equal(versions.length,boundaryStart+2);
 const boundedSaved=versions.at(-1);assert.equal(boundedSaved.spatial_context.schema,'scenesem.spatial-context.v2');assert(boundedSaved.spatial_context.memberships.some(m=>m.source==='automatic_candidate'&&m.verification==='unverified'));assert.deepEqual(boundedSaved.objects,versions.at(-2).objects);assert.equal(run('JSON.stringify(boxes)'),spatialGeometry);
 await get('spatialManage').onclick();get('zoneName').value='Cancelled boundary';get('zoneBoundEnabled').checked=true;get('zoneMin').value='0,0,0';get('zoneMax').value='1,1,1';get('zoneApply').onclick();get('spatialCancel').onclick();assert.equal(versions.length,boundaryStart+2);assert.deepEqual(JSON.parse(local.get('scenesem-project-draft-v1:'+meta.scene_id)).spatial_context,boundedSaved.spatial_context);
 // Existing automatic memberships: recompute -> diff -> working copy -> explicit checkpoint/save.
 const recheckStart=versions.length,recheckStored=local.get('scenesem-project-draft-v1:'+meta.scene_id);
 const prepareRecheck=async()=>{
  await get('spatialManage').onclick();get('zoneRecheck').onclick();assert.match(get('zoneRecheckInfo').textContent,/仍支持原区/);assert(get('zoneRecheckRows').children.every(x=>x.disabled));
  get('zoneEdit').value=boundedSaved.spatial_context.zones.find(z=>z.boundary).zone_id;get('zoneEdit').onchange();get('zoneMin').value='10,10,10';get('zoneMax').value='12,12,12';get('zoneApply').onclick();
  get('zoneEdit').value='';get('zoneEdit').onchange();get('zoneName').value='New synthetic zone';get('zoneBoundEnabled').checked=true;get('zoneMin').value='-1,-1,-1';get('zoneMax').value='2,2,2';get('zoneApply').onclick();
  get('zoneRecheck').onclick();assert.match(get('zoneRecheckInfo').textContent,/建议改区/);get('zoneRecheckRows').selectedOptions=get('zoneRecheckRows').children.filter(x=>!x.disabled).map(x=>({value:x.value}));
 };
 await prepareRecheck();assert.equal(versions.length,recheckStart);assert.equal(local.get('scenesem-project-draft-v1:'+meta.scene_id),recheckStored);
 get('zoneMin').oninput();get('zoneRecheckAccept').onclick();assert.match(get('zoneRecheckInfo').textContent,/先将表单/);get('zoneApply').onclick();get('zoneRecheck').onclick();get('zoneRecheckRows').selectedOptions=get('zoneRecheckRows').children.filter(x=>!x.disabled).map(x=>({value:x.value}));
 const savedCategory=run('boxes[3].category');run('boxes[3].category="plant"');get('zoneRecheckAccept').onclick();assert.match(get('zoneRecheckInfo').textContent,/过期/);run('boxes[3].category='+JSON.stringify(savedCategory));
 get('zoneRecheckAccept').onclick();assert.match(get('zoneRecheckInfo').textContent,/尚未保存/);assert.equal(versions.length,recheckStart);assert.equal(local.get('scenesem-project-draft-v1:'+meta.scene_id),recheckStored);get('spatialCancel').onclick();
 await prepareRecheck();get('zoneRecheckAccept').onclick();get('spatialPreview').onclick();assert.match(get('spatialDiff').textContent,/Synthetic bounds → New synthetic zone/);await get('spatialConfirm').onclick();assert.equal(versions.length,recheckStart+2);
 const rechecked=versions.at(-1),recheckCheckpoint=versions.at(-2);assert.deepEqual(recheckCheckpoint.spatial_context,boundedSaved.spatial_context);assert.deepEqual(rechecked.objects,recheckCheckpoint.objects);assert.deepEqual(rechecked.annotations,recheckCheckpoint.annotations);
 for(const m of recheckCheckpoint.spatial_context.memberships.filter(m=>m.source==='manual'))assert.deepEqual(rechecked.spatial_context.memberships.find(x=>x.object_id===m.object_id),m);
 assert(rechecked.spatial_context.memberships.some(m=>m.source==='automatic_candidate'&&m.notes.includes('recheck')&&m.verification==='unverified'));
 await get('spatialConfirm').onclick();assert.equal(versions.length,recheckStart+2);
 await get('projectLoad').onclick();get('projectLoadVersion').value=String(recheckCheckpoint.revision);await get('projectLoadConfirm').onclick();await get('projectSave').onclick();assert.deepEqual(versions.at(-1).spatial_context,boundedSaved.spatial_context);
 // Business claims: no write before confirmation, provenance constraints, checkpoint/restore.
 const businessStart=versions.length,businessLocal=local.get('scenesem-project-draft-v1:'+meta.scene_id);
 const prepareBusiness=async()=>{await get('businessManage').onclick();assert.equal(get('businessDialog').open,true);get('businessSubject').value='object|'+versions.at(-1).objects[0].object_id;get('businessName').value='服务类型';get('businessBasis').value='simulation';get('businessValue').value='Synthetic only';get('businessSource').value='Test specification v1';get('businessFrom').value='2026-09-01';get('businessUntil').value='2026-09-30';get('businessApply').onclick();get('businessPreview').onclick();assert.equal(get('businessReview').hidden,false);};
 await prepareBusiness();assert.equal(versions.length,businessStart);assert.equal(local.get('scenesem-project-draft-v1:'+meta.scene_id),businessLocal);get('businessCancel').onclick();
 await prepareBusiness();get('businessSource').value='changed after preview';await get('businessConfirm').onclick();assert.match(get('businessError').textContent,/过期/);assert.equal(versions.length,businessStart);get('businessCancel').onclick();
 await prepareBusiness();const businessFetch=context.fetch;let businessWrites=0;context.fetch=async(url,options)=>{if(url==='/project/save'&&++businessWrites===2)return {ok:false,json:async()=>({error:'synthetic business save conflict'})};return businessFetch(url,options);};await get('businessConfirm').onclick();assert.equal(versions.length,businessStart+1);assert.equal(versions.at(-1).business_context,undefined);assert.equal(get('businessConfirm').disabled,true);context.fetch=businessFetch;get('businessCancel').onclick();
 await prepareBusiness();await get('businessConfirm').onclick();assert.equal(versions.length,businessStart+3);const businessSaved=structuredClone(versions.at(-1));assert.equal(businessSaved.business_context.attributes[0].verification,'unverified');assert.deepEqual(businessSaved.annotations,versions.at(-2).annotations);assert.deepEqual(businessSaved.objects,versions.at(-2).objects);assert.deepEqual(businessSaved.spatial_context,versions.at(-2).spatial_context);
 await get('businessConfirm').onclick();assert.equal(versions.length,businessStart+3);await get('projectSave').onclick();assert.deepEqual(versions.at(-1).business_context,businessSaved.business_context);assert.deepEqual(JSON.parse(local.get('scenesem-project-draft-v1:'+meta.scene_id)).business_context,businessSaved.business_context);
 await get('businessManage').onclick();get('businessName').value='Unsaved';get('businessName').oninput();get('businessEdit').value=businessSaved.business_context.attributes[0].attribute_id;get('businessEdit').onchange();assert.match(get('businessError').textContent,/未丢弃/);get('businessCancel').onclick();
 await get('projectLoad').onclick();get('projectLoadVersion').value=String(businessStart);await get('projectLoadConfirm').onclick();await get('projectSave').onclick();assert.equal(versions.at(-1).business_context,undefined);
 await get('projectLoad').onclick();get('projectLoadVersion').value=String(businessSaved.revision);await get('projectLoadConfirm').onclick();await get('projectSave').onclick();assert.deepEqual(versions.at(-1).business_context,businessSaved.business_context);
 // Date checks are read-only and stale results cannot navigate over form edits.
 const dateStart=versions.length,dateLocal=local.get('scenesem-project-draft-v1:'+meta.scene_id);
 await get('businessManage').onclick();get('businessCheckDate').value='2026-10-01';get('businessCheckRun').onclick();assert.match(get('businessCheckInfo').textContent,/已过期 1/);assert.equal(get('businessCheckRows').children.length,1);
 get('businessCheckEdit').onclick();assert.equal(get('businessEdit').value,businessSaved.business_context.attributes[0].attribute_id);assert.equal(get('businessCheckEdit').disabled,true);
 get('businessCheckDate').value='2026-09-30';get('businessCheckRun').onclick();assert.equal(get('businessCheckRows').children.length,0);get('businessCheckFilter').value='all';get('businessCheckFilter').onchange();assert.equal(get('businessCheckRows').children.length,1);assert.match(get('businessCheckDetail').textContent,/非认证/);
 get('businessName').value='Unsubmitted edit';get('businessName').oninput();assert.equal(get('businessCheckEdit').disabled,true);get('businessCheckRun').onclick();assert.match(get('businessCheckInfo').textContent,/先将填写/);assert.equal(get('businessName').value,'Unsubmitted edit');get('businessCancel').onclick();
 await get('businessManage').onclick();get('businessCheckDate').value='2026-09-14';get('businessCheckRun').onclick();get('businessCheckDate').value='2026-10-01';get('businessCheckFilter').onchange();assert.match(get('businessCheckInfo').textContent,/过期/);
 get('businessCheckRun').onclick();const dateCategory=run('boxes[0].category');run('boxes[0].category="plant"');get('businessCheckEdit').onclick();assert.match(get('businessCheckInfo').textContent,/过期/);run('boxes[0].category='+JSON.stringify(dateCategory));get('businessCancel').onclick();
 assert.equal(versions.length,dateStart);assert.equal(local.get('scenesem-project-draft-v1:'+meta.scene_id),dateLocal);
 // Local material upload is explicit; archive and claim save are separate operations.
 const materialStart=versions.length,materialFetch=context.fetch,materialMap=new Map();let materialPosts=0,materialDelay=null,materialRelease,materialReject=false,checkState='integrity_verified';
 context.fetch=async(url,options)=>{
  if(!url.startsWith('/project/material/'))return materialFetch(url,options);
  const data=JSON.parse(options.body);
  if(url.endsWith('/archive')){materialPosts++;if(materialDelay)await materialDelay;if(materialReject)throw Error('synthetic upload response lost');materialMap.set(data.snapshot.sha256,data.base64);}
  return {ok:true,json:async()=>({snapshot:data.snapshot,status:url.endsWith('/archive')?'integrity_verified':checkState,content_verified:false})};
 };
 const materialBytes=Uint8Array.from([83,121,110,116,104]),materialHash=crypto.createHash('sha256').update(materialBytes).digest('hex');
 const materialFile={name:'synthetic.txt',size:materialBytes.length,arrayBuffer:async()=>materialBytes.buffer};
 const openMaterial=async()=>{await get('businessManage').onclick();get('businessEdit').value=businessSaved.business_context.attributes[0].attribute_id;get('businessEdit').onchange();get('materialFile').files=[materialFile];get('materialFile').onchange();};
 await openMaterial();await get('materialArchive').onclick();assert.equal(materialPosts,0);assert.match(get('materialInfo').textContent,/授权/);
 get('businessApply').onclick();assert.match(get('businessError').textContent,/归档已选材料/);
 get('materialConsent').checked=true;await get('materialArchive').onclick();assert.equal(materialMap.size,1);assert.match(get('materialInfo').textContent,/声明尚未保存/);assert.equal(versions.length,materialStart);
 // The DOM stub does not implement the native file-input value reset.
 get('materialFile').files=[];checkState='missing';await get('materialCheck').onclick();assert.match(get('materialInfo').textContent,/缺少快照/);
 checkState='invalid';await get('materialCheck').onclick();assert.match(get('materialInfo').textContent,/损坏/);
 checkState='integrity_verified';await get('materialCheck').onclick();assert.match(get('materialInfo').textContent,/内容未核验/);
 get('businessCancel').onclick();assert.equal(versions.length,materialStart);assert.equal(materialMap.size,1,'cancel does not delete archived material');assert.equal(versions.at(-1).business_context.schema,'scenesem.business-context.v1');
 await openMaterial();get('materialConsent').checked=true;await get('materialArchive').onclick();get('materialFile').files=[];get('businessApply').onclick();get('businessPreview').onclick();assert.match(get('businessDiff').textContent,/source_snapshot/);await get('businessConfirm').onclick();
 const materialSaved=structuredClone(versions.at(-1));assert.equal(versions.length,materialStart+2);assert.equal(materialSaved.business_context.schema,'scenesem.business-context.v2');assert.equal(materialSaved.business_context.attributes[0].source_snapshot.sha256,materialHash);assert.equal(materialSaved.business_context.attributes[0].verification,'unverified');assert.deepEqual(materialSaved.objects,versions.at(-2).objects);assert.equal(materialMap.size,1);
 await get('businessManage').onclick();get('businessEdit').value=materialSaved.business_context.attributes[0].attribute_id;get('businessEdit').onchange();get('materialDetach').onclick();get('businessApply').onclick();get('businessPreview').onclick();get('businessCancel').onclick();assert.deepEqual(versions.at(-1),materialSaved);
 // Changed forms after a request prevent implicit binding; a lost reply never auto-retries.
 await openMaterial();get('materialConsent').checked=true;materialDelay=new Promise(r=>materialRelease=r);const pendingMaterial=get('materialArchive').onclick();
 for(let i=0;i<100&&materialPosts<3;i++)await new Promise(r=>setTimeout(r,1));assert.equal(materialPosts,3);get('businessName').value='Changed during archive';materialRelease();await pendingMaterial;assert.match(get('materialInfo').textContent,/未自动绑定/);get('businessCancel').onclick();materialDelay=null;
 await openMaterial();get('materialConsent').checked=true;materialReject=true;const postsBeforeFailure=materialPosts;await get('materialArchive').onclick();assert.equal(materialPosts,postsBeforeFailure+1);assert.match(get('materialInfo').textContent,/文件可能已归档/);get('businessCancel').onclick();materialReject=false;context.fetch=materialFetch;get('materialFile').files=[];
 assert.equal(versions.length,materialStart+2);
 // Dataset material inclusion defaults to none, requires consent, and never writes a revision.
 const deliveryStart=versions.length,deliveryFetch=context.fetch,deliveryCreate=context.document.createElement,deliveryLinks=[];let deliveryPosts=0,deliveryFailure=false,deliveryDelay=null,deliveryRelease;
 context.URL={createObjectURL:()=> 'blob:synthetic-dataset',revokeObjectURL(){}};
 context.document.createElement=tag=>{const e=deliveryCreate(tag);if(tag==='a')deliveryLinks.push(e);return e;};
 context.fetch=async(url,options)=>{if(url!=='/project/dataset-selected')return deliveryFetch(url,options);deliveryPosts++;const d=JSON.parse(options.body);assert.deepEqual(d.material_hashes,[materialHash]);assert.equal(d.confirm_material_export,true);if(deliveryDelay)await deliveryDelay;return deliveryFailure?{ok:false,json:async()=>({error:'synthetic missing material'})}:{ok:true,blob:async()=>({type:'application/zip'})};};
 const selectDelivery=()=>{get('datasetExport').onclick();assert.equal(get('datasetWithMaterials').disabled,true);assert.equal(get('datasetMaterialConsent').checked,false);const checkbox=get('datasetMaterials').children[0].children[0];assert.equal(checkbox.checked,false);checkbox.checked=true;checkbox.onchange();return checkbox;};
 selectDelivery();await get('datasetWithMaterials').onclick();assert.equal(deliveryPosts,0);assert.match(get('datasetError').textContent,/确认导出用途/);
 get('datasetMaterialConsent').checked=true;await get('datasetWithMaterials').onclick();assert.equal(deliveryLinks.at(-1).href,'blob:synthetic-dataset');assert.equal(get('datasetDialog').open,false);assert.equal(versions.length,deliveryStart);
 const deliveryCount=deliveryLinks.length;selectDelivery();get('datasetMaterialConsent').checked=true;deliveryFailure=true;await get('datasetWithMaterials').onclick();assert.match(get('datasetError').textContent,/missing material/);assert.equal(get('datasetDialog').open,true);assert.equal(deliveryLinks.length,deliveryCount);deliveryFailure=false;get('datasetCancel').onclick();
 const checkDelivery=selectDelivery();get('datasetMaterialConsent').checked=true;deliveryDelay=new Promise(r=>deliveryRelease=r);const pendingDelivery=get('datasetWithMaterials').onclick();await tick();assert.equal(deliveryPosts,3);checkDelivery.checked=false;checkDelivery.onchange();deliveryRelease();await pendingDelivery;assert.match(get('datasetError').textContent,/预览已变化/);assert.equal(deliveryLinks.length,deliveryCount);get('datasetCancel').onclick();deliveryDelay=null;
 assert.equal(versions.length,deliveryStart);context.fetch=deliveryFetch;context.document.createElement=deliveryCreate;
 // Project backup shares the file selector but targets the restore-capable bundle endpoint.
 const backupBaseline=versions.length,backupFetch=context.fetch,backupCreate=context.document.createElement;let backupRequests=0,backupLinks=[];
 context.document.createElement=tag=>{const e=backupCreate(tag);if(tag==='a')backupLinks.push(e);return e;};
 context.fetch=async(url,options)=>{if(url==='/project/bundle-selected'){backupRequests++;const d=JSON.parse(options.body);assert.deepEqual(d.material_hashes,[materialHash]);assert.equal(d.confirm_material_export,true);return {ok:true,blob:async()=>({type:'application/zip'})};}return backupFetch(url,options);};
 get('projectExport').onclick();assert.match(get('exportKindTitle').textContent,/项目备份/);get('datasetDownload').onclick();assert.equal(backupLinks.at(-1).href,'/project/export/'+versions.length);
 get('projectExport').onclick();const backupChoice=get('datasetMaterials').children[0].children[0];backupChoice.checked=true;backupChoice.onchange();await get('datasetWithMaterials').onclick();assert.equal(backupRequests,0);get('datasetMaterialConsent').checked=true;await get('datasetWithMaterials').onclick();assert.equal(backupRequests,1);assert.match(backupLinks.at(-1).download,/SceneSem-project/);assert.equal(versions.length,backupBaseline);
 context.document.createElement=backupCreate;context.fetch=backupFetch;
 // Imported materials need a separate local-storage confirmation; cancellation never commits.
 let importMaterialWrites=0;context.fetch=async(url,options)=>{if(url==='/project/import/preview'){const r=await backupFetch(url,options),d=await r.json();return {ok:true,json:async()=>({...d,material_restore_count:1,material_capacity:{additional_bytes:5},material_availability:[{sha256:materialHash,names:['synthetic.txt'],status:'in_package'}],unbound_material_attribute_ids:[]})};}if(url==='/project/import/commit'){importMaterialWrites++;assert.equal(JSON.parse(options.body).confirm_material_restore,true);return {ok:false,json:async()=>({error:'synthetic material import failure'})};}return backupFetch(url,options);};
 get('projectImportFile').files=[{name:'materials.zip',size:1000}];await get('projectImportFile').onchange();assert.equal(get('projectImportMaterialPermission').hidden,false);assert.equal(get('projectImportMaterialConsent').checked,false);assert.match(get('projectImportMaterials').textContent,/包内已验证/);
 await get('projectImportConfirm').onclick();assert.equal(importMaterialWrites,0);assert.match(get('projectStatus').textContent,/确认包内材料/);get('projectImportMaterialConsent').checked=true;await get('projectImportConfirm').onclick();assert.equal(importMaterialWrites,1);assert.equal(versions.length,backupBaseline);assert.match(get('projectImportError').textContent,/synthetic material import failure/);get('projectImportCancel').onclick();context.fetch=backupFetch;
 // Normal Save auto-registers unowned regions without a separate intake or review action.
 await assert.rejects(context.ProjectUI.saveWorkbench(),/当前窗口/);for(const n of nodes.values())n.open=false;
 const autoSaveStart=versions.length;run('boxes.push({...structuredClone(boxes[0]),id:"preexisting-unowned",sequence:998});persist();snapshot();boxes.push({...structuredClone(boxes[0]),id:"auto-save-new",sequence:999});persist();');const autoSaveDraft=local.get('scenesem-project-draft-v1:'+meta.scene_id);
 rejectSave=true;await assert.rejects(context.ProjectUI.saveWorkbench());assert.equal(versions.length,autoSaveStart);assert.equal(local.get('scenesem-project-draft-v1:'+meta.scene_id),autoSaveDraft);rejectSave=false;
 await context.ProjectUI.saveWorkbench();assert.equal(versions.length,autoSaveStart+1);const autoRegistered=versions.at(-1).objects.find(o=>o.region_ids.includes('auto-save-new'));assert(autoRegistered);assert.equal(autoRegistered.task_qualification,false);assert.equal(autoRegistered.reviews.length,0);
 const autoObjectCount=versions.at(-1).objects.length;await context.ProjectUI.saveWorkbench();assert.equal(versions.at(-1).objects.length,autoObjectCount);
 // R1 paired undo: actual label handler, project metadata and synthetic storage/API.
 const draftKey='scenesem-project-draft-v1:'+meta.scene_id;
 const stableAutoId=versions.at(-1).objects.find(o=>o.region_ids.includes('preexisting-unowned')).object_id;
 const beforeUndo=versions.length,stackSize=run('undoStack.length');
 for(const n of nodes.values())n.open=false;
 const write=context.localStorage.setItem;
 const localBefore=[local.get('synthetic-test-only'),local.get(draftKey)],regionsBefore=run('JSON.stringify(boxes)');
 context.localStorage.setItem=(k,v)=>{if(k===draftKey)throw Error('synthetic quota');write(k,v);};
 run('undoRegionEdit()');assert.match(get('projectStatus').textContent,/操作未应用/);
 assert.equal(run('JSON.stringify(boxes)'),regionsBefore);assert.equal(run('undoStack.length'),stackSize);
 assert.deepEqual([local.get('synthetic-test-only'),local.get(draftKey)],localBefore,'failed second write rolls back first');
 context.localStorage.setItem=write;
 context.ObjectUI.hasPending=()=>true;run('undoRegionEdit()');assert.equal(run('undoStack.length'),stackSize);context.ObjectUI.hasPending=()=>false;
 get('reviewNotes').value='keep unsent text';run('undoRegionEdit()');assert.equal(run('undoStack.length'),stackSize);get('objectReviewReset').onclick();
 run('undoRegionEdit()');assert.equal(run('undoStack.length'),stackSize-1);assert.equal(versions.length,beforeUndo,'undo never rewinds or writes disk HEAD');
 assert(!run('boxes.some(b=>b.id==="auto-save-new")'));let restored=JSON.parse(local.get(draftKey));
 assert(!restored.objects.some(o=>o.region_ids.includes('auto-save-new')));
 assert.equal(restored.objects.find(o=>o.region_ids.includes('preexisting-unowned')).object_id,stableAutoId,'unrelated registration ID survives undo');
 assert.deepEqual(restored.spatial_context,versions.at(-1).spatial_context??null);
 assert.deepEqual(restored.business_context,versions.at(-1).business_context??null);
 await context.ProjectUI.saveWorkbench();assert.equal(versions.length,beforeUndo+1);
 assert(!versions.at(-1).objects.some(o=>o.region_ids.includes('auto-save-new')));
 assert(versions[beforeUndo-1].objects.some(o=>o.region_ids.includes('auto-save-new')),'saved history stays intact');
 // Deletion, ordinary save, undo, save: exact stable instance identity is restored.
 run('snapshot();boxes=boxes.filter(b=>b.id!=="preexisting-unowned");persist();');await context.ProjectUI.saveWorkbench();
 run('undoRegionEdit()');await context.ProjectUI.saveWorkbench();
 assert(versions.at(-1).annotations.boxes.some(b=>b.id==='preexisting-unowned'));
 assert.equal(versions.at(-1).objects.find(o=>o.region_ids.includes('preexisting-unowned')).object_id,stableAutoId);
 // One-click initial insertion creates IDs immediately, before any server save.
 const instant=JSON.parse(run('JSON.stringify({...boxes[0],id:"instant-new",sequence:997})'));
 const instantBefore=run('JSON.stringify(boxes)'),instantDraft=local.get(draftKey),instantStack=run('undoStack.length'),instantVersions=versions.length;
 const pairBefore=local.get('synthetic-test-only');
 context.localStorage.setItem=(k,v)=>{if(k===draftKey)throw Error('synthetic quota during initial');write(k,v);};
 assert.throws(()=>context.ProjectUI.applyInitialBatch([instant]),/操作未应用/);
 assert.equal(run('JSON.stringify(boxes)'),instantBefore);assert.equal(run('undoStack.length'),instantStack);assert.equal(local.get(draftKey),instantDraft);assert.equal(local.get('synthetic-test-only'),pairBefore);
 context.localStorage.setItem=write;
 context.ProjectUI.applyInitialBatch([instant]);assert.equal(versions.length,instantVersions);
 const instantObjects=JSON.parse(local.get(draftKey)).objects;
 const instantObject=instantObjects.find(o=>o.region_ids.includes('instant-new'));assert(instantObject);assert.equal(instantObject.task_qualification,false);assert.deepEqual(instantObject.reviews,[]);
 assert.deepEqual(instantObjects.filter(o=>o.object_id!==instantObject.object_id),JSON.parse(instantDraft).objects,'existing archives unchanged');
 assert.equal(run('undoStack.length'),instantStack+1);assert.throws(()=>context.ProjectUI.applyInitialBatch([instant]),/ID 重复/);
 await context.ProjectUI.saveWorkbench();assert.equal(versions.at(-1).objects.find(o=>o.region_ids.includes('instant-new')).object_id,instantObject.object_id);
 run('undoRegionEdit()');assert.equal(run('JSON.stringify(boxes)'),instantBefore);assert(!JSON.parse(local.get(draftKey)).objects.some(o=>o.object_id===instantObject.object_id));
 await context.ProjectUI.saveWorkbench();assert(!versions.at(-1).objects.some(o=>o.object_id===instantObject.object_id));
 console.log('PASS R1 immediate registration: paired write failure, pre-save identity, stable save ID and whole-batch undo. Synthetic only.');
 // Later metadata edits must never be silently discarded by an older label undo.
 run('snapshot();boxes[0].sequence+=100;persist();');const epochStack=run('undoStack.length');
 get('objectAlias').value='new metadata must survive';get('objectAlias').onchange();
 run('undoRegionEdit()');assert.equal(run('undoStack.length'),epochStack);assert.match(get('projectStatus').textContent,/另有档案/);
 assert(JSON.parse(local.get(draftKey)).objects.some(o=>o.alias==='new metadata must survive'));
 console.log('PASS R1 paired undo: save/undo/resave, stable IDs, delete restore, storage rollback, pending input and metadata boundary. Synthetic only.');
 local.set(draftKey,'{broken draft, preserve it');vm.runInContext(fs.readFileSync(__dirname+'/project-ui.js','utf8'),context);context.ProjectUI.regionsChanged();assert.equal(local.get(draftKey),'{broken draft, preserve it');get('batchRegister').onclick();assert.match(get('projectStatus').textContent,/异常对象草稿/);
 console.log('PASS S1: identity/linking, review queue and unsaved-form protection, review preview/cancel/stale/conflict/duplicate guards, evidence, restore, ZIP and corrupt-draft protection in Node VM. No browser/GPU/human quality acceptance.');
})().catch(e=>{console.error(e);process.exitCode=1;});

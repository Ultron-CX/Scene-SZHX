// Synthetic UI orchestration -> real Python store -> ZIP -> independent reader.
// No browser, live server, user data, model inference or GPU access.
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path'),vm=require('node:vm'),cp=require('node:child_process'),crypto=require('node:crypto');
const B=require('./boxes.js'),P=require('./project-core.js'),O=require('./object-core.js'),I=require('./initial-core.js');
require('./completion-core.js');const Cleanup=require('./cleanup-core.js');
const root=path.resolve(__dirname,'..'),tmp=path.join(root,'.tmp');fs.mkdirSync(tmp,{recursive:true});
const isolated=fs.mkdtempSync(path.join(tmp,'r1-flow-'));
function bridge(request){const r=cp.spawnSync(path.join(root,'.venv/Scripts/python.exe'),['-X','utf8',path.join(root,'scripts/r1_flow_bridge.py'),isolated],{input:JSON.stringify(request),encoding:'utf8',windowsHide:true,maxBuffer:4*1024*1024});assert.equal(r.status,0,r.stderr);const result=JSON.parse(r.stdout);if(!result.ok)throw Error(result.error);return result.data;}
const meta={scene_id:'synthetic-r1-flow-not-a-bank',source_hashes:{test:'a'.repeat(64)},coordinate_frame:'TEST',T_lcc_from_las:[]};
const box=(id,category,x)=>({id,category,sequence:1,status:'proposed',min:[x,0,0],max:[x+.5,.5,1],provenance:{method:'synthetic_fixture'}});
const structures=['wall','floor','ceiling'].map((c,i)=>box('structure-'+c,c,i*3));
const chair=box('chair','chair',9);chair.geometry={type:'surface-voxels',origin:[0,0,0],cell:.1,cells:[[90,0,0],[91,0,0],[91,1,0],[92,1,1],[92,1,2]]};Object.assign(chair,O.bounds(chair.geometry));
const candidates=[chair,box('screen','screen',12),box('table','table',15)];
const nodes=new Map(),local=new Map(),downloads=[];
function get(id){if(!nodes.has(id)){const el=element();el.id=id;}return nodes.get(id);}
function element(){const e={value:'',checked:false,hidden:false,disabled:false,open:false,children:[],style:{setProperty(){}},classList:{toggle(){}},dataset:{},textContent:'',append(...x){this.children.push(...x);},prepend(...x){this.children.unshift(...x);},add(x){this.children.push(x);},replaceChildren(...x){this.children=x;},insertBefore(x){this.children.push(x);},setAttribute(){},addEventListener(){},getContext:()=>({}),showModal(){this.open=true;},close(){this.open=false;},remove(){},click(){if(this.href){downloads.push({url:this.href,data:bridge({action:'export',revision:Number(this.href.split('/').at(-1))})});return;}return this.onclick?.();},set innerHTML(value){for(const m of value.matchAll(/id="([^"]+)"/g))get(m[1]);}};Object.defineProperty(e,'id',{get(){return this._id||'';},set(id){this._id=id;nodes.set(id,this);}});return e;}
const document={getElementById:get,createElement:element,querySelector:s=>s==='dialog[open]'?[...nodes.values()].find(n=>n.open)||null:get(s),querySelectorAll:()=>[...nodes.values()],body:element(),addEventListener(){},activeElement:{tagName:'BODY',blur(){}}};
const ctx={console,structuredClone,crypto:crypto.webcrypto,BoxCore:B,ProjectCore:P,ObjectCore:O,document,Option:function(text,value){return {textContent:text,value};},localStorage:{getItem:k=>local.get(k)??null,setItem:(k,v)=>local.set(k,v),removeItem:k=>local.delete(k)},setTimeout,clearTimeout,window:{addEventListener(){}},confirm:()=>true,pc:{Vec3:class{}},CompletionUI:{active:()=>false},CleanupUI:{active:()=>false},ObjectUI:{busy:()=>false,hasPending:()=>false},metaFixture:meta,
 fetch:async(url,options)=>{try{assert.equal(url,'/project/save');return {ok:true,json:async()=>bridge({action:'save',data:JSON.parse(options.body)})};}catch(e){return {ok:false,json:async()=>({error:e.message})};}}};
vm.createContext(ctx);const run=s=>vm.runInContext(s,ctx);
run(fs.readFileSync(__dirname+'/scene-app.js','utf8').replace(/init\(\);\s*$/,''));
run("meta=metaFixture;boxes=[];storageKey='synthetic-r1-regions';filterScene=()=>{};resetDraft=()=>{selected=null;draft=null;anchors=[];draftChanged=false;};");
run(fs.readFileSync(__dirname+'/project-ui.js','utf8'));run(fs.readFileSync(__dirname+'/workbench-ux.js','utf8'));
const plain=x=>JSON.parse(JSON.stringify(x));
(async()=>{
 const plan=I.plan([],structures,candidates,B,O,()=>crypto.randomUUID(),{id:'synthetic-batch',status:'completed_views'});assert.equal(plan.added.length,6);
 ctx.ProjectUI.applyInitialBatch(plan.added);assert.equal(ctx.ProjectUI.saveState().revision,0);assert(ctx.ProjectUI.saveState().changed);
 assert.equal(JSON.parse(local.get('scenesem-project-draft-v1:'+meta.scene_id)).objects.length,6);
 await get('exportMain').click();assert.equal(get('datasetDialog').open,true,get('message').textContent);assert.equal(ctx.ProjectUI.saveState().revision,1);assert.equal(run('dirty'),false);
 get('datasetDownload').click();assert.equal(downloads.length,1);const original=downloads[0].data;
 assert.equal(original.regions.length,6);assert.equal(original.objects.length,6);assert.deepEqual(original.quality.unregistered_region_ids,[]);assert.equal(original.quality.accuracy,null);
 assert(original.objects.every(o=>o.task_qualification===false&&!o.reviews.length));
 const target=plain(run('boxes.find(b=>b.category==="chair")')),identity=original.objects.find(o=>o.region_ids.includes(target.id)).object_id;
 assert(B.match(original.regions,9.05,.05,.05,'chair').visible);assert(!B.contains(target,9.05,.15,.05),'a hole is not filled by its box');
 ctx.cleaned=Cleanup.clean(target,new Set(['90,0,0']),['synthetic spill example']);
 run('snapshot();boxes=boxes.map(b=>b.id===cleaned.id?cleaned:b);persist();');ctx.WorkbenchUX.refresh();assert.match(get('saveState').textContent,/未保存/);
 await get('exportMain').click();get('datasetDownload').click();const edited=downloads[1].data;
 assert.equal(edited.regions.find(b=>b.id===target.id).geometry.cells.length,4);
 assert.equal(edited.objects.find(o=>o.region_ids.includes(target.id)).object_id,identity);
 assert.deepEqual(edited.regions.filter(b=>b.id!==target.id),original.regions.filter(b=>b.id!==target.id));
 run('undoRegionEdit()');await get('exportMain').click();get('datasetDownload').click();
 assert.deepEqual(downloads[2].data.regions,original.regions);assert.deepEqual(downloads[2].data.objects.map(o=>o.object_id),original.objects.map(o=>o.object_id));
 assert.deepEqual(bridge({action:'load',revision:1}).document.annotations.boxes,original.regions,'immutable original version preserved');
 // Real store CAS conflict: no stale export dialog or download after a rejected save.
 const last=bridge({action:'load'});bridge({action:'save',data:{expected_revision:last.head,annotations:last.document.annotations,objects:last.document.objects,spatial_context:null,business_context:null,reason:'synthetic_other_writer'}});
 await get('exportMain').click();assert.equal(get('datasetDialog').open,false);assert.equal(downloads.length,3);assert.match(get('message').textContent,/新版本/);assert.equal(ctx.ProjectUI.saveState().revision,3);
 console.log('PASS R1 UI-to-real-store flow: six synthetic classes, immediate IDs, support edit, save/export/read, undo, immutable history and CAS rejection. No browser/GPU/inference. Artifacts: '+isolated);
})().catch(e=>{console.error(e);process.exitCode=1;});

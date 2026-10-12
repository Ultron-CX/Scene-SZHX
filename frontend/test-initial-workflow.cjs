const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm'),crypto=require('node:crypto');
const B=require('./boxes.js'),C=require('./object-core.js'),I=require('./initial-core.js');
const region=(id,x,category='chair')=>({id,category,sequence:1,status:'proposed',min:[x,0,0],max:[x+.4,.4,.8],provenance:{model_score:.9,model_label:category,observations:[]}});
const old=region('human',0),wall=region('structure',10,'wall'),item=region('candidate',20),batch={id:'batch'};
const before=JSON.stringify([old]);let n=0;
const p=I.plan([old],[wall],[region('overlap',0),item,{...item,id:'bad',min:[NaN,0,0]}],B,C,()=>String(++n),batch);
assert.equal(JSON.stringify([old]),before);assert.equal(p.added.length,2);assert.equal(p.skipped.overlap,1);assert.equal(p.skipped.invalid,1);
assert(p.added.every(b=>b.status==='proposed'&&b.provenance.initial_batch.id==='batch'));
assert.equal(I.plan([old,...p.added],[wall],[item],B,C,()=>String(++n),batch).added.length,0);
function harness(mode){
 const nodes=new Map();function get(id){if(!nodes.has(id))nodes.set(id,{checked:false,value:'',hidden:false,disabled:false,children:[],append(...a){this.children.push(...a);},replaceChildren(...a){this.children=a;},setAttribute(){}});return nodes.get(id);}
 let calls=0,restored=0,saves=0;
 const meta={scene_id:'test',coordinate_frame:'world',source_hashes:{a:'hash'}},undo=[];
 const ctx={console,structuredClone,crypto:crypto.webcrypto,BoxCore:B,B,ObjectCore:{...C,fromMask:()=>[structuredClone(item)],merge:(list,next)=>list.push(...next)},InitialCore:I,
  ProjectCore:{framePayload:()=>({test:true})},ProjectUI:{busy:()=>false,hasReviewDraft:()=>false,applyInitialBatch:added=>{if(mode==='apply-fail')throw Error('synthetic paired draft failure');undo.push(structuredClone(ctx.boxes));ctx.boxes.push(...structuredClone(added));saves++;}},CompletionUI:{active:()=>false},CleanupUI:{active:()=>false},
  $:get,cat:c=>[c,c],clone:structuredClone,association:null,meta,gsEntity:{},loading:false,draftChanged:false,anchors:[],proposals:[wall],boxes:[structuredClone(old)],pressed:{clear(){}},
  target:{clone(){return this;}},dist:1,yaw:0,pitch:0,view:'gs',cam:{setPosition(){},lookAt(){}},pc:{Vec3:class{}},updateCameraClip(){},cameraUpdate(){restored++;},setView(){},filterScene(){},topZ:()=>1,
  snapshot:()=>undo.push(structuredClone(ctx.boxes)),persist:()=>{saves++;return true;},resetDraft(){},msg:s=>ctx.message=s,
  document:{querySelectorAll:()=>[...nodes.values()],createElement:()=>({append(){},setAttribute(){}})},window:{addEventListener(){}},setTimeout:cb=>{cb();return 1;},clearTimeout(){},frameFixture:{width:64,height:64,image:'synthetic'},
  fetch:async(url,opt)=>{let data;if(url==='/semantic/status')data={ready:true,instance_ready:true};else if(url==='/semantic/tour')data={scene_id:meta.scene_id,source_hashes:meta.source_hashes,positions:[[1,2,3]]};else{calls++;assert(JSON.parse(opt.body).frame,'one-click always archives new observation evidence');if(mode==='stop'&&calls===2)get('initialStop').onclick();if(mode==='fail'&&calls===2)throw Error('synthetic failure');if(mode==='stale'&&calls===1)ctx.boxes.push(region('external',30));data={width:64,height:64,scene_id:meta.scene_id,components:[{}]};}return {ok:true,json:async()=>data};}
 };
 vm.createContext(ctx);const code=fs.readFileSync(__dirname+'/object-ui.js','utf8').replace('globalThis.ObjectUI={','capture=async()=>frameFixture;globalThis.ObjectUI={');vm.runInContext(code,ctx);
 return {ctx,get,undo,stats:()=>({calls,restored,saves})};
}
(async()=>{
 for(const mode of ['complete','stop','fail']){const h=harness(mode);await h.ctx.ObjectUI.runInitial();assert.equal(h.ctx.boxes.length,3);assert.deepEqual(h.ctx.boxes[0],old);assert.equal(h.undo.length,1);assert.equal(h.ctx.ObjectUI.hasPending(),false);assert.equal(h.get('initialStop').hidden,true);assert.equal(h.stats().restored,1);assert.equal(h.get('initialAnnotate').disabled,false);assert.equal(h.ctx.boxes[1].provenance.initial_batch.status,{complete:'completed_views',stop:'stopped',fail:'partial_failure'}[mode],h.ctx.message);if(mode==='complete'){await h.ctx.ObjectUI.runInitial();assert.equal(h.ctx.boxes.length,3);assert.equal(h.undo.length,1);}h.ctx.boxes=h.undo.pop();assert.deepEqual(h.ctx.boxes,[old]);}
 const zero=harness('complete');await zero.ctx.ObjectUI.runInitial();assert.match(zero.ctx.message,/新增批次已关联实例，可撤销/);const zeroSaves=zero.stats().saves;await zero.ctx.ObjectUI.runInitial();assert.match(zero.ctx.message,/新增 0 条标注/);assert.match(zero.ctx.message,/跳过原因：异常 0、重叠 4、重复 1、数量上限 0/);assert.match(zero.ctx.message,/无需保存或撤销本批/);assert(!zero.ctx.message.includes('新增批次已关联实例'));assert.equal(zero.stats().saves,zeroSaves);assert.equal(zero.undo.length,1);
 const failedApply=harness('apply-fail');await failedApply.ctx.ObjectUI.runInitial();assert.equal(failedApply.undo.length,0);assert.deepEqual(failedApply.ctx.boxes,[old]);assert(failedApply.ctx.ObjectUI.hasPending());assert.match(failedApply.ctx.message,/synthetic paired draft failure/);
 const missingModule=harness('complete');delete missingModule.ctx.ProjectUI.applyInitialBatch;await missingModule.ctx.ObjectUI.runInitial();assert.equal(missingModule.undo.length,0);assert.deepEqual(missingModule.ctx.boxes,[old]);assert(missingModule.ctx.ObjectUI.hasPending());assert.match(missingModule.ctx.message,/实例关联模块未就绪/);
 const stale=harness('stale');await stale.ctx.ObjectUI.runInitial();assert.equal(stale.undo.length,0);assert.equal(stale.stats().saves,0);assert(stale.ctx.ObjectUI.hasPending());assert.match(stale.ctx.message,/工作台在运行期间变化/);
 const blocked=harness('complete');blocked.ctx.draftChanged=true;await blocked.ctx.ObjectUI.runInitial();assert.equal(blocked.stats().calls,0);assert.equal(blocked.ctx.boxes.length,1);
 console.log('PASS R1: append-only batch, overlap/invalid protection, rerun dedup, initial run/stop/partial failure/stale input, camera restore, evidence request and one-batch undo. Synthetic VM, no GPU acceptance.');
})().catch(e=>{console.error(e);process.exitCode=1;});

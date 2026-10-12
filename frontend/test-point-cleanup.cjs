const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
const B=require('./boxes.js'),O=require('./object-core.js'),I=require('./instance-tools.js'),C=require('./completion-core.js'),K=require('./cleanup-core.js'),D=require('./support-depth.js');
const width=256,height=256,near=.1,far=10,z=2,A=(far+near)/(near-far),P=2*far*near/(near-far);
const frame={width,height,near,far,projection:[1,0,0,0,0,1,0,0,0,0,A,-1,0,0,P,0],inverse:[1,0,0,0,0,1,0,0,0,0,0,1/P,0,0,-1,A/P],depth:new Float32Array(width*height).fill((z-near)/(far-near)),camera_position:[0,0,0]};
for(const p of [[0,0,-2],[.3,.3,-2],[0,0,-1]])assert(D.visibleAt(p,.04,frame));
for(const p of [[0,0,-3],[0,0,1],[5,0,-2],[0,0,-.01],[0,0,-11]])assert(!D.visibleAt(p,.04,frame));
assert(!D.visibleAt([0,0,-2],.04,{...frame,depth:new Float32Array(width*height).fill(NaN)}));
assert(!D.visibleAt([0,0,-2],.04,{...frame,depth:new Float32Array(width*height).fill(1)}));
const region=(id,category,cells)=>{const geometry={type:'surface-voxels',cell:.04,origin:[0,0,0],cells};return {id,category,sequence:1,status:'reviewed',geometry,...O.bounds(geometry)};};
const anchor=region('anchor','chair',[[0,0,-50],[1,0,-50],[2,0,-50]]),added=region('added','chair',[[10,0,-50]]),table=region('table','table',[[10,0,-50]]);
const mask=(x0,x1)=>({runs:Array.from({length:8},(_,i)=>[122+i,x0,x1])}),positive={matched_masks:[mask(124,165)]},negative={matched_masks:[mask(124,140)]};
let audit=C.auditSupport(anchor,[added],[frame,{...frame,camera_position:[.3,0,0]},{...frame,camera_position:[.6,0,0]}],[positive,negative,negative],[anchor]);
assert.equal(audit.flagged.length,1);assert(audit.flagged[0].is_new&&audit.flagged[0].multiview_disagreement);assert.equal(audit.flagged[0].positive_views,1);assert.equal(audit.flagged[0].negative_views,2);
audit=C.auditSupport(anchor,[added],[frame,{...frame,camera_position:[.1,0,0]},frame],[positive,negative,negative],[anchor]);assert.equal(audit.flagged.length,0,'Same location cannot create two independent negative views');
audit=C.auditSupport(anchor,[added],[frame,{...frame,depth:new Float32Array(width*height).fill(0),camera_position:[.3,0,0]},{...frame,depth:new Float32Array(width*height).fill(0),camera_position:[.6,0,0]}],[positive,negative,negative],[anchor]);assert.equal(audit.flagged.length,0,'Occlusion must not be negative evidence');
audit=C.auditSupport(anchor,[added],[frame],[positive],[anchor,table]);assert.equal(audit.flagged[0].conflict.category,'table');assert(audit.flagged[0].is_new);
const broad={id:'broad',category:'floor',sequence:1,status:'reviewed',min:[-1,-1,-3],max:[1,1,0]};assert.equal(C.labelConflicts(anchor,[broad]).size,0,'An AABB is not surface membership');
const floor={...B.fromContour([[-1,-1,-2.1],[1,-1,-2.1],[1,1,-2.1],[-1,1,-2.1]],'xy',.2),id:'floor',category:'floor',sequence:1,status:'reviewed'};assert.equal(C.labelConflicts(anchor,[floor]).size,3);
assert.deepEqual(K.brushKeys([{key:'a',x:5,y:0},{key:'b',x:15,y:3},{key:'c',x:8,y:8}],[0,0],[20,0],4),['a','b']);
const base=region('chair-024','chair',[[0,0,-50],[6,0,-50],[10,20,-75],[15,0,-50]]);base.provenance={evidence_state:'targeted_instance_refinement'};
const original=JSON.stringify(base),clean=K.clean(base,new Set(['0,0,-50']));assert.equal(JSON.stringify(base),original);assert.equal(clean.geometry.cells.length,3);assert.equal(clean.status,'draft');assert.equal(I.displayParts(clean).length,1);assert.deepEqual(K.restore(clean),base);
assert.throws(()=>K.clean(base,new Set(['not-owned'])));assert.throws(()=>K.clean(base,new Set(base.geometry.cells.map(c=>c.join(',')))));assert(!K.canRestore({...clean,category:'table'}));
const moved=structuredClone(clean);moved.geometry.origin[0]++;assert(!K.canRestore(moved));
const metaFixture={scene_id:'test',coordinate_frame:'world',source_hashes:{x:'a'},T_lcc_from_las:[]};assert.deepEqual(B.validate(B.save([clean],metaFixture),metaFixture),[clean]);

// Real UI handlers in a Node VM. No browser automation or GPU acceptance.
const nodes=new Map(),element=()=>({value:'',checked:false,hidden:false,disabled:false,style:{setProperty(){}},clientWidth:256,clientHeight:256,children:[],getContext:()=>({}),append(...c){this.children.push(...c);},replaceChildren(){this.children=[];},setAttribute(){},addEventListener(){},classList:{toggle(){}},getBoundingClientRect:()=>({left:0,top:0}),setPointerCapture(){}}),get=id=>{if(!nodes.has(id))nodes.set(id,element());return nodes.get(id);};
let depthReady=true,depthStamp='one';
const context={BoxCore:B,ObjectCore:O,InstanceTools:I,CompletionCore:C,CleanupCore:K,structuredClone,console,setTimeout,clearTimeout,confirm:()=>true,crypto:require('node:crypto').webcrypto,document:{getElementById:get,createElement:element,addEventListener(){},activeElement:{tagName:'BODY',blur(){}}},window:{addEventListener(){}},base,metaFixture,frame};
context.ObjectUI={busy:()=>false,refresh(){},visible:()=>false};context.CompletionUI={active:()=>false,refresh(){}};context.SupportDepth={ready:()=>depthReady,stamp:()=>depthStamp,checker:force=>{assert.equal(force,true);return (p,c)=>D.visibleAt(p,c,frame);}};
vm.createContext(context);vm.runInContext(fs.readFileSync(__dirname+'/scene-app.js','utf8').replace(/init\(\);\s*$/,''),context);const run=s=>vm.runInContext(s,context),plain=x=>JSON.parse(JSON.stringify(x));
run("meta=metaFixture;boxes=[clone(base),{...clone(base),id:'other',sequence:2}];cam={};filterScene=()=>{};persist=()=>true;fitBox=()=>{};project=p=>{const q=CompletionCore.project(p,frame);return {...q,z:1};};view='gs';bind();selectRegion(boxes[0],false)");get('cleanupRadius').value='12';
vm.runInContext(fs.readFileSync(__dirname+'/cleanup-ui.js','utf8'),context);const U=context.CleanupUI;
U.start();assert(U.active());assert.deepEqual(plain(run('boxes[0]')),base);U.setBrush('erase');
const position=c=>{const q=C.project(c.map((v,i)=>(v+.5)*.04),frame);return [q.x,q.y];},first=position(base.geometry.cells[0]),hidden=position(base.geometry.cells[2]);
depthReady=false;U.begin(first);U.end();assert.match(get('cleanupStats').textContent,/待删 0/);depthReady=true;
U.begin(hidden);U.end();assert.match(get('cleanupStats').textContent,/待删 0/,'No through-surface painting');
get('supportXray').checked=true;U.begin(first);U.end();assert.match(get('cleanupStats').textContent,/待删 1/);assert.deepEqual(plain(run('boxes[0]')),base);
U.setBrush('restore');U.begin(first);U.end();assert.match(get('cleanupStats').textContent,/待删 0/);get('undo').onclick();assert.match(get('cleanupStats').textContent,/待删 1/);
run('cancelSelection()');assert(U.active()&&!U.painting(),'Esc exits brush while retaining preview');context.confirm=()=>false;U.apply();assert.equal(run('undoStack.length'),0);assert(U.active());context.confirm=()=>true;U.apply();assert(!U.active());assert.equal(run('boxes[0].geometry.cells.length'),3);assert.deepEqual(plain(run('boxes[1].geometry')),base.geometry);
U.restore();assert.deepEqual(plain(run('boxes[0]')),base);get('undo').onclick();assert.equal(run('boxes[0].geometry.cells.length'),3);get('undo').onclick();assert.deepEqual(plain(run('boxes[0]')),base);
run('selectRegion(boxes[0],false)');U.start();U.setBrush('erase');U.begin(first);U.end();run('boxes=[{...boxes[0],status:"draft"},boxes[1]]');const changed=plain(run('boxes'));U.apply();assert.deepEqual(plain(run('boxes')),changed);U.cancel();get('cleanupDiscard').onclick();
run('selectRegion(boxes[0],false)');U.start();U.setBrush('erase');run('view="points"');U.begin(first);U.end();assert.match(get('cleanupStats').textContent,/待删 0/);run('view="gs"');U.cancel();
// Exercise actual pointer handlers: painting cannot rotate or pick another label.
U.start();U.setBrush('erase');run('lookAround=()=>{throw Error("brush rotated camera")};selectSurface=()=>{throw Error("brush selected another object")};');const overlay=get('overlay');overlay.onpointerdown({button:0,pointerId:1,clientX:first[0],clientY:first[1]});overlay.onpointermove({clientX:first[0]+1,clientY:first[1]});overlay.onpointerup({button:0});assert.match(get('cleanupStats').textContent,/待删 1/);get('undo').onclick();assert.match(get('cleanupStats').textContent,/待删 0/);U.cancel();
// Category suggestions remain unchecked until an explicit mark action.
// Cancel uses an inline decision, never a blocking native dialog; no label writes.
run('selectRegion(boxes[0],false)');U.start();U.setBrush('erase');U.begin(first);U.end();const beforeCancel=plain(run('boxes')),beforeUndo=run('undoStack.length');
context.confirm=()=>{throw Error('Cancel must not open a native dialog');};
assert.equal(U.cancel(),false);assert(U.active()&&!U.painting());assert.equal(get('cleanupCancelConfirm').hidden,false);assert(get('cleanupApply').disabled);
U.apply();assert.deepEqual(plain(run('boxes')),beforeCancel);assert.equal(run('undoStack.length'),beforeUndo);
get('cleanupKeep').onclick();assert(U.active());assert.equal(get('cleanupCancelConfirm').hidden,true);assert.match(get('cleanupStats').textContent,/待删 1/);
U.cancel();get('cleanupDiscard').onclick();assert(!U.active());assert.deepEqual(plain(run('boxes')),beforeCancel);assert.equal(run('undoStack.length'),beforeUndo);
U.start();assert.match(get('cleanupStats').textContent,/待删 0/);assert.equal(U.cancel(),true);assert(!U.active());context.confirm=()=>true;
run("boxes.push({...clone(base),id:'desk',category:'table',sequence:1});selectRegion(boxes[0],false)");U.start();assert.equal(get('cleanupConflicts').children[0].children[0].checked,false);U.mark();assert.match(get('cleanupStats').textContent,/待删 0/);const check=get('cleanupConflicts').children[0].children[0];check.checked=true;check.onchange();U.mark();assert.match(get('cleanupStats').textContent,/待删 4/);U.apply();assert(U.active(),'Cannot remove entire region in cleanup');U.clear();U.cancel();

async function testDepthRuntime(){
 let clock=1000,resolve,reject,world=new Float32Array(16),asset={},calls=0;const matrix={data:frame.projection};
 class Mat4{mul2(){this.data=frame.projection;return this;}}
 class Picker{resize(){}prepare(){}get depthBuffer(){return {read:()=>{calls++;return new Promise((yes,no)=>{resolve=yes;reject=no;});}};}}
 const ctx={CompletionCore:C,ObjectCore:{decodeDepth:()=>frame.depth},pc:{Mat4,Picker},$ :get,globalThis:null,app:{scene:{},graphicsDevice:{isWebGL2:true}},cam:{getWorldTransform:()=>({data:world}),camera:{nearClip:near,farClip:far,camera:{projectionMatrix:matrix,viewMatrix:{}}}},canvas:{clientWidth:256,clientHeight:256},gsEntity:{},gsAsset:asset,loading:false,view:'gs',draft:base,ObjectUI:{busy:()=>false},performance:{now:()=>clock}};ctx.globalThis=ctx;vm.createContext(ctx);vm.runInContext(fs.readFileSync(__dirname+'/support-depth.js','utf8'),ctx);const S=ctx.SupportDepth,tick=()=>new Promise(r=>setImmediate(r));get('supportXray').checked=false;
 S.update();assert.equal(calls,1);world[12]=1;resolve(new Uint8Array());await tick();assert(!S.ready(),'Discard late depth after camera changed');clock+=200;S.update();resolve(new Uint8Array());await tick();assert(S.ready());assert(S.checker(true)([0,0,-2],.04));assert(!S.checker(true)([0,0,-3],.04));
 get('supportXray').checked=true;assert(S.checker()([0,0,-3],.04));assert(!S.checker(true)([0,0,-3],.04));S.invalidate();assert(!S.ready());clock+=200;S.update();ctx.gsAsset={};resolve(new Uint8Array());await tick();assert(!S.ready(),'Discard depth after GS replacement');clock+=200;S.update();reject(Error('GPU read failure'));await tick();assert(!S.ready());assert.match(get('supportDepthStatus').textContent,/失败/);clock+=1600;S.update();resolve(new Uint8Array());await tick();assert(S.ready());ctx.loading=true;assert(!S.ready());ctx.loading=false;ctx.view='points';assert(!S.ready());assert(!S.checker(true)([0,0,-2],.04));
}
testDepthRuntime().then(()=>console.log('PASS v13: all-new-cell cross-view audit, camera dedup/occlusion exclusion, category geometry conflicts, visible-only erase/restore brush and pointer routing, confirmation/cancel, exact membership/other-label preservation, restore/undo/export, stale target rejection, depth cache camera/asset/failure isolation. Node VM + synthetic geometry; no live browser/GPU or chair-024 accuracy claim.')).catch(e=>{console.error(e);process.exitCode=1;});

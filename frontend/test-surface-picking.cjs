const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm'),B=require('./boxes.js');
// Tilted laptop-like surface, multi-view points, and an additional off-plane point.
const surface=[[1,2,.8],[1.4,2,.9],[1.4,2.3,1.05],[1,2.3,.95]];
for(const points of [surface,[...surface,[1.2,2.15,1.2]],[[0,0,1],[1,0,1],[1,1,1],[0,1,1]]]){
 const region={...B.fromSurfacePoints(points),id:'surface',category:'device',sequence:1,status:'draft'};
 assert(B.validBox(region),'Auto-oriented envelope must be saveable');
 assert.deepEqual(region.picked_points,points,'Never flatten original anchors');
 for(const p of points)assert(B.contains(region,...p),'Envelope must contain each picked point '+p);
 const meta={scene_id:'test',coordinate_frame:'source',source_hashes:{mesh:'test'}};
 assert.deepEqual(B.validate(B.save([region],meta),meta),[region]);
}
assert.throws(()=>B.fromSurfacePoints([[0,0,0],[1,0,0],[2,0,0]]));
assert.throws(()=>B.fromSurfacePoints(surface,NaN));
const nodes=new Map(),element=()=>({value:'',checked:false,textContent:'',style:{},clientWidth:800,clientHeight:600,getContext:()=>({}),append(){},replaceChildren(){},setAttribute(){},addEventListener(){},classList:{toggle(){}},getBoundingClientRect:()=>({left:10,top:20})});
const get=id=>{if(!nodes.has(id))nodes.set(id,element());return nodes.get(id);};
get('shape').value='surface';get('clipZ').value='100';
const calls=[],results=[];
class Picker{constructor(app,w,h,depth){assert.equal(depth,true);calls.push(['new',w,h]);}resize(w,h){calls.push(['resize',w,h]);}prepare(camera,scene){calls.push(['prepare']);}async getWorldPointAsync(x,y){calls.push(['read',x,y]);return results.shift();}}
const context={BoxCore:B,structuredClone,console,pc:{Picker},document:{getElementById:get,createElement:element,addEventListener(){}},window:{addEventListener(){}},setTimeout,clearTimeout};
vm.createContext(context);vm.runInContext(fs.readFileSync(__dirname+'/scene-app.js','utf8').replace(/init\(\);\s*$/,''),context);
const run=s=>vm.runInContext(s,context),plain=x=>JSON.parse(JSON.stringify(x));
run("app={scene:{}};cam={camera:{},getWorldTransform:()=>({data:new Float32Array(16)})};gsEntity={};meta={};fields=()=>{};refreshList=()=>{};bind();mode='pick';hit=()=>{throw Error('Proxy mesh must not be used for GS surface picking');};");
(async()=>{
 for(const p of surface){results.push({x:p[0],y:p[1],z:p[2]});await run('pickAnchorAt(100,200)');}
 assert.deepEqual(plain(run('anchors')),surface,'Every click retains its own depth, not first-point plane');
 assert.equal(calls.filter(c=>c[0]==='prepare').length,4);
 results.push(null);await run('pickAnchorAt(100,200)');assert.equal(run('anchors.length'),4);assert.match(get('message').textContent,/没有有效/);
 // Exercise the real bound pointer handler (mocked depth read, no GPU/browser claim).
 results.push({x:1.2,y:2.15,z:1.2});run('drag={moved:0,button:0};overlay.onpointerup({clientX:110,clientY:220})');
 await new Promise(resolve=>setImmediate(resolve));assert.equal(run('anchors.length'),5);assert.deepEqual(calls.at(-1),['read',100,200]);
 get('thickness').value='.03';run('finishShape()');assert.equal(run('draft.picked_points.length'),5);
 // New object restores simple defaults even after editing a tall fixed-plane structure.
 get('shape').value='polygon';get('thickness').value='2.5';get('newBox').onclick();assert.equal(get('shape').value,'surface');assert.equal(get('thickness').value,'.03');assert.equal(get('planeOptions').hidden,true);
 // Pending read cannot add a point to a cancelled/new annotation.
 let resolveDepth;context.delayed=new Promise(resolve=>resolveDepth=resolve);run('surfacePicker.getWorldPointAsync=()=>delayed');
 const pending=run('pickAnchorAt(50,50)');run('resetDraft()');resolveDepth({x:1,y:2,z:3});await pending;assert.equal(run('anchors.length'),0);assert.equal(run('pickBusy'),false);
 // Point mode uses actual sampled point coordinates and shares the same anchors.
 run("view='points';mode='pick';hit=()=>[1,2,.7]");await run('pickAnchorAt(50,50)');assert.deepEqual(plain(run('anchors')),[[1,2,.7]]);
 // Right click deselects without deleting saved labels; drag must not deselect.
 context.confirm=()=>{throw Error('Clean selection must not ask for confirmation');};
 run("resetDraft();boxes=[{id:'wall',category:'wall',sequence:1}];selected='wall';draft=clone(boxes[0]);drag={moved:0,button:2};overlay.onpointerup({})");
 assert.equal(run('selected'),null);assert.equal(run('draft'),null);assert.equal(run('boxes.length'),1);
 run("selected='wall';draft=clone(boxes[0]);drag={moved:6,button:2};overlay.onpointerup({})");assert.equal(run('selected'),'wall');
 // Declining the unsaved-edit warning preserves all editing state.
 context.confirm=()=>false;
 run("draftChanged=true;anchors=[[1,2,3]];drag={moved:0,button:2};overlay.onpointerup({})");
 assert.equal(run('selected'),'wall');assert.equal(run('draftChanged'),true);assert.equal(run('anchors.length'),1);
 context.confirm=()=>true;run('cancelSelection()');assert.equal(run('selected'),null);assert.equal(run('anchors.length'),0);assert.equal(run('boxes.length'),1);
 // Sub-threshold right-button jitter does not pan; crossing threshold pans.
 run("let panCount=0;target={add(){panCount++;return this;}};cam.right={clone:()=>({mulScalar(){return this;}})};cam.up=cam.right;cameraUpdate=()=>{};drag={last:[100,100],button:2,moved:0};overlay.onpointermove({clientX:111,clientY:120})");
 assert.equal(run('panCount'),0);
 run('overlay.onpointermove({clientX:120,clientY:120})');assert.equal(run('panCount'),2);
 // Real left-pointer binding delegates to in-place look, not orbit cameraUpdate.
 run("let turns=[];lookAround=(x,y)=>turns.push([x,y]);cameraUpdate=()=>{throw Error('Left drag must not orbit the camera');};drag={last:[100,100],button:0,moved:0};overlay.onpointermove({clientX:111,clientY:120})");
 assert.equal(run('turns.length'),0,'Click jitter must not rotate');
 run('overlay.onpointermove({clientX:121,clientY:124})');assert.deepEqual(plain(run('turns')),[[10,4]]);
 console.log('PASS: auto-oriented envelope, exact per-click anchors, roundtrip, GS depth API wiring, null/no-proxy fallback, pointer handler, fresh defaults, async cancellation, shared point-mode path. GPU accuracy not tested.');
})().catch(e=>{console.error(e);process.exitCode=1;});

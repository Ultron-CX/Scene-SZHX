const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
const B=require('./boxes.js'),C=require('./object-core.js');
const fixture=JSON.parse(fs.readFileSync(__dirname+'/../artifacts/k2-instance-selection-v10/source-chair-096.json','utf8')).box;
const original=JSON.stringify(fixture),parts=C.splitCandidate(fixture);
assert.equal(parts.length,6);assert.equal(parts[0].geometry.cells.length,269);
assert(Math.abs(parts[0].max[0]-parts[0].min[0]-.64)<1e-8);
assert.equal(parts.reduce((n,p)=>n+p.geometry.cells.length,0),327);
assert.deepEqual(new Set(parts.flatMap(p=>p.geometry.cells.map(c=>c.join(',')))),new Set(fixture.geometry.cells.map(c=>c.join(','))));
assert.equal(JSON.stringify(fixture),original,'Inspection/splitting must not mutate the source');
assert(C.geometryWarnings(fixture).length);assert(!C.geometryWarnings(parts[0]).length);
for(const p of parts){assert(B.validBox(p));assert.equal(C.viewCount(p),0,'Do not attribute all parent view evidence to every split fragment');}
// One 2D mask containing two well-separated depths must become two local supports.
const near=.1,far=10,A=(far+near)/(near-far),P=2*far*near/(near-far),depth=new Float32Array(256*256).fill(NaN),runs=[];
for(let y=100;y<110;y++)for(const [x0,x1,z]of [[60,70,2],[180,190,6]]){runs.push([y,x0,x1]);for(let x=x0;x<x1;x++)depth[y*256+x]=(z-near)/(far-near);}
const projected=C.fromMask({category:'screen',label:'monitor',score:.8,instance_id:25,runs},{width:256,height:256,near,far,depth,pose_id:'one-pose',inverse:[1,0,0,0,0,1,0,0,0,0,0,1/P,0,0,-1,A/P]},{backend:'instance',cell_m:.04,model:'test',image_sha256:'image',threshold:.6});
assert.equal(projected.length,2);assert.notEqual(projected[0].provenance.observations[0].instance_id,projected[1].provenance.observations[0].instance_id);
assert(projected.every(b=>b.max[2]-b.min[2]<.1),'Do not retain a 4 m depth bridge');
const pending=[];C.merge(pending,projected);assert.equal(pending.length,2);

const nodes=new Map(),element=()=>({value:'',checked:false,hidden:false,style:{},clientWidth:800,clientHeight:600,children:[],getContext:()=>({}),append(...c){this.children.push(...c);},replaceChildren(){this.children=[];},setAttribute(){},addEventListener(){},classList:{toggle(){}},getBoundingClientRect:()=>({left:0,top:0}),setPointerCapture(){}});
const get=id=>{if(!nodes.has(id))nodes.set(id,element());return nodes.get(id);};
const context={BoxCore:B,ObjectCore:C,structuredClone,console,setTimeout,clearTimeout,confirm:()=>true,crypto:require('node:crypto').webcrypto,document:{getElementById:get,createElement:element,addEventListener(){},activeElement:{tagName:'BODY',blur(){}}},window:{addEventListener(){}}};
vm.createContext(context);vm.runInContext(fs.readFileSync(__dirname+'/scene-app.js','utf8').replace(/init\(\);\s*$/,''),context);
const run=s=>vm.runInContext(s,context),plain=x=>JSON.parse(JSON.stringify(x));
get('clipZ').value='100';
run("cam={getWorldTransform:()=>({data:new Float32Array(16)})};meta={};fields=()=>{};refreshList=()=>{};filterScene=()=>{};persist=()=>true;mode='browse';hit=()=>{throw Error('Coarse mesh must never select a GS label');};");
(async()=>{
 // Reproduce different label ordering: the old selector chose the first chair,
 // while coloring selected the later screen. The new selector shares color policy.
 run("boxes=[{id:'wrong-chair',category:'chair',status:'proposed',sequence:1,min:[0,0,0],max:[2,2,2]},{id:'screen',category:'screen',status:'proposed',sequence:1,min:[0,0,0],max:[2,2,2]}];renderedSurfaceHit=async()=>[1,1,1]");
 await run('selectSurface(100,100)');assert.equal(run('selected'),'screen');assert.equal(run('selected'),run("B.compile(boxes)(1,1,1,'all').box.id"));
 run("boxes[0].status='reviewed'");await run('selectSurface(100,100)');assert.equal(run('selected'),'wrong-chair','Human precedence is shared by coloring and selection');
 run("renderedSurfaceHit=async()=>{throw Error('No GS depth');}");await run('selectSurface(100,100)');assert.equal(run('selected'),'wrong-chair','Missing depth must not use coarse mesh or a rear object');
 run('renderedSurfaceHit=async()=>[9,9,9]');await run('selectSurface(100,100)');assert.equal(run('selected'),null);
 let resolve;context.delayed=new Promise(r=>resolve=r);run('renderedSurfaceHit=()=>delayed');const waiting=run('selectSurface(100,100)');run('cancelSelection()');resolve([1,1,1]);await waiting;assert.equal(run('selected'),null,'Right cancel invalidates a pending depth selection');
 context.delayed=new Promise(r=>resolve=r);const stale=run('selectSurface(100,100)');run("selectRegion(boxes[1],false)");resolve([1,1,1]);await stale;assert.equal(run('selected'),'screen','Late depth must not overwrite a newer list selection');
 run("bind();selected=null;mode='browse';labelTargets=[{id:'wrong-chair',x:0,y:0,w:200,h:200}];let surfaceClicks=0;selectSurface=()=>{surfaceClicks++;};drag={moved:0,button:0};overlay.onpointerup({clientX:100,clientY:100})");assert.equal(run('surfaceClicks'),1);assert.equal(run('selected'),null,'Floating label hitboxes must not steal surface clicks');

 // Exercise the real repair action against the actual exported chair, with undo.
 context.fixture=fixture;
 const human={id:'human',category:'chair',sequence:116,status:'reviewed',min:[40,40,0],max:[41,41,1]};context.human=human;
 run('boxes=[clone(fixture),clone(human)];selected=fixture.id;draft=clone(fixture);draftChanged=false;anchors=[];undoStack=[];');
 context.confirm=()=>false;run('splitCurrentCandidate()');assert.equal(run('boxes.length'),2);
 context.confirm=()=>true;run('splitCurrentCandidate()');assert.equal(run('boxes.length'),7);assert.equal(run('selected'),fixture.id);
 const after=plain(run('boxes'));assert.deepEqual(after.at(-1),human);assert.deepEqual(after.slice(0,6).map(b=>b.sequence),[96,117,118,119,120,121]);
 assert.equal(after.slice(0,6).reduce((n,b)=>n+b.geometry.cells.length,0),327);assert(after.every(B.validBox));
 assert.deepEqual(plain(run('undoStack[0]')),[fixture,human]);
 get('undo').onclick();assert.deepEqual(plain(run('boxes')),[fixture,human]);
 // Inspecting the geometry does not perform a repair or mutate saved boxes.
 run('draft=clone(fixture);candidateGeometryInfo()');assert.match(get('geometryNote').textContent,/6 个空间片段/);assert.equal(run('boxes.length'),2);
 // Exercise the actual overlay renderer with a deterministic projection (no GPU).
 run("let renderedLines=[],startPoint=null;Object.assign(pen,{clearRect(){},setLineDash(){},beginPath(){},moveTo(x,y){startPoint=[x,y];},lineTo(x,y){renderedLines.push([startPoint,[x,y]]);},stroke(){},measureText(){return {width:50};},fillRect(){},fillText(){}});project=p=>({x:p[0]*100,y:p[1]*100,z:1});drawOverlay()");
 assert.equal(run('renderedLines.length'),parts.length*12);
 assert(run('renderedLines.every(([a,b])=>Math.hypot(a[0]-b[0],a[1]-b[1])<100)'),'No rectangle edge may bridge the actual chair fragments meters apart');
 console.log('PASS: actual chair-096 six-part support conservation, bounds, source immutability, same GS-depth/color selection policy, no coarse fallback, no label interception, async cancellation, real confirm/split/IDs/human preservation/undo. Browser GPU not tested.');
})().catch(error=>{console.error(error);process.exitCode=1;});

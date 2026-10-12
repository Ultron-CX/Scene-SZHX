const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
const B=require('./boxes.js'),C=require('./object-core.js'),I=require('./instance-tools.js');
const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0),plain=x=>JSON.parse(JSON.stringify(x));
const meta={scene_id:'test',coordinate_frame:'world',source_hashes:{test:'test'},T_lcc_from_las:[]};
function box(id,sequence,cells,category='chair'){const geometry={type:'surface-voxels',origin:[0,0,0],cell:.04,cells};return {id,sequence,category,status:'proposed',...C.bounds(geometry),geometry,provenance:{human_modified:false,view_ids:['a'],observations:[{view_id:'a',instance_id:id}],qualification:false}};}
const yaw=.62,pitch=.3,u=[Math.cos(yaw),Math.sin(yaw),0],v=[-Math.sin(yaw)*Math.sin(pitch),Math.cos(yaw)*Math.sin(pitch),Math.cos(pitch)],points=[];
for(let x=-.6;x<=.6;x+=.015)for(let y=-.35;y<=.35;y+=.015)points.push([0,1,2].map(i=>2+u[i]*x+v[i]*y));
const g=C.geometry(points,.04),before=JSON.stringify(g),frame=I.orientedBounds(g),aabb=C.bounds(g),volume=aabb.max.reduce((s,v,i)=>s*(v-aabb.min[i]),1),ratio=frame.size.reduce((a,b)=>a*b,1)/volume;
assert(ratio<.4,`Tilted screen volume ratio ${ratio}`);
assert.equal(JSON.stringify(g),before);
for(let i=0;i<3;i++)for(let j=0;j<3;j++)assert(Math.abs(dot(frame.axes[i],frame.axes[j])-(i===j?1:0))<1e-7);
for(const cell of g.cells)for(let k=0;k<8;k++){const p=cell.map((v,i)=>(v+((k&(1<<i))?1:0))*g.cell+g.origin[i]);for(let j=0;j<3;j++)assert(Math.abs(dot(p.map((v,i)=>v-frame.center[i]),frame.axes[j]))<=frame.size[j]/2+1e-8,'OBB must cover every support cell corner, not crop evidence');}
const moved=I.orientedBounds({...g,origin:[10,-20,3]});for(let i=0;i<3;i++)assert(Math.abs(moved.center[i]-frame.center[i]-[10,-20,3][i])<1e-8);assert.deepEqual(moved.axes,frame.axes);
for(const cells of [[[0,0,0]],[[0,0,0],[0,0,1]],[[0,0,0],[1,1,1],[2,2,2]]]){const f=I.orientedBounds({type:'surface-voxels',origin:[0,0,0],cell:.04,cells});assert(f.corners.flat().every(Number.isFinite));assert(f.size.every(v=>v>=.04-1e-8));}
const seat=box('seat',24,[[0,0,0],[1,0,0],[0,1,0],[1,1,0],[0,0,1]]),back=box('back',25,[[0,0,20],[1,0,20],[0,1,20],[1,1,20],[0,0,21]]);back.status='reviewed';
const originals=JSON.stringify([seat,back]),merged=I.mergeRegions([seat,back]);
assert.equal(merged.status,'draft');assert.equal(merged.id,seat.id);assert.equal(merged.sequence,24);assert.equal(merged.geometry.cells.length,10);assert.equal(merged.provenance.qualification,false);
assert.equal(C.spatialParts(merged.geometry,1).length,2);assert.equal(I.displayParts(merged).length,1);assert.deepEqual(I.restoreSources(merged),[seat,back]);assert.equal(JSON.stringify([seat,back]),originals);
assert.deepEqual(B.validate(B.save([merged],meta),meta),[merged]);
const changed=structuredClone(merged);changed.geometry.origin[0]=1;assert(!I.canRestore(changed));assert.throws(()=>I.restoreSources(changed));
const added=structuredClone(merged);added.geometry.cells.push([99,99,99]);assert.throws(()=>I.restoreSources(added));
assert.throws(()=>I.mergeRegions([seat,{...back,category:'screen'}]));assert.throws(()=>I.mergeRegions([seat,{...back,geometry:{...back.geometry,cell:.08}}]));
assert.throws(()=>I.mergeRegions([seat,seat]));
const more=box('leg',26,[[0,0,-20],[1,0,-20],[0,1,-20],[1,1,-20],[0,0,-21]]),again=I.mergeRegions([merged,more]);assert.equal(again.provenance.association_sources.length,3);assert.equal(I.restoreSources(again).length,3);
const actual=JSON.parse(fs.readFileSync(__dirname+'/../artifacts/k2-instance-selection-v10/source-chair-096.json','utf8')).box;
assert.equal(I.displayParts(actual).length,6);assert(I.displayParts(actual).map(I.orientedBounds).every(f=>Math.max(...f.size)<1),'Old cross-desk chair stays separated without explicit user association');

const nodes=new Map(),element=()=>({value:'',checked:false,hidden:false,disabled:false,style:{setProperty(){}},clientWidth:800,clientHeight:600,children:[],getContext:()=>({}),append(...c){this.children.push(...c);},replaceChildren(){this.children=[];},setAttribute(){},addEventListener(){},classList:{toggle(){}},getBoundingClientRect:()=>({left:0,top:0}),setPointerCapture(){}});
const get=id=>{if(!nodes.has(id))nodes.set(id,element());return nodes.get(id);};
const ctx={BoxCore:B,ObjectCore:C,InstanceTools:I,structuredClone,console,setTimeout,clearTimeout,confirm:()=>true,crypto:require('node:crypto').webcrypto,document:{getElementById:get,createElement:element,addEventListener(){},activeElement:{tagName:'BODY',blur(){}}},window:{addEventListener(){}},metaFixture:meta,seat,back};
vm.createContext(ctx);vm.runInContext(fs.readFileSync(__dirname+'/scene-app.js','utf8').replace(/init\(\);\s*$/,''),ctx);const run=s=>vm.runInContext(s,ctx);
run("meta=metaFixture;boxes=clone([seat,back]);filterScene=()=>{};persist=()=>true;fitBox=()=>{};cam={getWorldTransform:()=>({data:new Float32Array(16)})};bind();selectRegion(boxes[0],false);startAssociation();");
assert.equal(run('association.ids.size'),1);assert(get('commit').disabled);assert.equal(get('associationPanel').hidden,false);
run("toggleAssociation({...back,category:'screen'})");assert.equal(run('association.ids.size'),1);
run('chooseRegion(boxes[1],false)');assert.equal(run('association.ids.size'),2);
ctx.confirm=()=>false;run('confirmAssociation()');assert.equal(run('boxes.length'),2);assert.equal(run('undoStack.length'),0);
ctx.confirm=()=>true;run('confirmAssociation()');assert.equal(run('boxes.length'),1);assert.equal(run('association'),null);assert.equal(run('boxes[0].status'),'draft');assert.equal(get('commit').disabled,false);
assert.equal(run('exportAnnotations().observed_envelopes.regions[0].frames.length'),1);assert.deepEqual(plain(run('exportAnnotations().boxes[0].provenance.association_sources')),[seat,back]);
run('restoreAssociation()');assert.deepEqual(plain(run('boxes')),[seat,back]);get('undo').onclick();assert.equal(run('boxes.length'),1);
run('selectRegion(boxes[0],false);splitCurrentCandidate()');assert.equal(run('boxes.length'),2);assert.equal(run('boxes.reduce((n,b)=>n+b.geometry.cells.length,0)'),10);
run('startAssociation();cancelSelection()');assert.equal(run('association'),null);assert.equal(run('boxes.length'),2);
// ID/sequence collisions after import must not overwrite unrelated annotations.
run('boxes=clone([seat,back]);selectRegion(boxes[0],false);startAssociation();toggleAssociation(boxes[1]);confirmAssociation();boxes.push({...clone(back),min:[4,4,4],max:[5,5,5],geometry:undefined});restoreAssociation()');
assert.equal(run('boxes.length'),3);assert.equal(new Set(plain(run('boxes')).map(b=>b.id)).size,3);assert.equal(new Set(plain(run('boxes')).map(b=>b.category+':'+b.sequence)).size,3);
// Actual overlay wiring: rotated frame, axis comparison, support-only, no mutation.
ctx.panelBox={...box('screen',20,g.cells,'screen'),geometry:g,...C.bounds(g)};
run("resetDraft();boxes=[clone(panelBox)];selectRegion(boxes[0],false);let lines=[],cursor;Object.assign(pen,{clearRect(){},setLineDash(){},beginPath(){},moveTo(x,y){cursor=[x,y];},lineTo(x,y){lines.push([cursor,[x,y]]);},stroke(){},measureText(){return {width:50};},fillRect(){},fillText(){}});project=p=>({x:p[0],y:p[1],z:1});");
get('boundsMode').value='oriented';run('drawOverlay()');assert.equal(run('lines.length'),12);const obbLines=plain(run('lines'));
get('boundsMode').value='axis';run('lines=[];drawOverlay()');assert.equal(run('lines.length'),12);assert.notDeepEqual(plain(run('lines')),obbLines);
get('boundsMode').value='support';run('lines=[];drawOverlay()');assert.equal(run('lines.length'),0);
assert.deepEqual(plain(run('boxes[0]')),ctx.panelBox);
console.log(`PASS: v11 tilted screen OBB / AABB volume = ${ratio.toFixed(3)}; all voxel corners retained; degenerate/translated geometry; manual union/restore/split; source preservation, review reset, export, ID collisions, confirm/cancel/undo and overlay modes. No browser/GPU acceptance claimed.`);

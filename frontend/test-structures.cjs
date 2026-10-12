const fs=require('node:fs'),assert=require('node:assert/strict'),vm=require('node:vm'),path=require('node:path'),B=require('./boxes.js');
const root=path.resolve(__dirname,'../artifacts'),read=p=>JSON.parse(fs.readFileSync(path.join(root,p),'utf8'));
const meta=read('k2-annotation-scene-v2/scene.json'),doc=read('k2-annotation-structures-v2/proposals.json'),old=read('k2-annotation-proposals-v1/proposals.json');
const structures=B.validate(doc,meta);assert.equal(structures.length,25);assert.equal(structures.filter(b=>b.category==='floor').length,2);assert.deepEqual(B.validate(B.save(structures,meta),meta),structures);
let gapChecked=false;
for(const b of structures){const g=b.geometry;const [row,a,end]=g.runs[0];const world=(x,y)=>g.origin.map((o,i)=>o+(g.uv0[0]+(x+.5)*g.cell)*g.u[i]+(g.uv0[1]+(y+.5)*g.cell)*g.v[i]);assert(B.contains(b,...world(a,row)));
 for(let j=1;j<g.runs.length;j++){const left=g.runs[j-1],right=g.runs[j];if(left[0]===right[0]&&right[1]-left[2]>3){assert(!B.contains(b,...world(left[2]+1,left[0])));gapChecked=true;break;}}}
assert(gapChecked,'Must exercise a mesh-supported internal gap');
const bad=structuredClone(doc);bad.boxes[0].geometry.runs[0][2]=100000;assert.throws(()=>B.validate(bad,meta));
const elements=new Map(),element=id=>{if(!elements.has(id))elements.set(id,{value:'',textContent:'',getContext:()=>({})});return elements.get(id);};
const context={BoxCore:B,structuredClone,document:{getElementById:element},console,crypto:require('node:crypto').webcrypto,setTimeout,clearTimeout};vm.createContext(context);vm.runInContext(fs.readFileSync(__dirname+'/scene-app.js','utf8').replace(/init\(\);\s*$/,''),context);context.old=old.boxes;context.next=structures;
vm.runInContext(`legacyProposals=old;proposals=next;boxes=clone(old);boxes[0].status='reviewed';boxes[1].geometry.origin[0]+=.1;boxes[2].category='other';const update=proposalUpdate();if(update.remove!==old.length-3||update.kept.length!==3)throw Error('Manual edits not protected');if(update.fresh.length!==25)throw Error('Wrong update');boxes=clone(next);if(proposalUpdate().fresh.length!==0)throw Error('Update duplicated');`,context);
const html=fs.readFileSync(__dirname+'/workbench.html','utf8'),ids=[...html.matchAll(/\bid="([^"]+)"/g)].map(x=>x[1]);assert.equal(ids.length,new Set(ids).size,'Duplicate UI IDs');
console.log('PASS: 56 -> 25 structures, occupied/gap containment, roundtrip, invalid mask rejection, manual-change preservation, update idempotency, unique UI IDs');
